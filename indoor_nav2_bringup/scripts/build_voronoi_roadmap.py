#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.ndimage import binary_erosion
from scipy.spatial import Voronoi


class Graph:

    def __init__(self) -> None:
        self._attrs: dict = {}
        self._adj: dict = {}

    # node ops ---------------------------------------------------------------
    def add_node(self, node, **attrs) -> None:
        if node not in self._attrs:
            self._attrs[node] = dict(attrs)
            self._adj[node] = {}
        else:
            self._attrs[node].update(attrs)

    def __contains__(self, node) -> bool:
        return node in self._attrs

    @property
    def nodes(self):
        return self._attrs

    def number_of_nodes(self) -> int:
        return len(self._attrs)

    def degree(self, node) -> int:
        return len(self._adj.get(node, {}))

    def neighbors(self, node):
        return list(self._adj.get(node, {}).keys())

    # edge ops ---------------------------------------------------------------
    def add_edge(self, u, v, **attrs) -> None:
        if u not in self._attrs:
            self.add_node(u)
        if v not in self._attrs:
            self.add_node(v)
        self._adj[u][v] = dict(attrs)
        self._adj[v][u] = dict(attrs)

    def has_edge(self, u, v) -> bool:
        return v in self._adj.get(u, {})

    @property
    def edges(self):
        seen = set()
        out = []
        for u, nbrs in self._adj.items():
            for v, attrs in nbrs.items():
                key = (u, v) if id(u) <= id(v) else (v, u)
                if (u, v) in seen or (v, u) in seen:
                    continue
                seen.add(key)
                out.append((u, v, attrs))
        return out

    def edge_data(self, u, v) -> dict:
        return self._adj[u][v]

    def number_of_edges(self) -> int:
        return sum(len(n) for n in self._adj.values()) // 2


# ---------------------------------------------------------------------------
# Map loading
# ---------------------------------------------------------------------------

def load_nav2_map(yaml_path: Path) -> dict:
    with yaml_path.open('r', encoding='utf-8') as f:
        meta = yaml.safe_load(f)

    image_rel = meta['image']
    pgm_path = (yaml_path.parent / image_rel).resolve()
    if not pgm_path.exists():
        raise FileNotFoundError(f'pgm referenced by {yaml_path} not found: {pgm_path}')

    pgm = cv2.imread(str(pgm_path), cv2.IMREAD_GRAYSCALE)
    if pgm is None:
        raise IOError(f'failed to read {pgm_path}')

    # Match Nav2 map_server's bottom-left origin: image row 0 -> world y_max,
    # but the algorithm wants row 0 -> world y_min so that y = row*res + origin_y.
    pgm = np.flipud(pgm).copy()

    resolution = float(meta['resolution'])
    origin = list(meta['origin'])
    negate = int(meta.get('negate', 0))
    occ_thresh = float(meta.get('occupied_thresh', 0.65))
    free_thresh = float(meta.get('free_thresh', 0.196))

    # Standard map_server probability decoding.
    if negate:
        p = pgm.astype(np.float32) / 255.0
    else:
        p = (255.0 - pgm.astype(np.float32)) / 255.0

    occupied = p > occ_thresh
    free = p < free_thresh
    unknown = ~(occupied | free)

    return {
        'pgm': pgm,
        'resolution': resolution,
        'origin': origin,
        'occupied': occupied,
        'free': free,
        'unknown': unknown,
    }


# ---------------------------------------------------------------------------
# Free-map preparation
# ---------------------------------------------------------------------------

def build_free_map(map_data: dict, robot_radius: float) -> np.ndarray:
    """Inflate occupied + unknown by the robot radius and return the largest
    connected free-space component as a uint8 0/1 mask."""
    resolution = map_data['resolution']
    occupied = map_data['occupied']
    free = map_data['free']
    unknown = map_data['unknown']

    # Inflate everything that is not provably free. Treating unknown as
    # blocked is conservative for a static prior map.
    blocked = (occupied | unknown).astype(np.uint8)
    cells = max(1, int(math.ceil(robot_radius / resolution)))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * cells + 1, 2 * cells + 1))
    blocked_inflated = cv2.dilate(blocked, kernel, iterations=1)

    free_map = (free.astype(np.uint8) & (1 - blocked_inflated)).astype(np.uint8)

    # Keep only the largest connected component to avoid disconnected pockets.
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(free_map, 8)
    if num_labels <= 1:
        return free_map
    areas = stats[1:, cv2.CC_STAT_AREA]
    largest = 1 + int(np.argmax(areas))
    return (labels == largest).astype(np.uint8)


# ---------------------------------------------------------------------------
# Voronoi roadmap
# ---------------------------------------------------------------------------

def build_voronoi_graph(free_map: np.ndarray, resolution: float,
                        origin_xy: tuple) -> Graph:
    """Voronoi medial axis of `free_map` -> Graph.

    Nodes carry world (x, y) in `pos`. Grid coordinates are stored in `grid`
    (row, col). Edge weight `dist` is Euclidean distance in metres.
    """
    free_bin = (free_map > 0).astype(np.uint8)
    eroded = binary_erosion(free_bin, iterations=1).astype(np.uint8)
    boundary = (free_bin - eroded).astype(np.uint8)

    rows, cols = np.where(boundary == 1)
    if len(rows) < 4:
        raise ValueError('not enough boundary points to build a Voronoi diagram')

    points = np.column_stack((rows, cols))
    vor = Voronoi(points)

    h, w = free_map.shape
    g = Graph()

    def add_node_if_needed(idx: int) -> tuple:
        v = vor.vertices[idx]
        node_id = (float(v[0]), float(v[1]))
        if node_id in g:
            return node_id
        # World coordinates: row -> y, col -> x. (Free map is already flipped
        # so row 0 == world y = origin_y.)
        x = v[1] * resolution + origin_xy[0]
        y = v[0] * resolution + origin_xy[1]
        g.add_node(node_id, pos=(x, y), grid=(float(v[0]), float(v[1])))
        return node_id

    for simplex in vor.ridge_vertices:
        a, b = simplex
        if a < 0 or b < 0:
            continue
        va = vor.vertices[a]
        vb = vor.vertices[b]
        if not (0 <= va[0] < h and 0 <= va[1] < w and 0 <= vb[0] < h and 0 <= vb[1] < w):
            continue
        ar, ac = int(va[0]), int(va[1])
        br, bc = int(vb[0]), int(vb[1])
        if free_map[ar, ac] == 0 or free_map[br, bc] == 0:
            continue
        ua = add_node_if_needed(a)
        ub = add_node_if_needed(b)
        if not g.has_edge(ua, ub):
            dx = (vb[1] - va[1]) * resolution
            dy = (vb[0] - va[0]) * resolution
            g.add_edge(ua, ub, dist=float(math.hypot(dx, dy)))
    return g


def sparsify_graph(graph: Graph, resampling_dist: float) -> Graph:
    """Drop most degree-2 chain nodes, keeping resampled waypoints every
    ``resampling_dist`` metres so straight stretches stay represented."""
    if graph.number_of_nodes() < 10:
        return graph

    key_nodes = {n for n in graph.nodes if graph.degree(n) != 2}
    if not key_nodes:
        return graph

    new_g = Graph()
    for n in key_nodes:
        new_g.add_node(n, **graph.nodes[n])

    visited = set()
    for start in key_nodes:
        for neighbor in graph.neighbors(start):
            if (start, neighbor) in visited:
                continue
            path = [start, neighbor]
            visited.add((start, neighbor))
            prev, curr = start, neighbor
            while graph.degree(curr) == 2:
                nxts = [x for x in graph.neighbors(curr) if x != prev]
                if len(nxts) != 1:
                    break
                nxt = nxts[0]
                if (curr, nxt) in visited:
                    break
                path.append(nxt)
                visited.add((curr, nxt))
                prev, curr = curr, nxt

            edge_dists = [graph.edge_data(path[i], path[i + 1])['dist']
                          for i in range(len(path) - 1)]

            predecessor = path[0]
            if predecessor not in new_g:
                new_g.add_node(predecessor, **graph.nodes[predecessor])

            agg = 0.0
            for i in range(1, len(path) - 1):
                agg += edge_dists[i - 1]
                cand = path[i]
                if cand in key_nodes or agg >= resampling_dist:
                    if cand not in new_g:
                        new_g.add_node(cand, **graph.nodes[cand])
                    p_pos = np.asarray(new_g.nodes[predecessor]['pos'])
                    c_pos = np.asarray(new_g.nodes[cand]['pos'])
                    if not new_g.has_edge(predecessor, cand):
                        new_g.add_edge(predecessor, cand,
                                       dist=float(np.linalg.norm(p_pos - c_pos)))
                    predecessor = cand
                    agg = 0.0

            end = path[-1]
            if end not in new_g:
                new_g.add_node(end, **graph.nodes[end])
            if predecessor != end and not new_g.has_edge(predecessor, end):
                p_pos = np.asarray(new_g.nodes[predecessor]['pos'])
                e_pos = np.asarray(new_g.nodes[end]['pos'])
                new_g.add_edge(predecessor, end,
                               dist=float(np.linalg.norm(p_pos - e_pos)))
    return new_g


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------

def graph_to_json_payload(graph: Graph, *, source_map: str, frame_id: str,
                          resolution: float, origin: list,
                          robot_radius: float) -> dict:
    id_of = {n: i for i, n in enumerate(graph.nodes)}
    nodes = []
    for n, idx in id_of.items():
        x, y = graph.nodes[n]['pos']
        nodes.append({'id': idx, 'x': float(x), 'y': float(y)})

    edges = []
    for u, v, data in graph.edges:
        edges.append({
            'u': id_of[u],
            'v': id_of[v],
            'dist': float(data['dist']),
        })

    return {
        'meta': {
            'frame_id': frame_id,
            'source_map': source_map,
            'resolution': resolution,
            'origin': list(origin),
            'robot_radius_used': robot_radius,
            'version': 1,
        },
        'nodes': nodes,
        'edges': edges,
    }


def write_debug_images(out_dir: Path, free_map: np.ndarray, graph: Graph,
                       sparse: Graph) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_dir / 'free_map.png'), free_map * 255)

    h, _ = free_map.shape

    def render(g: Graph, path: Path, color_edge, color_node) -> None:
        # Render in image space (row 0 at top), so flip to match how a user
        # would compare against the original PGM.
        img = cv2.cvtColor(np.flipud(free_map) * 255, cv2.COLOR_GRAY2BGR)
        for u, v, _data in g.edges:
            ur, uc = u
            vr, vc = v
            p1 = (int(round(uc)), h - 1 - int(round(ur)))
            p2 = (int(round(vc)), h - 1 - int(round(vr)))
            cv2.line(img, p1, p2, color_edge, 1)
        for n in g.nodes:
            r, c = n
            p = (int(round(c)), h - 1 - int(round(r)))
            cv2.circle(img, p, 1, color_node, -1)
        cv2.imwrite(str(path), img)

    render(graph, out_dir / 'voronoi_raw.png', (0, 0, 200), (200, 0, 0))
    render(sparse, out_dir / 'voronoi_sparse.png', (0, 0, 200), (200, 0, 0))


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True, type=Path,
                        help='Path to map.yaml (Nav2 map_server format).')
    parser.add_argument('--output', required=True, type=Path,
                        help='Path to write the roadmap JSON.')
    parser.add_argument('--robot-radius', type=float, default=0.12,
                        help='Robot radius in metres for build-time inflation. Default: 0.12 (TB3 Burger).')
    parser.add_argument('--resampling-dist', type=float, default=0.4,
                        help='Sparsifier waypoint spacing along chains, in metres.')
    parser.add_argument('--frame-id', default='map',
                        help='Frame the roadmap coordinates are expressed in.')
    parser.add_argument('--debug', action='store_true',
                        help='Also write debug images next to the output JSON.')
    args = parser.parse_args()

    if not args.map.exists():
        print(f'[ERROR] map yaml not found: {args.map}', file=sys.stderr)
        return 1

    map_data = load_nav2_map(args.map)
    free_map = build_free_map(map_data, args.robot_radius)

    if free_map.sum() == 0:
        print('[ERROR] free map is empty after inflation -- robot_radius too large?',
              file=sys.stderr)
        return 2

    origin_xy = (map_data['origin'][0], map_data['origin'][1])
    raw = build_voronoi_graph(free_map, map_data['resolution'], origin_xy)
    sparse = sparsify_graph(raw, args.resampling_dist)

    payload = graph_to_json_payload(
        sparse,
        source_map=args.map.name,
        frame_id=args.frame_id,
        resolution=map_data['resolution'],
        origin=map_data['origin'],
        robot_radius=args.robot_radius,
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('w', encoding='utf-8') as f:
        json.dump(payload, f, indent=2)

    print(f'[INFO] raw graph: {raw.number_of_nodes()} nodes / {raw.number_of_edges()} edges')
    print(f'[INFO] sparse graph: {sparse.number_of_nodes()} nodes / {sparse.number_of_edges()} edges')
    print(f'[INFO] wrote: {args.output}')

    if args.debug:
        write_debug_images(args.output.parent / f'{args.output.stem}_debug',
                           free_map, raw, sparse)

    return 0


if __name__ == '__main__':
    sys.exit(main())
