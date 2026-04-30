cmake_minimum_required(VERSION 3.8)
project(sim_bringup)

find_package(ament_cmake REQUIRED)

file(GLOB WORLD_NAMES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/worlds"
     "${CMAKE_CURRENT_SOURCE_DIR}/worlds/*")
foreach(WORLD ${WORLD_NAMES})
  set(WDIR "${CMAKE_CURRENT_SOURCE_DIR}/worlds/${WORLD}")
  if(IS_DIRECTORY "${WDIR}")
    install(
      DIRECTORY worlds/${WORLD}/
      DESTINATION share/${PROJECT_NAME}/worlds/${WORLD}
      PATTERN "photos" EXCLUDE
      PATTERN ".DS_Store" EXCLUDE
    )
    if(EXISTS "${WDIR}/photos")
      install(
        DIRECTORY worlds/${WORLD}/photos/
        DESTINATION share/${PROJECT_NAME}/worlds/photos
        PATTERN ".DS_Store" EXCLUDE
      )
    endif()
  endif()
endforeach()

install(
  DIRECTORY robots/
  DESTINATION share/${PROJECT_NAME}/robots
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
