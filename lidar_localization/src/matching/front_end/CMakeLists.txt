file(GLOB ALL_SOURCE "*.cpp")

add_library(MATCHING_FRONT_END SHARED
  ${ALL_SOURCE}
)

target_link_libraries(MATCHING_FRONT_END
  ${catkin_LIBRARIES}
  SUBSCRIBER
  PUBLISHER
  CLOUD_FILTER
  REGISTRATION
)