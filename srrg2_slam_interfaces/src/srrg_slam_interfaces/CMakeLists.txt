add_library(srrg2_slam_interfaces_library SHARED
  aligner.cpp
  correspondence_finder.cpp
  merger.cpp
  merger_correspondence_homo.cpp
  measurement_adaptor_odom.cpp
  tracker.cpp
  local_map.cpp
  instances.cpp
)

target_link_libraries(srrg2_slam_interfaces_library
  srrg2_solver_core_library
  srrg2_solver_types2d_library
  srrg2_solver_types3d_library
  srrg2_solver_factor_graph_utils_library
  srrg2_system_utils_library
  srrg2_messages_library
  ${caktin_LIBRARIES}
  ${OpenCV_LIBRARIES} #ds TODO burst once raster images are handled by our viewer
)
