add_library(srrg2_slam_interfaces_library SHARED
  registration/aligners/aligner_slice_odometry_prior.cpp
  raw_data_preprocessors/raw_data_preprocessor_odom.cpp
  trackers/tracker.cpp
  mapping/local_map.cpp
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
