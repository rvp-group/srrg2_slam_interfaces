catkin_add_gtest(test_motion_model test_motion_model.cpp)
target_link_libraries(test_motion_model
  ${catkin_LIBRARIES}
  srrg2_slam_interfaces_library
)

catkin_add_gtest(test_motion_model_slice test_motion_model_slice.cpp)
target_link_libraries(test_motion_model_slice
  ${catkin_LIBRARIES}
  srrg2_slam_interfaces_library
)

catkin_add_gtest(test_local_map_splitting_criterion test_local_map_splitting_criterion.cpp)
target_link_libraries(test_local_map_splitting_criterion
  ${catkin_LIBRARIES}
  srrg2_slam_interfaces_library
)
