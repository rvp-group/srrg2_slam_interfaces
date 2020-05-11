#pragma once
#include "initializers/initializer_camera.h"
#include "mapping/local_map.h"
#include "mapping/local_map_splitting_criterions/local_map_splitting_criterion_rotation.hpp"
#include "mapping/local_map_splitting_criterions/local_map_splitting_criterion_translation.hpp"
#include "mapping/local_map_splitting_criterions/local_map_splitting_criterion_viewpoint.hpp"
#include "mapping/local_map_splitting_criterions/local_map_splitting_criterion_visibility.hpp"
#include "mapping/merger_correspondence_homo.h"
#include "motion_models/motion_model_constant_velocity.hpp"
#include "raw_data_preprocessors/raw_data_preprocessor_odom.h"
#include "raw_data_preprocessors/raw_data_preprocessor_tracker_estimate.hpp"
#include "registration/aligners/aligner_slice_motion_model.hpp"
#include "registration/aligners/aligner_slice_odometry_prior.h"
#include "registration/aligners/multi_aligner.h"
#include "registration/local_map_selectors/local_map_selector_breadth_first.h"
#include "registration/local_map_selectors/local_map_selector_user_defined.h"
#include "registration/loop_closure.h"
#include "registration/loop_detector/multi_loop_detector_brute_force.h"
#include "registration/loop_detector/multi_loop_detector_hbst.h"
#include "registration/relocalization/multi_relocalizer.h"
#include "system/multi_graph_slam.h"
#include "trackers/multi_tracker.h"
#include "trackers/tracker_slice_processor.h"
#include "trackers/tracker_slice_processor_estimation_buffer.hpp"
#include "trackers/tracker_slice_processor_odom.h"
#include <srrg_solver/solver_core/solver.h>
#include <srrg_solver/variables_and_factors/types_2d/se2_point2point_error_factor.h>
#include <srrg_solver/variables_and_factors/types_2d/se2_prior_error_factor.h>
#include <srrg_solver/variables_and_factors/types_3d/se3_point2point_error_factor.h>
#include <srrg_solver/variables_and_factors/types_3d/se3_prior_error_factor_ad.h>

namespace srrg2_slam_interfaces {
  using namespace srrg2_solver;

  // term crits for aligners
  using AlignerTerminationCriteriaStandard2D = AlignerTerminationCriteriaStandard_<MultiAligner2D>;
  using AlignerTerminationCriteriaStandard3DQR =
    AlignerTerminationCriteriaStandard_<MultiAligner3DQR>;

  // graph slams
  using MultiGraphSLAM2D    = MultiGraphSLAM_<LoopClosure2D>;
  using MultiGraphSLAM3D    = MultiGraphSLAM_<LoopClosure3D>;
  using MultiGraphSLAM2DPtr = std::shared_ptr<MultiGraphSLAM2D>;
  using MultiGraphSLAM3DPtr = std::shared_ptr<MultiGraphSLAM3D>;

  using MultiRelocalizer2D = MultiRelocalizer_<MultiGraphSLAM2D>;
  using MultiRelocalizer3D = MultiRelocalizer_<MultiGraphSLAM3D>;

  using LocalMapSelectorBreadthFirst2D = LocalMapSelectorBreadthFirst_<MultiGraphSLAM2D>;
  using LocalMapSelectorBreadthFirst3D = LocalMapSelectorBreadthFirst_<MultiGraphSLAM3D>;

  using LocalMapSplittingCriterionDistance2D =
    LocalMapSplittingCriterionDistance_<MultiGraphSLAM2D>;
  using LocalMapSplittingCriterionDistance3D =
    LocalMapSplittingCriterionDistance_<MultiGraphSLAM3D>;
  using LocalMapSplittingCriterionRotation2D =
    LocalMapSplittingCriterionRotation_<MultiGraphSLAM2D>;
  using LocalMapSplittingCriterionRotation3D =
    LocalMapSplittingCriterionRotation_<MultiGraphSLAM3D>;
  using LocalMapSplittingCriterionViewpoint3D =
    LocalMapSplittingCriterionViewpoint_<MultiGraphSLAM3D>;
  using LocalMapSplittingCriterionVisibility3D =
    LocalMapSplittingCriterionVisbility_<MultiGraphSLAM3D>;

  using MultiLoopDetectorBruteForce2D =
    MultiLoopDetectorBruteForce_<MultiGraphSLAM2D, MultiAligner2D>;
  using MultiLoopDetectorBruteForce3D =
    MultiLoopDetectorBruteForce_<MultiGraphSLAM3D, MultiAligner3DQR>;

  using MultiLoopDetectorHBST2D =
    MultiLoopDetectorHBST_<MultiGraphSLAM2D, MultiAligner2D, SE2Point2PointErrorFactor>;
  using MultiLoopDetectorHBST3D =
    MultiLoopDetectorHBST_<MultiGraphSLAM3D, MultiAligner3DQR, SE3Point2PointErrorFactor>;
  using MultiLoopDetectorHBST2DPtr = std::shared_ptr<MultiLoopDetectorHBST2D>;
  using MultiLoopDetectorHBST3DPtr = std::shared_ptr<MultiLoopDetectorHBST3D>;

  void srrg2_slam_interfaces_registerTypes() __attribute__((constructor));

} // namespace srrg2_slam_interfaces
