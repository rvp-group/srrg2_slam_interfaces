#pragma once
#include "local_map_splitting_criterion_base.hpp"

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;
  using namespace srrg2_solver;

  template <typename SlamAlgorithmType_>
  class LocalMapSplittingCriterionDistance_
    : public LocalMapSplittingCriterionBase_<SlamAlgorithmType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using EstimateType = typename SlamAlgorithmType_::LocalMapType::EstimateType;
    using ThisType     = LocalMapSplittingCriterionDistance_<SlamAlgorithmType_>;
    PARAM(PropertyFloat,
          local_map_distance,
          "distance between the center of the local maps (in meters)",
          1,
          nullptr);
    virtual ~LocalMapSplittingCriterionDistance_() {
    }

    virtual void compute() override {
      ThisType::_has_to_split = false;
      if (!ThisType::_slam) {
        throw std::runtime_error("LocalMapSplittingCriterionDistance|SLAM algorithm not set");
      }
      const EstimateType pose_in_map = ThisType::_slam->robotPoseInCurrentLocalMap();
      if (pose_in_map.translation().norm() > ThisType::param_local_map_distance.value()) {
        ThisType::_has_to_split = true;
      }
    }
  };

} // namespace srrg2_slam_interfaces
