#pragma once
#include "local_map_splitting_criterion_base.hpp"

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;
  using namespace srrg2_solver;

  // ds some diamond inheritance would've been nice but we keep the mess away from serialization
  template <typename SlamAlgorithmType_>
  class LocalMapSplittingCriterionViewpoint_
    : public LocalMapSplittingCriterionBase_<SlamAlgorithmType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using EstimateType    = typename SlamAlgorithmType_::LocalMapType::EstimateType;
    using TranslationType = LocalMapSplittingCriterionDistance_<SlamAlgorithmType_>;
    using RotationType    = LocalMapSplittingCriterionRotation_<SlamAlgorithmType_>;
    using ThisType        = LocalMapSplittingCriterionViewpoint_<SlamAlgorithmType_>;

    PARAM(PropertyFloat,
          local_map_distance,
          "distance between the center of the local maps (in meters)",
          1,
          nullptr);
    PARAM(PropertyFloat,
          local_map_angle_distance_radians,
          "rotation difference between the center of the local maps (in radians)",
          0.5 /*approximately 30 degrees*/,
          nullptr);

    virtual ~LocalMapSplittingCriterionViewpoint_() {
    }

    virtual void compute() override {
      ThisType::_has_to_split = false;
      if (!ThisType::_slam) {
        throw std::runtime_error("LocalMapSplittingCriterionViewpoint|SLAM algorithm not set");
      }
      criterion_translation.setSlamAlgorithm(ThisType::_slam);
      criterion_translation.param_local_map_distance.setValue(param_local_map_distance.value());
      criterion_translation.compute();
      if (criterion_translation.hasToSplit()) {
        ThisType::_has_to_split = true;
        return;
      }
      criterion_rotation.setSlamAlgorithm(ThisType::_slam);
      criterion_rotation.param_local_map_angle_distance_radians.setValue(
        param_local_map_angle_distance_radians.value());
      criterion_rotation.compute();
      if (criterion_rotation.hasToSplit()) {
        ThisType::_has_to_split = true;
        return;
      }
    }

  protected:
    // ds aggregation for now (not configurable)
    TranslationType criterion_translation;
    RotationType criterion_rotation;
  };

} // namespace srrg2_slam_interfaces
