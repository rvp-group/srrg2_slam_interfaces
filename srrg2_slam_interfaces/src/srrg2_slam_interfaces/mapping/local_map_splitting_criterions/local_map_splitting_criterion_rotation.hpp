#pragma once
#include "local_map_splitting_criterion_base.hpp"
#include <srrg_geometry/geometry_defs.h>

namespace srrg2_slam_interfaces {

  /**
   * @brief Rotation based splitting criterion
   */
  template <typename SlamAlgorithmType_>
  class LocalMapSplittingCriterionRotation_
    : public LocalMapSplittingCriterionBase_<SlamAlgorithmType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using EstimateType                = typename SlamAlgorithmType_::LocalMapType::EstimateType;
    static constexpr size_t Dimension = EstimateType::Dim;
    static constexpr size_t AngularDimension = (EstimateType::Dim == 2) ? 1 : EstimateType::Dim;
    using VectorType                         = srrg2_core::Vector_<float, AngularDimension>;
    using MatrixType                         = srrg2_core::MatrixN_<float, Dimension>;
    using ThisType = LocalMapSplittingCriterionRotation_<SlamAlgorithmType_>;
    PARAM(srrg2_core::PropertyFloat,
          local_map_angle_distance_radians,
          "rotation difference between the center of the local maps (in radians)",
          0.5 /*approximately 30 degrees*/,
          nullptr);
    virtual ~LocalMapSplittingCriterionRotation_() {
    }

    virtual void compute() override {
      ThisType::_has_to_split = false;
      if (!ThisType::_slam) {
        throw std::runtime_error("LocalMapSplittingCriterionRotation|SLAM algorithm not set");
      }
      const EstimateType pose_in_map    = ThisType::_slam->robotInLocalMap();
      const MatrixType& rotation_matrix = pose_in_map.linear();
      const float angular_delta         = _rotation2EulerDeltaChecked(rotation_matrix);
      if (angular_delta > param_local_map_angle_distance_radians.value()) {
        ThisType::_has_to_split = true;
      }
    }

  protected:
    // ds ugly ugly ugly local helper functions to have a human-understandable parameter
    // ds instead of sweet quaternions
    inline float _rotation2EulerDeltaChecked(const srrg2_core::Matrix2f& rotation_) const {
      return std::fabs(atan2(rotation_(1, 0), rotation_(0, 0)));
    }
    inline float _rotation2EulerDeltaChecked(const srrg2_core::Matrix3f& rotation_) const {
      srrg2_core::Vector3f delta = rotation_.eulerAngles(0, 1, 2);
      for (size_t i = 0; i < 3; ++i) {
        const double absolute_angle = std::fabs(delta[i]);
        assert(absolute_angle <= M_PI + 1e-5 /*hi float*/);
        // ds FIXME this is only valid for small angular changes
        delta[i] = std::min(absolute_angle, M_PI + 1e-5 /*hi float*/ - absolute_angle);
      }
      return delta.norm();
    }
  };

} // namespace srrg2_slam_interfaces
