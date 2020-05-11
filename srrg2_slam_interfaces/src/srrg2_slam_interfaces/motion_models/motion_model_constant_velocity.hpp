#pragma once
#include "motion_model_base.hpp"

namespace srrg2_slam_interfaces {

  template <typename EstimateType_>
  class MotionModelConstantVelocity : public MotionModelBase_<EstimateType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using EstimateType = EstimateType_;
    using ThisType     = MotionModelConstantVelocity<EstimateType>;
    using BaseType     = MotionModelBase_<EstimateType>;

    virtual ~MotionModelConstantVelocity() {
    }

    void compute() override {
      ThisType::_motion = _robot_in_local_map_previous.inverse() * _robot_in_local_map;
    }

    void setRobotInLocalMap(const EstimateType& robot_in_local_map_,
                            const double timestamp_seconds_ = 0.0) override {
      _robot_in_local_map_previous = _robot_in_local_map;
      _robot_in_local_map          = robot_in_local_map_;
    }

    // ds TODO depracted now, can be deleted as soon as MM slice is active
    void shiftTrackerEstimate(const EstimateType& estimate_,
                              const double timestamp_seconds_ = 0.0) override {
      // ds the previous will cancel out this tracker estimate in the next computation
      // ds TODO this is kind of hacky and hence should be purged with a cleaner alternative
      _robot_in_local_map_previous =
        estimate_ * (_robot_in_local_map_previous.inverse() * _robot_in_local_map);
      _robot_in_local_map = estimate_;
    }

    void setRobotInLocalMapPrevious(const EstimateType& robot_in_local_map_,
                                    const double timestamp_seconds_ = 0.0) override {
      _robot_in_local_map_previous = robot_in_local_map_;
    }

    void clear() override {
      BaseType::clear();
      _robot_in_local_map.setIdentity();
      _robot_in_local_map_previous.setIdentity();
    }

  protected:
    EstimateType _robot_in_local_map          = EstimateType::Identity();
    EstimateType _robot_in_local_map_previous = EstimateType::Identity();
  };

  using MotionModelConstantVelocity2D    = MotionModelConstantVelocity<srrg2_core::Isometry2f>;
  using MotionModelConstantVelocity3D    = MotionModelConstantVelocity<srrg2_core::Isometry3f>;
  using MotionModelConstantVelocity2DPtr = std::shared_ptr<MotionModelConstantVelocity2D>;
  using MotionModelConstantVelocity3DPtr = std::shared_ptr<MotionModelConstantVelocity3D>;

} // namespace srrg2_slam_interfaces
