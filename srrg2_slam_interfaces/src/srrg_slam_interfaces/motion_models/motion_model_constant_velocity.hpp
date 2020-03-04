#pragma once
#include "motion_model_base.hpp"

namespace srrg2_slam_interfaces {

  template <typename EstimateType_>
  class MotionModelConstantVelocity : public MotionModelBase_<EstimateType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using EstimateType = EstimateType_;
    using ThisType     = MotionModelConstantVelocity<EstimateType>;
    virtual ~MotionModelConstantVelocity() {
    }

    void compute() override {
      ThisType::_estimate = _tracker_estimate.inverse() * _tracker_estimate_previous;
    }

    void addTrackerEstimate(const EstimateType& estimate_,
                            const double timestamp_seconds_ = 0.0) override {
      _tracker_estimate_previous = _tracker_estimate;
      _tracker_estimate          = estimate_;
    }

    // ds TODO depracted now, can be deleted as soon as MM slice is active
    void shiftTrackerEstimate(const EstimateType& estimate_,
                              const double timestamp_seconds_ = 0.0) override {
      // ds the previous will cancel out this tracker estimate in the next computation
      // ds TODO this is kind of hacky and hence should be purged with a cleaner alternative
      _tracker_estimate_previous =
        estimate_ * (_tracker_estimate.inverse() * _tracker_estimate_previous);
      _tracker_estimate = estimate_;
    }

    void setTrackerEstimatePrevious(const EstimateType& estimate_,
                                    const double timestamp_seconds_ = 0.0) override {
      _tracker_estimate_previous = estimate_;
    }

    void clear() override {
      ThisType::_estimate.setIdentity();
      _tracker_estimate.setIdentity();
      _tracker_estimate_previous.setIdentity();
    }

  protected:
    EstimateType _tracker_estimate          = EstimateType::Identity();
    EstimateType _tracker_estimate_previous = EstimateType::Identity();
  };
  
  using MotionModelConstantVelocity2D    = MotionModelConstantVelocity<srrg2_core::Isometry2f>;
  using MotionModelConstantVelocity3D    = MotionModelConstantVelocity<srrg2_core::Isometry3f>;
  using MotionModelConstantVelocity2DPtr = std::shared_ptr<MotionModelConstantVelocity2D>;
  using MotionModelConstantVelocity3DPtr = std::shared_ptr<MotionModelConstantVelocity3D>;

} // namespace srrg2_slam_interfaces
