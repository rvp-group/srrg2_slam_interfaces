#pragma once
#include "tracker_slice_processor_base.h"

namespace srrg2_slam_interfaces {

  /**
   * @brief Tracker slice processor prior class
   * Handles the registration process for prior measurement (e.g. odometry, IMU)
   */
  template <typename EstimateType_, typename FixedMeasurementType_>
  class TrackerSliceProcessorPrior_ : public TrackerSliceProcessorStandard_<EstimateType_,
                                                                            FixedMeasurementType_,
                                                                            FixedMeasurementType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using EstimateType         = EstimateType_;
    using FixedMeasurementType = FixedMeasurementType_; // the measurements remapped
    using MovingSceneType      = FixedMeasurementType;  // the scene
    using ThisType             = TrackerSliceProcessorPrior_<EstimateType, FixedMeasurementType>;

    template <typename T>
    friend class MultiTrackerBase_;

  protected:
    inline void merge() override {
      *(ThisType::_scene_slice) = ThisType::_measurement_slice;
    }
    inline void clip() override {
      ThisType::_clipped_scene_slice = *(ThisType::_scene_slice);
    }

    void enhanceSceneProperty(Property_<MovingSceneType*>* p_) override {
    }
  };

} // namespace srrg2_slam_interfaces
