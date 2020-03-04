#pragma once
#include "multi_tracker_slice.h"

namespace srrg2_slam_interfaces {

  template <typename EstimateType_, typename FixedMeasurementType_, typename MovingSceneType_>
  class MultiTrackerSlicePrior_ : public MultiTrackerSliceBase_<EstimateType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using EstimateType           = EstimateType_;
    using FixedType              = FixedMeasurementType_; // the measurements remapped
    using MovingSceneType        = MovingSceneType_;      // the scene
    using MeasurementAdaptorType = MeasurementAdaptor_<FixedType>;
    using MergerType             = Merger_<EstimateType, FixedType, MovingSceneType>;
    using ClipperType            = SceneClipper_<EstimateType, MovingSceneType>;
    using BaseType               = MultiTrackerSliceBase_<EstimateType_>;

    template <typename T>
    friend class MultiTrackerBase_;

    // ds measurement adaptor for odometry retrieval
    // ds this slice is also (ab)used for pumping in poses for the motion model aligner slice
    PARAM(PropertyConfigurable_<MeasurementAdaptorType>,
          adaptor,
          "measurement adaptor used in the slice",
          nullptr,
          nullptr);

  protected:
    void sanityCheck(const std::string& message = "MultiTrackerSlicePrior_") const;
    void merge() override;
    void adapt() override;
    void clip() override {
      _clipped_scene_slice = *_scene_slice;
    }
    bool isSceneSliceEmpty() const override {
      return (param_adaptor->status() == MeasurementAdaptorType::Ready);
    }
    void setScene(srrg2_core::PropertyContainerBase* scene_) override;
    void setMeasurement(srrg2_core::BaseSensorMessagePtr meas) override;
    void populateScene(srrg2_core::PropertyContainerDynamic& container) override;

    void addMeasurementProperty();
    void addSceneProperty(srrg2_core::PropertyContainerBase* scene_) override;

    // this is where the adaptors write the converted measurements
    FixedType _adapted_slice;

    // this is where the new scene gets mapped (extracting fields from local map)
    MovingSceneType* _scene_slice = nullptr;
    MovingSceneType _static_scene_slice;

    // this is where the local map gets clipped during the alignment
    MovingSceneType _clipped_scene_slice;
  };

} // namespace srrg2_slam_interfaces
