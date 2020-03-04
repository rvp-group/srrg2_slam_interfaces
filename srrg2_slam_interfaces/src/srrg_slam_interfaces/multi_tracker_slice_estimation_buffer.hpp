#pragma once
#include "multi_tracker_slice_prior.h"

namespace srrg2_slam_interfaces {

  //! buffered tracker pose estimation as prior slice for e.g. fancy motion modelling
  template <typename EstimateType_, typename FixedType_, typename MovingType_>
  class MultiTrackerSliceEstimationBuffer_
    : public MultiTrackerSlicePrior_<EstimateType_, FixedType_, MovingType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using EstimateType = EstimateType_;
    using FixedType    = FixedType_;
    using MovingType   = MovingType_;
    using BaseType     = MultiTrackerSlicePrior_<EstimateType, FixedType, MovingType>;

    template <typename T>
    friend class MultiTrackerBase_;

  protected:
    void setEstimate(const EstimateType& estimate_) override {
      BaseType::setEstimate(estimate_);
    }

    void setScene(srrg2_core::PropertyContainerBase* scene_) override {
      BaseType::addSceneProperty(scene_);

      // ds if not relocalized (otherwise the new coordinate frame origin will be already set)
      if (!_relocalized) {
        // ds the tracker moved into a new local map (i.e. in which it's pose == identity)
        // ds to keep the motion model sane we have to shift it into the new local map
        _updated_tracker_estimate = BaseType::_tracker_estimate;
        _scene_changed            = true;
      }
    }

    void setClosure(const CorrespondenceVector& correspondences_,
                    const EstimateType& relative_transform_,
                    const EstimateType& tracker_estimate_) override {
      // ds the tracker moved into an older local map with a shifted pose (loop closure offset)
      // ds to keep the motion model sane we have to shift it into the local map + relocalization
      _updated_tracker_estimate = BaseType::_tracker_estimate * tracker_estimate_.inverse();
      _relocalized              = true;
    }

    void setMeasurement(srrg2_core::BaseSensorMessagePtr measurement_) override {
      // ds no actual measurement to process - just configure the adaptor
      assert(BaseType::param_adaptor.value());
      BaseType::addMeasurementProperty();
      BaseType::param_adaptor->setDest(&this->_adapted_slice);
    }

    void merge() override {
      BaseType::merge();

      // ds if we have a tracker estimate adaptor
      std::shared_ptr<MeasurementAdaptorTrackerEstimate_<FixedType>> tracker_estimate_adaptor =
        std::dynamic_pointer_cast<MeasurementAdaptorTrackerEstimate_<FixedType>>(
          BaseType::param_adaptor.value());
      if (tracker_estimate_adaptor) {
        if (_scene_changed || _relocalized) {
          // ds provide adaptor with tracker pose instead of a measurement
          // ds with re-centering after re-population (estimate reset)
          tracker_estimate_adaptor->setCoordinateFrameOrigin(_updated_tracker_estimate);
          _scene_changed = false;
          _relocalized   = false;
        }

        // ds set refined pose estimate that can be used in the next compute
        tracker_estimate_adaptor->setEstimate(BaseType::_tracker_estimate);
      }
    }

    bool isSceneSliceEmpty() const override {
      // ds this slice does not have an actual scene, but a transform instead
      // ds it is assumed to be not "empty" if the scene is set
      return (this->_scene_slice != nullptr);
    }

    // ds scene change (into another/new local map)
    bool _scene_changed = false;
    EstimateType _updated_tracker_estimate;

    // ds relocalization event, we keep the latest estimate to compute the local map transform
    bool _relocalized = false;
  };

  using MultiTrackerSliceEstimationBuffer2D =
    MultiTrackerSliceEstimationBuffer_<srrg2_core::Isometry2f,
                                       srrg2_core::StdDequeEigenIsometry2f,
                                       srrg2_core::StdDequeEigenIsometry2f>;
  using MultiTrackerSliceEstimationBuffer3D =
    MultiTrackerSliceEstimationBuffer_<srrg2_core::Isometry3f,
                                       srrg2_core::StdDequeEigenIsometry3f,
                                       srrg2_core::StdDequeEigenIsometry3f>;
  using MultiTrackerSliceEstimationBuffer2DPtr =
    std::shared_ptr<MultiTrackerSliceEstimationBuffer2D>;
  using MultiTrackerSliceEstimationBuffer3DPtr =
    std::shared_ptr<MultiTrackerSliceEstimationBuffer3D>;

} // namespace srrg2_slam_interfaces
