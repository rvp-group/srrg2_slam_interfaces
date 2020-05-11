#pragma once
#include "tracker_slice_processor_prior.h"

namespace srrg2_slam_interfaces {

  //! buffered tracker pose estimation as prior slice for e.g. fancy motion modelling
  template <typename EstimateType_, typename FixedMeasurementType_>
  class TrackerSliceProcessorEstimationBuffer_
    : public TrackerSliceProcessorPrior_<EstimateType_, FixedMeasurementType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using EstimateType         = EstimateType_;
    using FixedMeasurementType = FixedMeasurementType_;
    using MovingType           = FixedMeasurementType;
    using ThisType = TrackerSliceProcessorEstimationBuffer_<EstimateType, FixedMeasurementType>;
    using BaseType = TrackerSliceProcessorPrior_<EstimateType, FixedMeasurementType>;
    template <typename T>
    friend class MultiTrackerBase_;

  protected:
    //    void setRobotInLocalMap(const EstimateType& estimate_) override {
    //      BaseType::setRobotInLocalMap(estimate_);
    //    }

    void setScene(srrg2_core::PropertyContainerBase* scene_) override {
      BaseType::addSceneProperty(scene_);

      // ds if not relocalized (otherwise the new coordinate frame origin will be already set)
      if (!_relocalized) {
        // ds the tracker moved into a new local map (i.e. in which it's pose == identity)
        // ds to keep the motion model sane we have to shift it into the new local map
        _robot_in_local_map_updated = ThisType::_robot_in_local_map;
        _scene_changed              = true;
      }
    }

    void setClosure(const srrg2_core::CorrespondenceVector& correspondences_,
                    const EstimateType& fixed_in_moving_,
                    const EstimateType& robot_in_moving_local_map_) override {
      // ds the tracker moved into an older local map with a shifted pose (loop closure offset)
      // ds to keep the motion model sane we have to shift it into the local map + relocalization
      _robot_in_local_map_updated =
        ThisType::_robot_in_local_map * robot_in_moving_local_map_.inverse();
      //      std::cerr << "_robot_in_local_map: \n" << ThisType::_robot_in_local_map.matrix() <<
      //      std::endl; std::cerr << "robot_in_moving_local_map_: \n"
      //                << robot_in_moving_local_map_.matrix() << std::endl;
      //      std::cerr << "_robot_in_local_map_updated: \n"
      //                << _robot_in_local_map_updated.matrix() << std::endl;
      _relocalized = true;
    }

    void setRawData(srrg2_core::BaseSensorMessagePtr measurement_) override {
      // ds no actual measurement to process - just configure the adaptor
      assert(ThisType::param_adaptor.value());
      ThisType::addMeasurementProperty();
      ThisType::param_adaptor->setMeas(&(ThisType::_measurement_slice));
    }

    void merge() override {
      BaseType::merge();

      // ds if we have a tracker estimate adaptor
      std::shared_ptr<RawDataPreprocessorTrackerEstimate_<FixedMeasurementType>>
        tracker_estimate_adaptor =
          std::dynamic_pointer_cast<RawDataPreprocessorTrackerEstimate_<FixedMeasurementType>>(
            ThisType::param_adaptor.value());
      if (tracker_estimate_adaptor) {
        if (_scene_changed || _relocalized) {
          // ds provide adaptor with tracker pose instead of a measurement
          // ds with re-centering after re-population (estimate reset)
          tracker_estimate_adaptor->setCoordinateFrameOrigin(_robot_in_local_map_updated);
          _scene_changed = false;
          _relocalized   = false;
        }

        // ds set refined pose estimate that can be used in the next compute
        tracker_estimate_adaptor->setRobotInLocalMap(ThisType::_robot_in_local_map);
      }
    }

    bool isSceneSliceEmpty() const override {
      // ds this slice does not have an actual scene, but a transform instead
      // ds it is assumed to be not "empty" if the scene is set
      return (ThisType::_scene_slice != nullptr);
    }

    // ds scene change (into another/new local map)
    bool _scene_changed = false;
    EstimateType _robot_in_local_map_updated;

    // ds relocalization event, we keep the latest estimate to compute the local map transform
    bool _relocalized = false;
  };

  using TrackerSliceProcessorEstimationBuffer2D =
    TrackerSliceProcessorEstimationBuffer_<srrg2_core::Isometry2f,
                                           srrg2_core::StdDequeEigenIsometry2f>;
  using TrackerSliceProcessorEstimationBuffer3D =
    TrackerSliceProcessorEstimationBuffer_<srrg2_core::Isometry3f,
                                           srrg2_core::StdDequeEigenIsometry3f>;
  using TrackerSliceProcessorEstimationBuffer2DPtr =
    std::shared_ptr<TrackerSliceProcessorEstimationBuffer2D>;
  using TrackerSliceProcessorEstimationBuffer3DPtr =
    std::shared_ptr<TrackerSliceProcessorEstimationBuffer3D>;

} // namespace srrg2_slam_interfaces
