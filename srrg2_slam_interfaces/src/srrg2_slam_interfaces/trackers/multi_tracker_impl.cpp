#include "multi_tracker.h"

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;

  template <typename EstimateType_>
  void
  MultiTrackerBase_<EstimateType_>::setRobotInLocalMap(const EstimateType_& robot_in_local_map_) {
    PROFILE_TIME("MultiTrackerBase::setRobotInLocalMap");
    _robot_in_local_map = robot_in_local_map_;

    // ds update tracker slice estimates as well
    for (size_t index_slice = 0; index_slice < param_slice_processors.size(); ++index_slice) {
      SliceProcessorTypePtr slice = param_slice_processors.value(index_slice);
      slice->setRobotInLocalMap(_robot_in_local_map);
    }
  }

  template <typename EstimateType_>
  void MultiTrackerBase_<EstimateType_>::populateScene(PropertyContainerDynamic& container_) {
    PROFILE_TIME("MultiTrackerBase::populateScene");
    for (size_t sit = 0; sit < param_slice_processors.size(); ++sit) {
      SliceProcessorTypePtr proc = param_slice_processors.value(sit);
      proc->populateScene(container_);
    }
  }

  template <typename EstimateType_>
  void
  MultiTrackerBase_<EstimateType_>::setClosure(const CorrespondenceVector& correspondences_,
                                               const EstimateType& reference_in_query_,
                                               const EstimateType& robot_in_moving_local_map_) {
    PROFILE_TIME("MultiTrackerBase::setClosure");

    // ds provide slice with current tracker estimate and closure data (correspondences for merging)
    for (size_t i = 0; i < param_slice_processors.size(); ++i) {
      SliceProcessorTypePtr slice = param_slice_processors.value(i);
      slice->setClosure(correspondences_, reference_in_query_, robot_in_moving_local_map_);
    }
  }

  template <typename EstimateType_>
  void MultiTrackerBase_<EstimateType_>::setPlatform(PlatformPtr platform_) {
    PROFILE_TIME("MultiTrackerBase::setPlatform");
    BaseType::setPlatform(platform_);
    if (param_aligner.value()) {
      param_aligner->setPlatform(platform_);
    }
    for (size_t i = 0; i < param_slice_processors.size(); ++i) {
      SliceProcessorTypePtr slice = param_slice_processors.value(i);
      assert(slice);
      slice->setPlatform(platform_);
    }
  }

  template <typename EstimateType_>
  void MultiTrackerBase_<EstimateType_>::preprocessRawData() {
    PROFILE_TIME("MultiTrackerBase::preprocessRawData");
    bool scene_empty = true;
    for (size_t sit = 0; sit < param_slice_processors.size(); ++sit) {
      SliceProcessorTypePtr proc = param_slice_processors.value(sit);
      proc->setRobotInLocalMap(_robot_in_local_map /*consider the same point set as before*/);
      proc->adapt();
      // srrg the priors are always good -> the system will never be in initializing state -> the
      // tracker will get lost for the first frame since the odom is there, but the aligner will not
      // be able to align the current meas with the empty scene We need to have a better handling of
      // this case, maybe an initialized case for the aligner too or sliced iteration stats such
      // that every slice has its own min_num_inliers and combine them
      if (std::dynamic_pointer_cast<TrackerSliceProcessorPrior_<EstimateType, EstimateType>>(
            proc)) {
        continue;
      }
      scene_empty &= proc->isSceneSliceEmpty();
    }
    if (scene_empty) {
      this->_status = TrackerBase::Initializing;
    } else {
      this->_status = TrackerBase::Initialized;
    }
  }

  template <typename EstimateType_>
  void MultiTrackerBase_<EstimateType_>::align() {
    PROFILE_TIME("MultiTrackerBase::align");
    if (this->_status != TrackerBase::Initialized) {
      return;
    }
    // do the clipping
    for (size_t sit = 0; sit < param_slice_processors.size(); ++sit) {
      SliceProcessorTypePtr proc = param_slice_processors.value(sit);
      proc->setRobotInLocalMap(_robot_in_local_map /*consider the same point set as before*/);
      proc->clip();
    }

    // ds configure aligner
    AlignerTypePtr aligner = param_aligner.value();
    aligner->setFixed(&_measurement_container);
    aligner->setMoving(&_clipped_scene_container);

    // ds set initial guess (if set externally)
    // ds this guess can be overwritten by aligner slices in aligner->compute()
    aligner->setMovingInFixed(_moving_in_fixed_guess);

    // ds compute relative transform between all fixed and moving slices
    aligner->compute();

    // ds store correspondences for merging
    aligner->storeCorrespondences();
    // ds evaluate tracker status based on aligner performance
    if (param_aligner->status() == AlignerBase::Success) {
      // ds update current tracker and slice pose estimates
      // srrg for the tracker the local map is fixed and the robot is moving
      _updateRobotInLocalMap(aligner->movingInFixed());
      this->_status = TrackerBase::Tracking;
    } else {
      std::cerr << FG_RED("TrackerStandard_::align|Track Lost - aligner fail") << std::endl;
      std::cerr << FG_RED("                       |Aligner state code: " << aligner->status())
                << std::endl;
      this->_status = TrackerBase::Lost;

      // ds tracker estimate is not updated TODO @mc correct?
    }
  }

  template <typename EstimateType_>
  void MultiTrackerBase_<EstimateType_>::merge() {
    PROFILE_TIME("MultiTrackerBase::merge");
    //    const EstimateType_ tracker_from_scene(_robot_in_local_map.inverse());
    for (size_t index_slice = 0; index_slice < param_slice_processors.size(); ++index_slice) {
      SliceProcessorTypePtr slice = param_slice_processors.value(index_slice);
      slice->setRobotInLocalMap(_robot_in_local_map);
      slice->setRobotInWorld(_local_map_in_world * _robot_in_local_map);
      slice->merge();
    }

    // ds last step in tracking (afaik), count frame as processed
    ++BaseType::_num_frames_processed;
  }

  template <typename EstimateType_>
  void MultiTrackerBase_<EstimateType_>::setRawData(BaseSensorMessagePtr message_) {
    PROFILE_TIME("MultiTrackerBase::setRawData");
    BaseType::setRawData(message_);
    _measurement_container.clear();
    for (size_t i = 0; i < param_slice_processors.size(); ++i) {
      SliceProcessorTypePtr proc = param_slice_processors.value(i);
      proc->bindMeasurements(_measurement_container);
      proc->setRawData(message_);
    }
  }

  template <typename EstimateType_>
  void MultiTrackerBase_<EstimateType_>::setScene(SceneContainerType* scene_) {
    PROFILE_TIME("MultiTrackerBase::setScene");
    _clipped_scene_container.clear();
    bool scene_empty = true;
    for (size_t sit = 0; sit < param_slice_processors.size(); ++sit) {
      SliceProcessorTypePtr proc = param_slice_processors.value(sit);
      proc->bindClippedScene(_clipped_scene_container);
      proc->setScene(scene_);
      if (std::dynamic_pointer_cast<TrackerSliceProcessorPrior_<EstimateType, EstimateType>>(
            proc)) {
        continue;
      }
      scene_empty &= proc->isSceneSliceEmpty();
    }
    if (scene_empty) {
      this->_status = TrackerBase::Initializing;
    } else {
      this->_status = TrackerBase::Initialized;
    }
  }

  template <typename EstimateType_>
  void MultiTrackerBase_<EstimateType_>::draw(srrg2_core::ViewerCanvasPtr canvas_) const {
    assert(param_aligner.value());
    param_aligner->draw(canvas_);

    // ia recursively call all the tr-slice draw
    for (size_t sit = 0; sit < param_slice_processors.size(); ++sit) {
      const SliceProcessorTypePtr& proc = param_slice_processors.value(sit);
      proc->draw(canvas_);
    }
  }

} // namespace srrg2_slam_interfaces
