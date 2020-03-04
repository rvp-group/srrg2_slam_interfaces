#include "multi_tracker_slice.h"

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;
  using namespace srrg2_solver;

  template <typename EstimateType_, typename FixedMeasurementType_, typename MovingSceneType_>
  void MultiTrackerSlice_<EstimateType_, FixedMeasurementType_, MovingSceneType_>::populateScene(
    PropertyContainerDynamic& container) {
    using PropertyType = Property_<MovingSceneType*>;

    // seek for property
    PropertyBase* p = container.property(this->param_scene_slice_name.value());

    // if not there add it
    if (!p) {
      PropertyType* p_new = new PropertyType(this->param_scene_slice_name.value(), "", &container);
      p_new->setValue(new MovingSceneType());

      // ds check if the merger supports correspondences (defined by its dynamic type)
      MergerCorrespondencePtrType merger_correspondence =
        std::dynamic_pointer_cast<MergerCorrespondenceType>(param_merger.value());

      // ds and whether correspondences are available or not (in the clipped scene)
      if (merger_correspondence && ThisType::_clipped_scene) {
        // ds grab correspondences used for fixed measurement alignment from scene object
        using PropertyCorrespondenceVector = Property_<CorrespondenceVector>;
        PropertyCorrespondenceVector* auxiliary_data =
          ThisType::_clipped_scene->template property<PropertyCorrespondenceVector>(
            ThisType::param_measurement_slice_name.value() +
            CorrespondenceFinderBase::auxiliary_data_suffix_correspondences);
        if (auxiliary_data) {
          // ds get correspondences from previous aligment
          const CorrespondenceVector& local_correspondences = auxiliary_data->value();
          MovingSceneType& moving_cloud                     = *p_new->value();
          moving_cloud.reserve(local_correspondences.size());
          _correspondences.clear();
          _correspondences.reserve(local_correspondences.size());
          // srrg take the correspondence.moving_idx and put it in the current local map
          for (size_t i = 0; i < local_correspondences.size(); ++i) {
            const Correspondence& correspondence = local_correspondences[i];

            // ds populate current scene with corresponding point from previous scene
            // ds note that points from the previous scene without correspondences are not imported!
            assert(static_cast<size_t>(correspondence.moving_idx) < _clipped_scene_slice.size());
            moving_cloud.emplace_back(_clipped_scene_slice[correspondence.moving_idx]);
            _correspondences.emplace_back(
              Correspondence(i, correspondence.fixed_idx, correspondence.response));
          }

          // ds provide merger with correspondences
          // ds we need to do it already because the clipped scene will be swapped out after this
          // operation by the graph slam module - TODO change this confusing mechanic
          merger_correspondence->setCorrespondences(&_correspondences);
        }
      }
      return;
    }

    // otherwise do a type check
    PropertyType* typed_prop = dynamic_cast<PropertyType*>(p);
    if (!typed_prop) {
      throw std::runtime_error("MultiTrackerSlice::populateScene| cast fail");
    }

    // if empty pointer, create one
    if (!typed_prop->value()) {
      typed_prop->setValue(new MovingSceneType);
    }
  }

  template <typename EstimateType_, typename FixedMeasurementType_, typename MovingSceneType_>
  void MultiTrackerSlice_<EstimateType_, FixedMeasurementType_, MovingSceneType_>::sanityCheck(
    const std::string& message) const {
    std::string msg = message;
    if (!this->param_adaptor.value()) {
      msg += "| no adaptor";
      throw std::runtime_error(msg.c_str());
    }
    if (!this->param_clipper.value()) {
      msg += "| no clipper";
      throw std::runtime_error(msg.c_str());
    }
    if (!this->param_merger.value()) {
      msg += "| no merger";
      throw std::runtime_error(msg.c_str());
    }
  }

  template <typename EstimateType_, typename FixedMeasurementType_, typename MovingSceneType_>
  void MultiTrackerSlice_<EstimateType_, FixedMeasurementType_, MovingSceneType_>::
    addMeasurementProperty() {
    sanityCheck("MultiTrackerSlice::addMeasurementProperty|");
    using PropertyType = Property_<FixedType*>;

    PropertyType* prop =
      new PropertyType(this->param_measurement_slice_name.value(), "", this->_adapted_measurements);
    prop->setValue(&_adapted_slice);
  }

  template <typename EstimateType_, typename FixedMeasurementType_, typename MovingSceneType_>
  void MultiTrackerSlice_<EstimateType_, FixedMeasurementType_, MovingSceneType_>::addSceneProperty(
    PropertyContainerBase* scene_) {
    using PropertyType = Property_<MovingSceneType*>;

    // we look for a property
    PropertyBase* p = scene_->property(this->param_scene_slice_name.value());

    // if not there AND the scene is dynamic
    // we add it, and set the pointer to the static buffer (for stand alone operation)
    if (!p) {
      PropertyContainerDynamic* dyn_scene = dynamic_cast<PropertyContainerDynamic*>(scene_);
      if (!dyn_scene) {
        throw std::runtime_error(
          "MultiTrackerSlice::addSceneProperty| scene not dynamic, cannot add properties");
      }
      PropertyType* p_new = new PropertyType(this->param_scene_slice_name.value(), "", dyn_scene);
      //      std::cerr << "added property [" << this->param_scene_slice_name.value() << "] to the
      //      scene"
      //                << std::endl;
      p_new->setValue(&_static_scene_slice);
      p = p_new;
    }

    // try cast
    PropertyType* typed_prop = dynamic_cast<PropertyType*>(p);
    if (!typed_prop) {
      throw std::runtime_error("MultiTrackerSlice::addSceneProperty| cast fail");
    }

    _scene_slice = typed_prop->value();

    // wer add anyway the property to the clipping buffer
    PropertyType* prop_clipped =
      new PropertyType(this->param_scene_slice_name.value(), "", this->_clipped_scene);
    //    std::cerr << "added property [" << this->param_scene_slice_name.value()
    //              << "] to the clipped_scene" << std::endl;
    prop_clipped->setValue(&_clipped_scene_slice);
    sanityCheck("MultiTrackerSlice::addSceneProperty|");
  }

  template <typename EstimateType_, typename FixedMeasurementType_, typename MovingSceneType_>
  void MultiTrackerSlice_<EstimateType_, FixedMeasurementType_, MovingSceneType_>::setClosure(
    const CorrespondenceVector& correspondences_,
    const EstimateType& relative_transform_,
    const EstimateType& tracker_estimate_) {
    _correspondences.clear();
    if (!param_closure_merger.value()) {
      return;
    }

    // ds the correspondences are already global - will be freed after merging TODO smart pointer
    _correspondences.reserve(correspondences_.size());

    // ds we need to carry along the global points with correspondences from the current scene
    // ds the current map points are temporarily preserver in the scene slice
    // ds the scene will be swapped out by the graph slam system once we relocalized
    _clipped_scene_slice.clear();
    _clipped_scene_slice.reserve(correspondences_.size());
    for (size_t i = 0; i < correspondences_.size(); ++i) {
      const Correspondence& correspondence = correspondences_[i];

      // ds moving is the index of the target local map (in which we relocalize -> fixed)
      _correspondences.emplace_back(
        Correspondence(correspondence.moving_idx, i, correspondence.response));

      // ds grab the global point from the scene and it it to the current
      assert(static_cast<size_t>(correspondence.fixed_idx) < _scene_slice->size());
      _clipped_scene_slice.emplace_back((*_scene_slice)[correspondence.fixed_idx]);
    }

    // ds for merging the closure correspondences we need a high level merger (2D-2D or 3D-3D)
    // ds this is because also the local maps store their data in high level format
    // ds the actual merge will be automatically performed in MultiTrackerSlice::merge
    param_closure_merger->setMoving(&_clipped_scene_slice /*instead of adapted*/);
    param_closure_merger->setTransform(relative_transform_ /*closure correction*/);
    param_closure_merger->setCorrespondences(&_correspondences);
    _have_loop_closure_correspondences = true;
  }

  template <typename EstimateType_, typename FixedMeasurementType_, typename MovingSceneType_>
  void MultiTrackerSlice_<EstimateType_, FixedMeasurementType_, MovingSceneType_>::setScene(
    PropertyContainerBase* scene_) {
    addSceneProperty(scene_);
    sanityCheck("MultiTrackerSlice::setScene");
  }

  template <typename EstimateType_, typename FixedMeasurementType_, typename MovingSceneType_>
  void MultiTrackerSlice_<EstimateType_, FixedMeasurementType_, MovingSceneType_>::setMeasurement(
    BaseSensorMessagePtr meas) {
    sanityCheck("MultiTrackerSlice_::setMeasurement()");
    addMeasurementProperty();
    param_adaptor->setDest(&_adapted_slice);
    if (!param_adaptor->setMeasurement(meas)) {
      std::cerr << FG_RED("MultiTrackerSlice::setMeasurement| adaptor->setMeasurement failed")
                << std::endl;
    }
    return;
  }

  template <typename EstimateType_, typename FixedMeasurementType_, typename MovingSceneType_>
  void MultiTrackerSlice_<EstimateType_, FixedMeasurementType_, MovingSceneType_>::adapt() {
    sanityCheck("MultiTrackerSlice_::adapt()");
    param_adaptor->compute();
    if (param_adaptor->status() == MeasurementAdaptorType::Error) {
      std::cerr << FG_RED("TrackerStandard_::adapt()| adaptor status == Error") << std::endl;
    }
  }

  template <typename EstimateType_, typename FixedMeasurementType_, typename MovingSceneType_>
  void MultiTrackerSlice_<EstimateType_, FixedMeasurementType_, MovingSceneType_>::merge() {
    sanityCheck("MultiTrackerSlice_::merge()");
    assert(param_merger.value());

    // ds prepare merger
    param_merger->setScene(_scene_slice);
    param_merger->setMoving(&_adapted_slice);
    param_merger->setTransform(ThisType::_tracker_estimate * ThisType::_sensor_in_robot);

    // ds check if the merger supports correspondences (defined by its dynamic type)
    MergerCorrespondencePtrType merger_correspondence =
      std::dynamic_pointer_cast<MergerCorrespondenceType>(param_merger.value());
    if (!merger_correspondence) {
      // ds perform regular merge if no correspondences are required
      param_merger->compute();
      return;
    }

    // ds set global tracker pose estimate (global landmark refinement)
    merger_correspondence->setTrackerInWorld(ThisType::_tracker_in_world *
                                             ThisType::_sensor_in_robot);

    // ds if loop closure data is available for correspondence based merging
    if (_have_loop_closure_correspondences) {
      // ds correspondence-based closure merger must be available
      if (!param_closure_merger.value()) {
        throw std::runtime_error("MultiTrackerSlice::merge|ERROR: correspondence-based merger for "
                                 "closure aligment not available");
      }

      // ds prepare merger and compute - moving, correspondences and transform have been set
      param_closure_merger->setScene(_scene_slice);
      param_closure_merger->compute();

      // ds free correspondences set in closure update
      _correspondences.clear();
      _have_loop_closure_correspondences = false;
      return;
    }

    // ds clipped scene (which carries correspondences) must be available for correspondence merger!
    if (!ThisType::_clipped_scene) {
      throw std::runtime_error(
        "MultiTrackerSlice::merge|ERROR: auxiliary data not available for correspondence import");
    }

    // ds grab correspondences used for fixed measurement alignment from scene object
    CorrespondenceVector global_correspondences;
    using PropertyCorrespondenceVector = Property_<CorrespondenceVector>;
    PropertyCorrespondenceVector* correspondence_property =
      ThisType::_clipped_scene->template property<PropertyCorrespondenceVector>(
        ThisType::param_measurement_slice_name.value() +
        CorrespondenceFinderBase::auxiliary_data_suffix_correspondences);
    if (correspondence_property) {
      const CorrespondenceVector& local_correspondences = correspondence_property->value();

      // ds set correspondences between fixed and scene as determined by aligner
      // ds NOTE that the correspondences have been computed between fixed and local scene
      // ds for the merger we have to flip the correspondences and map them to the global scene
      // ds TODO figure out a workaround for this overhead here
      global_correspondences.reserve(local_correspondences.size());
      const std::vector<int>& local_to_global(param_clipper->globalIndices());
      assert(local_to_global.size() <= _scene_slice->size());
      assert(local_to_global.size() >= local_correspondences.size());
      for (const Correspondence& correspondence : local_correspondences) {
        assert(static_cast<size_t>(correspondence.moving_idx) < local_to_global.size());
        global_correspondences.emplace_back(
          Correspondence(local_to_global[correspondence.moving_idx] /* map to global scene */,
                         correspondence.fixed_idx,
                         correspondence.response));
      }

      // ds set correspondences and merge
      merger_correspondence->setCorrespondences(&global_correspondences);
      merger_correspondence->compute();
    } else {
      // ds perform regular merge (we are in a new local map)
      merger_correspondence->compute();
    }
  }

  template <typename EstimateType_, typename FixedMeasurementType_, typename MovingSceneType_>
  void MultiTrackerSlice_<EstimateType_, FixedMeasurementType_, MovingSceneType_>::clip() {
    sanityCheck("MultiTrackerSlice_::clip()");
    param_clipper->setGlobalScene(_scene_slice);
    param_clipper->setLocalScene(&_clipped_scene_slice);
    param_clipper->setAuxiliaryData(ThisType::_clipped_scene,
                                    ThisType::param_scene_slice_name.value());
    param_clipper->setTransform(ThisType::_tracker_estimate);
    param_clipper->setSensorInRobot(ThisType::_sensor_in_robot);

    param_clipper->compute();
  }

  template <typename EstimateType_, typename FixedMeasurementType_, typename MovingSceneType_>
  bool
  MultiTrackerSlice_<EstimateType_, FixedMeasurementType_, MovingSceneType_>::isSceneSliceEmpty()
    const {
    sanityCheck("MultiTrackerSlice_::isSceneEmpty()");
    return _scene_slice->empty() || (param_adaptor->status() != MeasurementAdaptorType::Ready);
  }

} // namespace srrg2_slam_interfaces
