#include "tracker_slice_processor_base.h"

namespace srrg2_slam_interfaces {

  template <typename EstimateType_>
  void TrackerSliceProcessorBase_<EstimateType_>::setRobotInLocalMap(
    const EstimateType& robot_in_local_map_) {
    if (param_frame_id.value().length() && param_base_frame_id.value().length() &&
        srrg2_core::PlatformUser::_platform) {
      EstimateType sensor_in_robot = EstimateType::Identity();
      if (!srrg2_core::PlatformUser::_platform->getTransform(
            sensor_in_robot, param_frame_id.value(), param_base_frame_id.value())) {
        throw std::runtime_error("unable to get the transformation");
      }
      setSensorInRobot(sensor_in_robot);
    }
    _robot_in_local_map = robot_in_local_map_;
  }

  template <typename EstimateType_, typename FixedMeasurementType_, typename MovingSceneType_>
  void TrackerSliceProcessorStandard_<EstimateType_, FixedMeasurementType_, MovingSceneType_>::
    sanityCheck(const std::string& message) const {
    std::string msg = message;
    if (!param_adaptor.value()) {
      msg += "| no adaptor";
      throw std::runtime_error(msg.c_str());
    }
  }

  template <typename EstimateType_, typename FixedMeasurementType_, typename MovingSceneType_>
  void
  TrackerSliceProcessorStandard_<EstimateType_, FixedMeasurementType_, MovingSceneType_>::adapt() {
    sanityCheck("TrackerSliceProcessorPrior_::adapt()");
    param_adaptor->compute();
    if (param_adaptor->status() == RawDataPreprocessorType::Error) {
      std::cerr << FG_RED("TrackerStandard_::adapt()| adaptor status == Error") << std::endl;
    }
  }

  template <typename EstimateType_, typename FixedMeasurementType_, typename MovingSceneType_>
  void TrackerSliceProcessorStandard_<EstimateType_, FixedMeasurementType_, MovingSceneType_>::
    setRawData(BaseSensorMessagePtr msg_) {
    sanityCheck("TrackerSliceProcessorPrior_::setRawData()");
    addMeasurementProperty();
    param_adaptor->setMeas(&_measurement_slice);
    if (!param_adaptor->setRawData(msg_)) {
      std::cerr << FG_RED("TrackerSliceProcessorPrior_::setRawData| adaptor->setMeasurement failed")
                << std::endl;
    }
  }

  template <typename EstimateType_, typename FixedMeasurementType_, typename MovingSceneType_>
  void TrackerSliceProcessorStandard_<EstimateType_, FixedMeasurementType_, MovingSceneType_>::
    populateScene(PropertyContainerDynamic& container) {
    using PropertyType = Property_<MovingSceneType*>;

    // seek for property
    PropertyBase* p = container.property(ThisType::param_scene_slice_name.value());

    // if not there add it
    if (!p) {
      PropertyType* p_new =
        new PropertyType(ThisType::param_scene_slice_name.value(), "", &container);
      //      std::cerr << "TrackerSliceProcessorPrior_::populateScene| added property ["
      //                << ThisType::param_scene_slice_name.value() << "] to the scene" <<
      //                std::endl;
      p_new->setValue(new MovingSceneType);

      enhanceSceneProperty(p_new);
      return;
    }

    // otherwise do a type check
    PropertyType* typed_prop = dynamic_cast<PropertyType*>(p);
    if (!typed_prop) {
      throw std::runtime_error("TrackerSliceProcessorPrior_::populateScene| cast fail");
    }

    // if empty pointer, create one
    if (!typed_prop->value()) {
      typed_prop->setValue(new MovingSceneType);
    }
  }

  template <typename EstimateType_, typename FixedMeasurementType_, typename MovingSceneType_>
  void TrackerSliceProcessorStandard_<EstimateType_, FixedMeasurementType_, MovingSceneType_>::
    addMeasurementProperty() {
    sanityCheck("TrackerSliceProcessorPrior_::addMeasurementProperty|");
    using PropertyType = Property_<FixedMeasurementType*>;

    PropertyType* prop = new PropertyType(
      ThisType::param_measurement_slice_name.value(), "", ThisType::_measurements_container);
    prop->setValue(&_measurement_slice);
  }

  template <typename EstimateType_, typename FixedMeasurementType_, typename MovingSceneType_>
  void TrackerSliceProcessorStandard_<EstimateType_, FixedMeasurementType_, MovingSceneType_>::
    addSceneProperty(PropertyContainerBase* scene_) {
    using PropertyType = Property_<MovingSceneType*>;

    // we look for a property
    PropertyBase* p = scene_->property(ThisType::param_scene_slice_name.value());

    // if not there AND the scene is dynamic
    // we add it, and set the pointer to the static buffer (for stand alone operation)
    if (!p) {
      PropertyContainerDynamic* dyn_scene = dynamic_cast<PropertyContainerDynamic*>(scene_);
      if (!dyn_scene) {
        throw std::runtime_error("TrackerSliceProcessorPrior_::addSceneProperty| scene not "
                                 "dynamic, cannot add properties");
      }
      PropertyType* p_new =
        new PropertyType(ThisType::param_scene_slice_name.value(), "", dyn_scene);
      //      std::cerr << "added property [" << ThisType::param_scene_slice_name.value() << "] to
      //      the scene" << std::endl;
      p_new->setValue(&_static_scene_slice);
      p = p_new;
    }

    // try cast
    PropertyType* typed_prop = dynamic_cast<PropertyType*>(p);
    if (!typed_prop) {
      throw std::runtime_error("TrackerSliceProcessorPrior_::addSceneProperty| cast fail");
    }

    _scene_slice = typed_prop->value();

    // wer add anyway the property to the clipping buffer
    PropertyType* prop_clipped = new PropertyType(
      ThisType::param_scene_slice_name.value(), "", ThisType::_clipped_scene_container);
    //    std::cerr << "added property [" << ThisType::param_scene_slice_name.value()
    //              << "] to the clipped_scene" << std::endl;
    prop_clipped->setValue(&_clipped_scene_slice);
    sanityCheck("TrackerSliceProcessorPrior_::addSceneProperty|");
  }

  template <typename EstimateType_, typename FixedMeasurementType_, typename MovingSceneType_>
  void
  TrackerSliceProcessorStandard_<EstimateType_, FixedMeasurementType_, MovingSceneType_>::setScene(
    PropertyContainerBase* scene_) {
    addSceneProperty(scene_);
    sanityCheck("TrackerSliceProcessorPrior_::setScene");
  }

} // namespace srrg2_slam_interfaces
