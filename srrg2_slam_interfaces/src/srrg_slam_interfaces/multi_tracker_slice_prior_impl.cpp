#pragma once
#include "measurement_adaptor_odom.h"
#include "measurement_adaptor_tracker_estimate.hpp"
#include "multi_tracker_slice_prior.h"

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;
  using namespace srrg2_solver;

  template <typename EstimateType_, typename FixedMeasurementType_, typename MovingSceneType_>
  void
  MultiTrackerSlicePrior_<EstimateType_, FixedMeasurementType_, MovingSceneType_>::populateScene(
    PropertyContainerDynamic& container) {
    using PropertyType = Property_<MovingSceneType*>;

    // seek for property
    PropertyBase* p = container.property(this->param_scene_slice_name.value());

    // if not there add it
    if (!p) {
      PropertyType* p_new = new PropertyType(this->param_scene_slice_name.value(), "", &container);
      //      std::cerr << "MultiTrackerSlice::populateScene| added property ["
      //                << this->param_scene_slice_name.value() << "] to the scene" << std::endl;
      p_new->setValue(new MovingSceneType);
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
  void MultiTrackerSlicePrior_<EstimateType_, FixedMeasurementType_, MovingSceneType_>::sanityCheck(
    const std::string& message) const {
    std::string msg = message;
    if (!this->param_adaptor.value()) {
      msg += "| no adaptor";
      throw std::runtime_error(msg.c_str());
    }
  }

  template <typename EstimateType_, typename FixedMeasurementType_, typename MovingSceneType_>
  void MultiTrackerSlicePrior_<EstimateType_, FixedMeasurementType_, MovingSceneType_>::
    addMeasurementProperty() {
    sanityCheck("MultiTrackerSlice::addMeasurementProperty|");
    using PropertyType = Property_<FixedType*>;

    PropertyType* prop =
      new PropertyType(this->param_measurement_slice_name.value(), "", this->_adapted_measurements);
    prop->setValue(&_adapted_slice);
  }

  template <typename EstimateType_, typename FixedMeasurementType_, typename MovingSceneType_>
  void
  MultiTrackerSlicePrior_<EstimateType_, FixedMeasurementType_, MovingSceneType_>::addSceneProperty(
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
  void MultiTrackerSlicePrior_<EstimateType_, FixedMeasurementType_, MovingSceneType_>::setScene(
    PropertyContainerBase* scene_) {
    addSceneProperty(scene_);
    sanityCheck("MultiTrackerSlice::setScene");
  }

  template <typename EstimateType_, typename FixedMeasurementType_, typename MovingSceneType_>
  void
  MultiTrackerSlicePrior_<EstimateType_, FixedMeasurementType_, MovingSceneType_>::setMeasurement(
    BaseSensorMessagePtr meas) {
    sanityCheck("MultiTrackerSlicePrior_::setMeasurement()");
    addMeasurementProperty();
    param_adaptor->setDest(&_adapted_slice);
    if (!param_adaptor->setMeasurement(meas)) {
      std::cerr << FG_RED("MultiTrackerSlice::setMeasurement| adaptor->setMeasurement failed")
                << std::endl;
    }
  }

  template <typename EstimateType_, typename FixedMeasurementType_, typename MovingSceneType_>
  void MultiTrackerSlicePrior_<EstimateType_, FixedMeasurementType_, MovingSceneType_>::adapt() {
    sanityCheck("MultiTrackerSlicePrior_::adapt()");
    param_adaptor->compute();
    if (param_adaptor->status() == MeasurementAdaptorType::Error) {
      std::cerr << FG_RED("TrackerStandard_::adapt()| adaptor status == Error") << std::endl;
    }
  }

  template <typename EstimateType_, typename FixedMeasurementType_, typename MovingSceneType_>
  void MultiTrackerSlicePrior_<EstimateType_, FixedMeasurementType_, MovingSceneType_>::merge() {
    *_scene_slice = _adapted_slice;
  }

} // namespace srrg2_slam_interfaces
