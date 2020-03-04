#include "multi_aligner_slice.h"
#include "srrg_config/property_configurable.h"
#include "srrg_property/property_identifiable.h"

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;
  using namespace srrg2_solver;

  template <typename FactorType_, typename FixedType_, typename MovingType_>
  void MultiAlignerSlice_<FactorType_, FixedType_, MovingType_>::sanityCheck(
    const std::string& message) {
    std::string msg = message;
    if (!_fixed_slice) {
      msg += "| no fixed";
      throw std::runtime_error(msg.c_str());
    }
    if (!_moving_slice) {
      msg += "| no moving";
      throw std::runtime_error(msg.c_str());
    }
    if (!param_finder.value()) {
      msg += "| no finder";
      throw std::runtime_error(msg.c_str());
    }
  }

  template <typename FactorType_, typename FixedType_, typename MovingType_>
  void
  MultiAlignerSlice_<FactorType_, FixedType_, MovingType_>::setEstimate(const EstimateType& est_) {
    sanityCheck("MultiAlignerSlice_::setEstimate");

    if (this->param_frame_id.value().length() && this->param_base_frame_id.value().length() &&
        this->_platform) {
      EstimateType sensor_in_robot = EstimateType::Identity();
      if (!this->_platform->getTransform(
            sensor_in_robot, this->param_frame_id.value(), this->param_base_frame_id.value())) {
        throw std::runtime_error("unable to get the transformation");
      }
      setSensorInRobot(sensor_in_robot);
    }

    param_finder->setEstimate(_robot_in_sensor * est_);
  }

  template <typename FactorType_, typename FixedType_, typename MovingType_>
  FactorBasePtr MultiAlignerSlice_<FactorType_, FixedType_, MovingType_>::factor() {
    sanityCheck("MultiAlignerSlice_::factor");
    _factor->setVariableId(0, 0);
    return _factor;
  }

  template <typename FactorType_, typename FixedType_, typename MovingType_>
  CorrespondenceVector*
  MultiAlignerSlice_<FactorType_, FixedType_, MovingType_>::computeAssociation() {
    sanityCheck("MultiAlignerSlice_::computeCorrespondences()|");
    param_finder->setCorrespondences(&_correspondences);
    param_finder->compute();
    _factor->setCorrespondences(_correspondences);
    // srrg I think this should be moved somewhere else (right before the compute)
    setupFactor();
    return &_correspondences;
  }

  template <typename FactorType_, typename FixedType_, typename MovingType_>
  void MultiAlignerSlice_<FactorType_, FixedType_, MovingType_>::storeCorrespondences() {
    // ds check for the possibility of adding auxiliary data to the scene
    PropertyContainerDynamic* auxiliary_data =
      dynamic_cast<PropertyContainerDynamic*>(ThisType::_moving_scene);
    if (auxiliary_data) {
      using PropertyCorrespondenceVector = Property_<CorrespondenceVector>;
      const std::string property_name =
        ThisType::param_fixed_slice_name.value() +
        CorrespondenceFinderBase::auxiliary_data_suffix_correspondences;
      PropertyCorrespondenceVector* property =
        auxiliary_data->property<PropertyCorrespondenceVector>(property_name);

      // ds if the property is already set
      if (property) {
        // ds update the entry - copying the correspondences (expensive but manipulation safe)
        property->value() = _correspondences;
      } else {
        // ds store the correspondences in a container for this slice
        // ds the correspondences can be later picked up by the tracker for use in a tracker slice
        auxiliary_data->addProperty(new PropertyCorrespondenceVector(
          property_name, "aligner correspondences", this, _correspondences));
      }
    }
  }

  template <typename FactorType_, typename FixedType_, typename MovingType_>
  bool MultiAlignerSlice_<FactorType_, FixedType_, MovingType_>::associationGood() const {
    return (int) _correspondences.size() > param_min_num_correspondences.value();
  }

  template <typename FactorType_, typename FixedType_, typename MovingType_>
  void MultiAlignerSlice_<FactorType_, FixedType_, MovingType_>::bindFixed() {
    _fixed_slice = nullptr;

    // sanityCheck("MultiAlignerSlice_::updateFixed()|");
    if (!this->_fixed_scene) {
      throw std::runtime_error("MultiAlignerSlice_::bindFixed()| no fixed");
    }
    PropertyBase* p_base = this->_fixed_scene->property(this->param_fixed_slice_name.value());
    if (!p_base) {
      std::cerr << "property name: " << this->param_fixed_slice_name.value() << std::endl;
      std::cerr << "num properties in fixed: " << this->_fixed_scene->properties().size()
                << std::endl;
      std::cerr << "properties currently in fixed: " << std::endl;
      for (auto it : this->_fixed_scene->properties()) {
        std::cerr << it.first << std::endl;
      }
      throw std::runtime_error("MultiAlignerSlice_::bindFixed()| no property in fixed");
    }
    // 1 the property might be a plain object, so we try to fetch it as such
    {
      Property_<FixedType*>* p_fixed = dynamic_cast<Property_<FixedType*>*>(p_base);
      if (p_fixed) {
        _fixed_slice = p_fixed->value();
      }
    }

    // 2 the property might be stored brutally, we get the pointer
    if (!_fixed_slice) {
      Property_<FixedType>* p_fixed = dynamic_cast<Property_<FixedType>*>(p_base);
      if (p_fixed) {
        _fixed_slice = &p_fixed->value();
      }
    }

    // 3 the property might be a shared pointer, we get the raw pointer
    if (!_fixed_slice) {
      Property_<std::shared_ptr<FixedType>>* p_fixed =
        dynamic_cast<Property_<std::shared_ptr<FixedType>>*>(p_base);
      if (p_fixed) {
        _fixed_slice = p_fixed->value().get();
      }
    }
    if (!_fixed_slice) {
      throw std::runtime_error("MultiAlignerSlice_::bindFixed()| cast fail");
    }

    // std::cerr << "fixed size: " << _fixed_slice->size() << std::endl;
    param_finder->setFixed(_fixed_slice);
    _factor->setFixed(*_fixed_slice);
  }

  template <typename FactorType_, typename FixedType_, typename MovingType_>
  void MultiAlignerSlice_<FactorType_, FixedType_, MovingType_>::bindMoving() {
    _moving_slice = nullptr;

    // sanityCheck("MultiAlignerSlice_::updateMoving()|");
    if (!ThisType::_moving_scene)
      throw std::runtime_error("MultiAlignerSlice_::bindMoving()| no moving");
    PropertyBase* p_base =
      ThisType::_moving_scene->property(ThisType::param_moving_slice_name.value());
    if (!p_base) {
      throw std::runtime_error("MultiAlignerSlice_::bindMoving()| no property in moving");
    }

    // 1 the property might be a plain object, so we try to fetch it as such
    {
      Property_<MovingType*>* p_moving = dynamic_cast<Property_<MovingType*>*>(p_base);
      if (p_moving) {
        _moving_slice = p_moving->value();
      }
    }

    // 2 the property might be stored brutally, we get the pointer
    if (!_moving_slice) {
      Property_<MovingType>* p_moving = dynamic_cast<Property_<MovingType>*>(p_base);
      if (p_moving) {
        _moving_slice = &p_moving->value();
      }
    }

    // 3 the property might be a shared pointer, we get the raw pointer
    if (!_moving_slice) {
      Property_<std::shared_ptr<MovingType>>* p_moving =
        dynamic_cast<Property_<std::shared_ptr<MovingType>>*>(p_base);
      if (p_moving) {
        _moving_slice = p_moving->value().get();
      }
    }
    if (!_moving_slice)
      throw std::runtime_error("MultiAlignerSlice_::bindMoving()| cast fail");

    // std::cerr << "moving size: " << _moving_slice->size() << std::endl;
    param_finder->setMoving(_moving_slice);
    _factor->setMoving(*_moving_slice);
  }

} // namespace srrg2_slam_interfaces
