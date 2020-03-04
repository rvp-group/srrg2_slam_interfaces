#include "multi_aligner_slice_prior.h"
#include "srrg_config/property_configurable.h"
#include "srrg_property/property_identifiable.h"
#include <memory>

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;
  using namespace srrg2_solver;

  template <typename FactorType_, typename FixedType_, typename MovingType_>
  void MultiAlignerSlicePrior_<FactorType_, FixedType_, MovingType_>::sanityCheck(
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
  }

  template <typename FactorType_, typename FixedType_, typename MovingType_>
  void MultiAlignerSlicePrior_<FactorType_, FixedType_, MovingType_>::bindFixed() {
    _fixed_slice = 0;

    // sanityCheck("MultiAlignerSlicePrior_::updateFixed()|");
    if (!this->_fixed_scene) {
      throw std::runtime_error("MultiAlignerSlicePrior_::bindFixed()| no fixed");
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
      throw std::runtime_error("MultiAlignerSlicePrior_::bindFixed()| no property in fixed");
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
      throw std::runtime_error("MultiAlignerSlicePrior_::bindFixed()| cast fail");
    }
  }

  template <typename FactorType_, typename FixedType_, typename MovingType_>
  void MultiAlignerSlicePrior_<FactorType_, FixedType_, MovingType_>::bindMoving() {
    _moving_slice = 0;

    // sanityCheck("MultiAlignerSlicePrior_::updateMoving()|");
    if (!this->_moving_scene) {
      throw std::runtime_error("MultiAlignerSlicePrior_::bindMoving()| no moving");
    }
    PropertyBase* p_base = this->_moving_scene->property(this->param_moving_slice_name.value());
    if (!p_base) {
      throw std::runtime_error("MultiAlignerSlicePrior_::bindMoving()| no property in moving");
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
    if (!_moving_slice) {
      throw std::runtime_error("MultiAlignerSlicePrior_::bindMoving()| cast fail");
    }
  }

} // namespace srrg2_slam_interfaces
