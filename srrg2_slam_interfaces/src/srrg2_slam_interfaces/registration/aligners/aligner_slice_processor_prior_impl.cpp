#include "aligner_slice_processor_base_impl.cpp"
#include "aligner_slice_processor_prior.h"
#include "srrg_config/property_configurable.h"
#include "srrg_property/property_identifiable.h"

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;
  using namespace srrg2_solver;

  template <typename FactorType_, typename FixedType_, typename MovingType_>
  void AlignerSliceProcessorPrior_<FactorType_, FixedType_, MovingType_>::sanityCheck(
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
  inline srrg2_solver::FactorBasePtr
  AlignerSliceProcessorPrior_<FactorType_, FixedType_, MovingType_>::factor() {
    sanityCheck("AlignerSliceProcessor_::factor");
    _factor->setVariableId(0, 0);
    return _factor;
  }

  template <typename FactorType_, typename FixedType_, typename MovingType_>
  void AlignerSliceProcessorPrior_<FactorType_, FixedType_, MovingType_>::bindFixed() {
    _fixed_slice = nullptr;
    BaseType::bindSlice(
      _fixed_slice, BaseType::_fixed_scene, BaseType::param_fixed_slice_name.value());
    if (!_fixed_slice) {
      throw std::runtime_error("AlignerSliceProcessor_::bindFixed()| fixed slice cast fail");
    }
  }

  template <typename FactorType_, typename FixedType_, typename MovingType_>
  void AlignerSliceProcessorPrior_<FactorType_, FixedType_, MovingType_>::bindMoving() {
    _moving_slice = nullptr;
    BaseType::bindSlice(
      _moving_slice, BaseType::_moving_scene, BaseType::param_moving_slice_name.value());
    if (!_moving_slice) {
      throw std::runtime_error("AlignerSliceProcessor_::bindMoving()| moving slice cast fail");
    }
  }

} // namespace srrg2_slam_interfaces
