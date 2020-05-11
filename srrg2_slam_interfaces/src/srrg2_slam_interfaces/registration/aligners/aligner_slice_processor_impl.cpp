#include "aligner_slice_processor.h"
#include "aligner_slice_processor_prior_impl.cpp"

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;
  using namespace srrg2_solver;

  template <typename FactorType_, typename FixedType_, typename MovingType_>
  void AlignerSliceProcessor_<FactorType_, FixedType_, MovingType_>::sanityCheck(
    const std::string& message) {
    BaseType::sanityCheck(message);
    std::string msg = message;
    if (!param_finder.value()) {
      msg += "| no finder";
      throw std::runtime_error(msg.c_str());
    }
  }

  template <typename FactorType_, typename FixedType_, typename MovingType_>
  void AlignerSliceProcessor_<FactorType_, FixedType_, MovingType_>::setMovingInFixed(
    const EstimateType& moving_in_fixed_) {
    sanityCheck("AlignerSliceProcessor_::setEstimate");

    if (BaseType::param_frame_id.value().length() &&
        BaseType::param_base_frame_id.value().length() && BaseType::_platform) {
      EstimateType sensor_in_robot = EstimateType::Identity();
      if (!BaseType::_platform->getTransform(sensor_in_robot,
                                             BaseType::param_frame_id.value(),
                                             BaseType::param_base_frame_id.value())) {
        throw std::runtime_error("unable to get the transformation");
      }
      setSensorInRobot(sensor_in_robot);
    }

    param_finder->setLocalMapInSensor(_robot_in_sensor * moving_in_fixed_);
  }

  template <typename FactorType_, typename FixedType_, typename MovingType_>
  CorrespondenceVector*
  AlignerSliceProcessor_<FactorType_, FixedType_, MovingType_>::computeCorrespondences() {
    sanityCheck("AlignerSliceProcessor_::computeCorrespondences()|");
    param_finder->setCorrespondences(&_correspondences);
    param_finder->compute();
    BaseType::_factor->setCorrespondences(_correspondences);
    // TODO srrg I think this should be moved somewhere else (right before the compute)
    setupFactor();
    return &_correspondences;
  }

  template <typename FactorType_, typename FixedType_, typename MovingType_>
  void AlignerSliceProcessor_<FactorType_, FixedType_, MovingType_>::storeCorrespondences() {
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
  bool AlignerSliceProcessor_<FactorType_, FixedType_, MovingType_>::correspondencesGood() const {
    return (int) _correspondences.size() > param_min_num_correspondences.value();
  }

  template <typename FactorType_, typename FixedType_, typename MovingType_>
  inline void AlignerSliceProcessor_<FactorType_, FixedType_, MovingType_>::bindFixed() {
    BaseType::bindFixed();
    param_finder->setFixed(BaseType::_fixed_slice);
    BaseType::_factor->setFixed(*BaseType::_fixed_slice);
  }

  template <typename FactorType_, typename FixedType_, typename MovingType_>
  inline void AlignerSliceProcessor_<FactorType_, FixedType_, MovingType_>::bindMoving() {
    BaseType::bindMoving();
    param_finder->setMoving(BaseType::_moving_slice);
    BaseType::_factor->setMoving(*BaseType::_moving_slice);
  }

} // namespace srrg2_slam_interfaces
