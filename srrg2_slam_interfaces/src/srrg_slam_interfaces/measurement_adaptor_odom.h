#pragma once
#include "measurement_adaptor.h"
#include <srrg_messages/messages/odometry_message.h>

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;

  template <typename DestType_>
  class MeasurementAdaptorOdom_ : public srrg2_slam_interfaces::MeasurementAdaptor_<DestType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using DestType = DestType_;
    using ThisType = MeasurementAdaptorOdom_<DestType>;
    using BaseType = srrg2_slam_interfaces::MeasurementAdaptor_<DestType>;

    PARAM(PropertyString, topic, "topic of odom, to be blasted", "/odom", nullptr);

    // void compute() override;
    bool setMeasurement(srrg2_core::BaseSensorMessagePtr measurement_) override {
      ThisType::_status = ThisType::Status::Error;
      OdometryMessagePtr odom_message =
        extractMessage<OdometryMessage>(measurement_, param_topic.value());
      if (!odom_message) {
        return false;
      }

      BaseType::setMeasurement(measurement_);
      _saved_odom       = odom_message;
      ThisType::_status = ThisType::Status::Initializing;
      return true;
    }

  protected:
    OdometryMessagePtr _saved_odom;
  };

  class MeasurementAdaptorOdom2D : public MeasurementAdaptorOdom_<Isometry2f> {
  public:
    void compute() override;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  class MeasurementAdaptorOdom3D : public MeasurementAdaptorOdom_<Isometry3f> {
  public:
    void compute() override;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace srrg2_slam_interfaces
