#pragma once
#include "raw_data_preprocessor.h"
#include <srrg_messages/messages/odometry_message.h>

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;

  template <typename MeasurementType_>
  class RawDataPreprocessorOdom_ : public RawDataPreprocessor_<MeasurementType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using MeasurementType = MeasurementType_;
    using ThisType        = RawDataPreprocessorOdom_<MeasurementType>;
    using BaseType        = RawDataPreprocessor_<MeasurementType>;

    PARAM(PropertyString, topic, "topic of odom, to be blasted", "/odom", nullptr);

    bool setRawData(srrg2_core::BaseSensorMessagePtr message_) override {
      OdometryMessagePtr odom_message =
        extractMessage<OdometryMessage>(message_, param_topic.value());
      if (!odom_message) {
        // why should I invalidate the whole message?
        BaseType::reset();
        return false;
      }

      BaseType::setRawData(message_);
      _saved_odom       = odom_message;
      ThisType::_status = ThisType::Status::Initializing;
      return true;
    }

  protected:
    OdometryMessagePtr _saved_odom;
  };

  class RawDataPreprocessorOdom2D : public RawDataPreprocessorOdom_<Isometry2f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void compute() override;
  };

  class RawDataPreprocessorOdom3D : public RawDataPreprocessorOdom_<Isometry3f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void compute() override;
  };

  using RawDataPreprocessorOdom2DPtr = std::shared_ptr<RawDataPreprocessorOdom2D>;
  using RawDataPreprocessorOdom3DPtr = std::shared_ptr<RawDataPreprocessorOdom3D>;

} // namespace srrg2_slam_interfaces
