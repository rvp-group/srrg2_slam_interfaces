#pragma once
#include "srrg_config/configurable.h"
#include "srrg_data_structures/platform.h"
#include "srrg_messages/message_handlers/message_pack.h"
#include "srrg_messages/messages/base_sensor_message.h"
#include "srrg_system_utils/profiler.h"

namespace srrg2_slam_interfaces {

  //! this class converts measurements, if possible from a PropertyContainer
  template <typename DestType_>
  class MeasurementAdaptor_ : public srrg2_core::Configurable,
                              public srrg2_core::PlatformUser,
                              public srrg2_core::Profiler {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    enum Status { Ready, Initializing, Error };
    using DestType = DestType_;
    using ThisType = MeasurementAdaptor_<DestType>;
    using BaseType = Configurable;

    virtual void compute() = 0;
    virtual void reset() {
      _dest        = nullptr;
      _measurement = nullptr;
      _status      = Error;
    }

    inline void setDest(DestType* dest_) {
      _dest              = dest_;
      _dest_changed_flag = true;
    }

    virtual bool setMeasurement(srrg2_core::BaseSensorMessagePtr measurement_) {
      _measurement              = measurement_;
      _measurement_changed_flag = true;
      return true;
    }

    inline Status status() const {
      return _status;
    }

  protected:
    Status _status                                = Error;
    bool _dest_changed_flag                       = true;
    bool _measurement_changed_flag                = true;
    srrg2_core::BaseSensorMessagePtr _measurement = 0;
    DestType* _dest                               = 0;
  };

  // strips a message of a type and a topic from either a container or a raw measurement
  template <typename DestMessageType_>
  std::shared_ptr<DestMessageType_> extractMessage(srrg2_core::BaseSensorMessagePtr measurement,
                                                   const std::string& topic_) {
    srrg2_core::MessagePackPtr message_pack =
      std::dynamic_pointer_cast<srrg2_core::MessagePack>(measurement);

    // ia if it's not a pack, try to process a single laser message
    if (!message_pack) {
      if (measurement->topic.value() == topic_) {
        return std::dynamic_pointer_cast<DestMessageType_>(measurement);
      }
      return std::shared_ptr<DestMessageType_>();
    }
    // is a pack
    for (srrg2_core::BaseSensorMessagePtr& message : message_pack->messages) {
      if (message->topic.value() != topic_) {
        continue;
      }
      std::shared_ptr<DestMessageType_> result =
        std::dynamic_pointer_cast<DestMessageType_>(message);
      if (result) {
        return result;
      }
    }
    return nullptr;
  }

} // namespace srrg2_slam_interfaces
