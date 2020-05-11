#pragma once
#include <srrg_config/configurable.h>
#include <srrg_data_structures/platform.h>
#include <srrg_messages/message_handlers/message_pack.h>
#include <srrg_messages/messages/base_sensor_message.h>
#include <srrg_system_utils/profiler.h>

namespace srrg2_slam_interfaces {

  /**
   * @brief this class converts measurements, if possible from a PropertyContainer
   */
  template <typename MeasurementType_>
  class RawDataPreprocessor_ : public srrg2_core::Configurable,
                               public srrg2_core::PlatformUser,
                               public srrg2_core::Profiler {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**
     * @brief status of the module
     */
    enum Status { Ready = 0x0, Initializing = 0x1, Error = 0x2 };
    using MeasurementType    = MeasurementType_;
    using MeasurementTypePtr = std::shared_ptr<MeasurementType>;
    using ThisType           = RawDataPreprocessor_<MeasurementType>;
    using BaseType           = Configurable;

    RawDataPreprocessor_() {
      reset();
    }

    virtual ~RawDataPreprocessor_() = default;

    /**
     * @brief processes the raw data and produces measurements with an adequate level of abstraction
     * to be presented to the other modules
     */
    virtual void compute() = 0;

    /**
     * @brief resets the state of the module
     */
    virtual void reset() {
      _raw_data = nullptr;
      _meas     = nullptr;
      _status   = Error;
    }

    /**
     * @brief the destination location for the computed measurement
     * @param[in/out] measurement
     */
    inline void setMeas(MeasurementType* dest_) {
      _meas              = dest_;
      _meas_changed_flag = true;
    }

    // srrg enable this when raw data preprocessor holds the meas and comment the function above
    //    inline MeasurementTypePtr measurement() {
    //      return _meas;
    //    }
    //
    /**
     * @brief provide the raw message data to the module
     * @param[in] raw_data: raw data message ptr
     */
    virtual bool setRawData(srrg2_core::BaseSensorMessagePtr raw_data_) {
      _raw_data_changed_flag = true;
      _status                = Status::Error;
      _raw_data              = raw_data_;
      return true;
    }

    /**
     * @brief module status getter
     * @return one of the enum Status
     */
    inline Status status() const {
      return _status;
    }

  protected:
    Status _status                             = Error;
    bool _meas_changed_flag                    = true;
    bool _raw_data_changed_flag                = true;
    srrg2_core::BaseSensorMessagePtr _raw_data = nullptr;
    MeasurementType* _meas                     = nullptr;
  };

  /**
   * @brief strips a message of a type and a topic from either a container or a raw measurement
   */
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
