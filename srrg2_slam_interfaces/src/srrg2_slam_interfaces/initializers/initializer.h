#pragma once
#include <srrg_config/configurable.h>
#include <srrg_config/property_configurable_vector.h>
#include <srrg_data_structures/platform.h>
#include <srrg_messages/messages/base_sensor_message.h>

namespace srrg2_slam_interfaces {

  /**
   * @brief base class for initializer
   * the system will not start while it is not completely initialized
   */
  class Initializer : public srrg2_core::Configurable, public srrg2_core::PlatformUser {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual ~Initializer() = default;

    void setRawData(srrg2_core::BaseSensorMessagePtr message_) {
      _message = message_;
    }

    /**
     * @brief initialize the system
     */
    virtual void initialize() = 0;
    /**
     * @brief check if the system is ready
     */
    const bool isInitialized() const {
      return _initialized;
    }

  protected:
    bool _initialized = false; /**< initialized control flag*/

    srrg2_core::BaseSensorMessagePtr _message = nullptr; /**< current message */
  };

  using InitializerPtr = std::shared_ptr<Initializer>;

  /**
   * @brief multi-case class for initializer
   * the system will not start while all the initializers are not ready
   */
  class MultiInitializer : public Initializer {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PARAM_VECTOR(srrg2_core::PropertyConfigurableVector_<Initializer>, initializers, "", nullptr);

    virtual void initialize() override {
      _initialized = true;
      for (size_t i = 0; i < param_initializers.size(); ++i) {
        auto initializer = param_initializers[i];
        initializer->setRawData(_message);
        initializer->initialize();
        _initialized &= initializer->isInitialized();
      }
    }
  };

} // namespace srrg2_slam_interfaces
