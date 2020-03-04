#pragma once
#include <srrg_config/property_configurable_vector.h>
#include <srrg_messages/messages/base_sensor_message.h>

namespace srrg2_slam_interfaces {

  class Initializer : public srrg2_core::Configurable, public srrg2_core::PlatformUser {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual ~Initializer() = default;

    void setMeasurement(srrg2_core::BaseSensorMessagePtr meas_) {
      _measurement = meas_;
    }

    virtual void initialize() = 0;
    const bool isInitialized() const {
      return _initialized;
    }

  protected:
    bool _initialized = false;

    srrg2_core::BaseSensorMessagePtr _measurement = nullptr;
  };

  using InitializerPtr = std::shared_ptr<Initializer>;

  class MultiInitializer : public Initializer {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PARAM_VECTOR(srrg2_core::PropertyConfigurableVector_<Initializer>, initializers, "", nullptr);

    virtual void initialize() override {
      _initialized = true;
      for (size_t i = 0; i < param_initializers.size(); ++i) {
        auto initializer = param_initializers[i];
        initializer->setMeasurement(_measurement);
        initializer->initialize();
        _initialized &= initializer->isInitialized();
      }
    }
  };

} // namespace srrg2_slam_interfaces
