#include "tracker.h"

#include <srrg_messages/message_handlers/message_pack.h> //ds visualization only
#include <srrg_messages/messages/image_message.h>        //ds visualization only

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;

  void TrackerBase::compute() {
    adaptMeasurements();
    align();
    merge();
  }

  bool TrackerBase::putMessage(BaseSensorMessagePtr msg_) {
    setMeasurement(msg_);
    compute();
    return true;
  }

  void TrackerBase::setMeasurement(BaseSensorMessagePtr measurement_) {
    _measurement = measurement_;
  }

} // namespace srrg2_slam_interfaces
