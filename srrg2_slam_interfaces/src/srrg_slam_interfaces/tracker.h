#pragma once
#include <srrg_messages/message_handlers/message_platform_sink.h>
#include <srrg_messages/messages/image_message.h> //ds visualization only
#include <srrg_viewer/drawable_base.h>

#include "merger.h"

namespace srrg2_slam_interfaces {

  // base class for a tracker
  // a tracker does the following:
  // 1. performs if needed the initialization of the scene
  // 2. if the scene is already initialized, it integrates the measurement
  //    and updates the current position
  // 3. it updates the status of the tracker
  // the function <track> is composed by the three of them
  //  - adaptMeasurements: takes the raw data and initializes all the PM
  //  - align: takes new data and tries to align to old one
  //  - merge: side effect on map
  class TrackerBase : public srrg2_core::MessagePlatformSink, public srrg2_core::DrawableBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    enum Status { Error = 0, Initializing = 1, Initialized = 2, Tracking = 3, Lost = 4 };

    TrackerBase() {
      this->_is_open = true;
    }

    virtual ~TrackerBase() {
    }

    // these are pure virtual, but the linked complains
    virtual void adaptMeasurements() = 0;
    virtual void align()             = 0; // ia this name sucks a lot - track is ok
    virtual void merge()             = 0;
    virtual void setMeasurement(srrg2_core::BaseSensorMessagePtr measurement_);
    virtual void compute();
    const Status status() const {
      return _status;
    }

    // ia sink stuff, override this if necessary
    bool putMessage(srrg2_core::BaseSensorMessagePtr msg_) override;

  protected:
    Status _status                                = Error;
    srrg2_core::BaseSensorMessagePtr _measurement = nullptr;
  };

  // specialization of the base tracker to an estimate type
  // it sets the estimate before the
  template <typename TransformType_, typename MeasurementSceneType_>
  class Tracker_ : public TrackerBase {
  public:
    using MeasurementSceneType = MeasurementSceneType_;
    using TransformType        = TransformType_;

    virtual void setEstimate(const TransformType& est) = 0;
    virtual const TransformType& estimate() const      = 0;
    inline int numFramesProcessed() const {
      return _num_frames_processed;
    }
    virtual MeasurementSceneType& measurementScene() = 0;

  protected:
    int _num_frames_processed = 0;
  };

} // namespace srrg2_slam_interfaces
