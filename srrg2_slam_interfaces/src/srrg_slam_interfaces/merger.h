#pragma once
#include <srrg_config/configurable.h>
#include <srrg_data_structures/correspondence.h>
#include <srrg_data_structures/platform.h>
#include <srrg_messages/messages/base_sensor_message.h>

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;

  class MergerBase : public Configurable, public PlatformUser {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    enum Status { Error, Initializing, Success };
    virtual ~MergerBase();

    const Status status() const {
      return _status;
    }

  protected:
    Status _status = Error;
  };

  // Mother of merger (typed)
  // FixedType and MovingType can be anything
  template <typename TransformType_, typename SceneType_, typename MovingType_>
  class Merger_ : public MergerBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using TransformType = TransformType_;
    using MovingType    = MovingType_;
    using SceneType     = SceneType_;

    inline void setTransform(const TransformType& transform_) {
      _transform              = transform_;
      _transform_changed_flag = true;
    }

    inline const TransformType& transform() const {
      return _transform;
    }

    inline void setScene(SceneType* scene_) {
      _scene              = scene_;
      _scene_changed_flag = true;
    }

    inline void setMoving(const MovingType* moving_) {
      _moving              = moving_;
      _moving_changed_flag = true;
    }

    inline void setMeasurement(BaseSensorMessagePtr measurement_) {
      _measurement = measurement_;
    }

    virtual void compute() = 0;

  protected:
    const MovingType* _moving = nullptr;
    SceneType* _scene         = nullptr;
    TransformType _transform  = TransformType::Identity();
    BaseSensorMessagePtr _measurement;
    bool _moving_changed_flag    = true;
    bool _scene_changed_flag     = true;
    bool _transform_changed_flag = true;
  };

  //! correspondence-based (sparse) mother of mergers
  //! additionally to correspondences also current global pose estimates can be set (BA) uagh
  template <typename TransformType_, typename SceneType_, typename MovingType_>
  class MergerCorrespondence_ : public Merger_<TransformType_, SceneType_, MovingType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ThisType = MergerCorrespondence_<TransformType_, SceneType_, MovingType_>;
    using BaseType = Merger_<TransformType_, SceneType_, MovingType_>;

    PARAM(PropertyUnsignedInt,
          target_number_of_merges,
          "target number of points to merge, if hit no further points without correspondences will "
          "be added to moving (i.e. determines the pool of trackable points)",
          200,
          nullptr);

    PARAM(PropertyBool,
          enable_binning,
          "toggles point binning (distribution homogenization)",
          true,
          nullptr);

    virtual ~MergerCorrespondence_(){};

    using TransformType = TransformType_;
    using MovingType    = MovingType_;
    using SceneType     = SceneType_;

    inline void setCorrespondences(const CorrespondenceVector* correspondences_) {
      _correspondences              = correspondences_;
      _correspondences_changed_flag = true;
    }
    inline const CorrespondenceVector* correspondences() const {
      return _correspondences;
    }
    inline void setTrackerInWorld(const TransformType& tracker_in_world_) {
      _tracker_in_world = tracker_in_world_;
    }

  protected:
    const CorrespondenceVector* _correspondences = nullptr;
    bool _correspondences_changed_flag           = true;
    TransformType _tracker_in_world              = TransformType::Identity();
  };

} // namespace srrg2_slam_interfaces
