#pragma once
#include <srrg_config/configurable.h>
#include <srrg_data_structures/correspondence.h>
#include <srrg_data_structures/platform.h>
#include <srrg_messages/messages/base_sensor_message.h>

namespace srrg2_slam_interfaces {

  /**
   * @brief minimal interface to the merger
   * just implements the status
   */
  class MergerBase : public srrg2_core::Configurable, public srrg2_core::PlatformUser {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**
     * @brief status of the merging process
     */
    enum Status {
      Error        = 0x0 /**< merging not successful*/,
      Initializing = 0x1 /**< merger is ready with a scene*/,
      Success      = 0x2 /**< merging successful*/
    };

    /**
     * @brief dtor
     */
    virtual ~MergerBase() = default;

    /**
     * @brief status getter
     * @return the status
     */
    const Status status() const {
      return _status;
    }

  protected:
    Status _status = Error; /**< current status of the merger*/
  };

  /**
   * @brief basic merger interface
   * defined via an estimate type (SE2/SE3), a scene type and a measurement type.
   * The scene is considered to be the fixed part to which apply the merging measurement
   * which is displaced by ``_measurement_in_scene`` w.r.t the scene origin
   */
  template <typename EstimateType_, typename FixedSceneType_, typename MovingMeasurementType_>
  class Merger_ : public MergerBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using EstimateType          = EstimateType_;
    using FixedSceneType        = FixedSceneType_;
    using MovingMeasurementType = MovingMeasurementType_;
    /**
     * @brief set the offset between scene and measurement origin (sensor)
     * @param[in] measurement_in_scene_: displacement between the scene and the measurement
     */
    inline void setMeasurementInScene(const EstimateType& measurement_in_scene_) {
      _measurement_in_scene       = measurement_in_scene_;
      _meas_in_scene_changed_flag = true;
    }

    /**
     * @brief offset getter
     * @return displacement of the measurement in scene coordinates
     */
    inline const EstimateType& measurementInScene() const {
      return _measurement_in_scene;
    }

    /**
     * @brief set the scene slice. After compute this scene will be modified
     * @param[in] scene_: scene container
     */
    inline void setScene(FixedSceneType* scene_) {
      _scene              = scene_;
      _scene_changed_flag = true;
    }

    /**
     * @brief set the measurement slice.
     * @param[in] moving_: measurement container
     */
    inline void setMeasurement(const MovingMeasurementType* measurement_) {
      _measurement       = measurement_;
      _meas_changed_flag = true;
    }

    //    inline void setMeasurement(srrg2_core::BaseSensorMessagePtr measurement_) {
    //      _measurement = measurement_;
    //    }

    /**
     * @brief performs merging
     */
    virtual void compute() = 0;

  protected:
    const MovingMeasurementType* _measurement =
      nullptr; /**< pointer to the measurement that will be added in the scene*/
    FixedSceneType* _scene = nullptr; /**< pointer to the scene that will be enhanced*/
    EstimateType _measurement_in_scene =
      EstimateType::Identity(); /**< offset of the measurement wrt the scene*/
    //    srrg2_core::BaseSensorMessagePtr _message = nullptr;
    bool _meas_changed_flag          = true; /**< meas change control flag*/
    bool _scene_changed_flag         = true; /**< scene change control flag*/
    bool _meas_in_scene_changed_flag = true; /**< transform change control flag*/
  };

  /**
   * @brief sparse correspondence-based mergers interface
   * additionally to correspondences also current global pose estimates can be set (BA) (?)
   */
  template <typename EstimateType_, typename FixedSceneType_, typename MovingMeasurementType_>
  class MergerCorrespondence_
    : public Merger_<EstimateType_, FixedSceneType_, MovingMeasurementType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using EstimateType          = EstimateType_;
    using FixedSceneType        = FixedSceneType_;
    using MovingMeasurementType = MovingMeasurementType_;
    using ThisType = MergerCorrespondence_<EstimateType, FixedSceneType, MovingMeasurementType>;
    using BaseType = Merger_<EstimateType, FixedSceneType, MovingMeasurementType>;

    PARAM(srrg2_core::PropertyUnsignedInt,
          target_number_of_merges,
          "target number of points to merge, if hit no further points without correspondences will "
          "be added to moving (i.e. determines the pool of trackable points)",
          200,
          nullptr);

    PARAM(srrg2_core::PropertyBool,
          enable_binning,
          "toggles point binning (distribution homogenization)",
          true,
          nullptr);

    virtual ~MergerCorrespondence_() = default;

    /**
     * @brief set the scene-measurement correspondence vector
     * @param[in] correspondences_: the corr vector
     */
    inline void setCorrespondences(const srrg2_core::CorrespondenceVector* correspondences_) {
      _correspondences              = correspondences_;
      _correspondences_changed_flag = true;
    }

    /**
     * @brief correspondence vector retriever
     * @return the corr vector
     */
    inline const srrg2_core::CorrespondenceVector* correspondences() const {
      return _correspondences;
    }

    /**
     * @brief set the pose of the measurement origin (sensor) w.r.t the world (e.g. for landmark
     * estimation)
     * @param[in] measurement_in_world_: pose of the measurement in world coordinates
     */
    inline void setMeasurementInWorld(const EstimateType& measurement_in_world_) {
      _measurement_in_world = measurement_in_world_;
      _world_in_measurement = _measurement_in_world.inverse();
    }

  protected:
    const srrg2_core::CorrespondenceVector* _correspondences =
      nullptr; /**< the correspondence vector, where for each entry corr(0)=scene_idx
                  corr(1)=meas_idx*/
    bool _correspondences_changed_flag = true; /**< corr vector change control flag*/
    EstimateType _measurement_in_world =
      EstimateType::Identity(); /**< pose of the measurement origin (sensor) in world coordinates*/
    EstimateType _world_in_measurement =
      EstimateType::Identity(); /**< pose of the world in measurement origin (sensor) */
  };

} // namespace srrg2_slam_interfaces
