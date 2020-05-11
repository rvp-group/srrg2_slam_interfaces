#pragma once
#include <srrg_config/configurable.h>
#include <srrg_data_structures/platform.h>
#include <srrg_messages/messages/base_sensor_message.h>

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;

  /**
   * @brief clipper interface
   * this class clips the scene around a desired location
   * defined by an estimate type (SE2/SE3) and the type of the scene to work on
   * computes the scene clipped around the given pose and presents the clipped scene in
   * robot coordinates to have the scene directly comparable with the measurement
   */
  template <typename EstimateType_, typename SceneType_>
  class SceneClipper_ : public Configurable, public PlatformUser {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**
     * @brief status of the scene clipper
     */
    enum Status {
      Error      = 0x0 /**< clipping not successful*/,
      Successful = 0x1, /**< scene successfully clipped */
      Ready      = 0x2  /**< clipped not performed due to empty scene */
    };
    using EstimateType = EstimateType_;
    using SceneType    = SceneType_;
    using ThisType     = SceneClipper_<EstimateType, SceneType>;
    using BaseType     = Configurable;

    /**
     * @brief compute the scene clipped in
     */
    virtual void compute() = 0;

    virtual void reset() {
      _clipped_scene_in_robot = 0;
      _full_scene             = 0;
      _robot_in_local_map.setIdentity();
      _status = Error;
    }

    /**
     * @brief set the output scene, will be placed in robot coords
     * @param[in] pointer to the output local clipped scene
     */
    inline void setClippedSceneInRobot(SceneType* scene_) {
      _clipped_scene_in_robot = scene_;
    }

    /**
     * @brief set the full scene
     * @param[in] pointer to the input global scene
     */
    inline void setFullScene(SceneType* scene_) {
      _full_scene = scene_;
    }

    /**
     * @brief set dynamic container for carrying auxiliary information (e.g. projections)
     * @param[in] auxiliary_data_: precomputed data to give to the clipper
     * @param[in] prefix_: unique tag to access the data
     */
    inline void setAuxiliaryData(PropertyContainerDynamic* auxiliary_data_,
                                 const std::string& prefix_ = "") {
      _auxiliary_data        = auxiliary_data_;
      _prefix_auxiliary_data = prefix_;
    }

    /**
     * @brief set the robot pose wrt the current map (or local map) in which it works
     * @param[in] robot_in_local_map_: robot pose wrt map origin
     */

    inline void setRobotInLocalMap(const EstimateType& robot_in_local_map_) {
      _robot_in_local_map = robot_in_local_map_;
      _local_map_in_robot = _robot_in_local_map.inverse();
    }

    /**
     * @brief cache sensor pose wrt the robot
     * @param[in] sensor_in_robot_: sensor in robot pose
     */
    inline void setSensorInRobot(const EstimateType& sensor_in_robot_) {
      _sensor_in_robot = sensor_in_robot_;
      _robot_in_sensor = _sensor_in_robot.inverse();
    }

    /**
     * @brief status getter
     * @return the status
     */
    inline Status status() const {
      return _status;
    }

    /**
     * @brief mapping from local (clipped scene) to global (scene) indices
     * required for e.g. aligner-based correspondence-based merging
     * @return vector where v[i] = global_idx
     */
    virtual const std::vector<int> globalIndices() const {
      // ds implementation is not enforced
      return std::vector<int>(0);
    }

  protected:
    SceneType* _clipped_scene_in_robot = nullptr; /**< resulting clipped scene in robot coordinates
    this is so, because the clipped scene will be compared directly with the measurement*/
    SceneType* _full_scene                    = nullptr; /**< whole scene to be clipped*/
    PropertyContainerDynamic* _auxiliary_data = nullptr; /** auxiliary data ptr*/
    std::string _prefix_auxiliary_data        = "";      /**< prefix to the auxiliary data  */
    EstimateType _robot_in_local_map =
      EstimateType::Identity(); /**< cached robot pose wrt the current local map*/
    EstimateType _local_map_in_robot =
      EstimateType::Identity(); /**< cached robot local map wrt the robot */
    EstimateType _sensor_in_robot = EstimateType::Identity(); /**< cached sensor pose wrt robot*/
    EstimateType _robot_in_sensor = EstimateType::Identity(); /**< cached robot pose wrt sensor */
    Status _status                = Error;                    /**< status of the clipper */
  };

} // namespace srrg2_slam_interfaces
