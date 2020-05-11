#pragma once
#include "srrg2_slam_interfaces/initializers/initializer.h"
#include "srrg2_slam_interfaces/mapping/local_map.h"
#include "srrg2_slam_interfaces/mapping/local_map_splitting_criterions/local_map_splitting_criterion_rotation.hpp"
#include "srrg2_slam_interfaces/mapping/local_map_splitting_criterions/local_map_splitting_criterion_translation.hpp"
#include "srrg2_slam_interfaces/mapping/local_map_splitting_criterions/local_map_splitting_criterion_viewpoint.hpp"
#include "srrg2_slam_interfaces/mapping/local_map_splitting_criterions/local_map_splitting_criterion_visibility.hpp"
#include "srrg2_slam_interfaces/registration/loop_closure.h"
#include "srrg2_slam_interfaces/registration/loop_detector/loop_detector.h"
#include "srrg2_slam_interfaces/registration/relocalization/multi_relocalizer.h"
#include "srrg2_slam_interfaces/trackers/multi_tracker.h"
#include <srrg_solver/solver_core/factor_graph.h>
#include <srrg_solver/solver_core/solver.h>
#include <srrg_solver/utils/factor_graph_utils/factor_graph_closure_validator.h>
#include <srrg_solver/utils/factor_graph_utils/instances.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_viewer/drawable_base.h>

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;
  using namespace srrg2_solver;

  /**
   * @brief the SLAM algorithm class
   * Here everything is performed to produce a map
   */
  template <typename LoopClosureType_>
  class MultiGraphSLAM_ : public srrg2_core::MessagePlatformSink,
                          public srrg2_core::DrawableBase,
                          public srrg2_core::Profiler {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using LoopClosureType        = LoopClosureType_;
    using LoopClosureTypePtr     = std::shared_ptr<LoopClosureType>;
    using LocalMapType           = typename LoopClosureType::LocalMapType;
    using VariableType           = typename LocalMapType::VariableType;
    using EstimateType           = typename VariableType::EstimateType;
    using TrackerType            = MultiTrackerBase_<EstimateType>;
    using AlignerType            = MultiAlignerBase_<VariableType>;
    using ThisType               = MultiGraphSLAM_<LoopClosureType>;
    using LoopDetectorType       = LoopDetector_<ThisType>;
    using RelocalizerType        = MultiRelocalizer_<ThisType>;
    using SplittingCriterionType = LocalMapSplittingCriterionBase_<ThisType>;
    using FactorBaseType         = typename LoopClosureType::FactorType;
    using FactorBaseTypePtr      = std::shared_ptr<FactorBaseType>;
    using InformationMatrixType  = typename LoopClosureType::FactorType::InformationMatrixType;
    using BaseType               = srrg2_core::MessagePlatformSink;

    PARAM(PropertyConfigurable_<TrackerType>, tracker, "incremental tracker", nullptr, nullptr);
    PARAM(PropertyConfigurable_<Solver>,
          global_solver,
          "global solver for loop closures",
          nullptr,
          nullptr);
    PARAM(PropertyConfigurable_<LoopDetectorType>,
          loop_detector,
          "detector used to produce loop closures",
          nullptr,
          nullptr);
    PARAM(PropertyConfigurable_<FactorGraphClosureValidator>,
          closure_validator,
          "validator used to confirm loop closures",
          nullptr,
          nullptr);
    PARAM(PropertyConfigurable_<RelocalizerType>,
          relocalizer,
          "relocalizer to avoid creation of new nodes",
          nullptr,
          nullptr);
    PARAM(PropertyConfigurable_<SplittingCriterionType>,
          splitting_criterion,
          "heuristics that determine when a new local map has to be generated",
          nullptr,
          nullptr);
    PARAM(PropertyConfigurable_<Initializer>,
          initializer,
          "initialization algorithm",
          nullptr,
          nullptr);
    MultiGraphSLAM_();
    /**
     * @brief give an already created graph to the system
     * @param[in] the factor graph
     */
    inline void setGraph(FactorGraphPtr graph_) {
      _graph = graph_;
      _robot_in_local_map.setIdentity();
      _current_local_map = nullptr;
    }
    /**
     * @brief retrieve the current graph
     */
    inline FactorGraphPtr graph() {
      return _graph;
    }

    /**
     * @brief auto-run of the SLAM algorithm, for shell compatibility
     * @param[in] the current message
     */
    bool putMessage(BaseSensorMessagePtr message_) override;
    /**
     * @brief gather the given message
     * @param[in] the current message
     */
    inline void setRawData(BaseSensorMessagePtr message_) {
      assert(message_);
      _message = message_;
    }

    /**
     * @brief performs SLAM
     */
    void compute();

    /**
     * @brief broadcast the platform to all the internal modules who needs it
     */
    void setPlatform(PlatformPtr platform_) override {
      std::cerr << "MultiGraphSLAM::setPlatform|handle: " << platform_ << std::endl;
      BaseType::setPlatform(platform_);
      if (param_tracker.value()) {
        param_tracker->setPlatform(platform_);
      }
      if (param_initializer.value()) {
        param_initializer->setPlatform(platform_);
      }
    }

    /**
     * @brief the global pose of the robot getter
     * @return global pose of the robot
     */
    inline const EstimateType robotInWorld() const {
      if (!_current_local_map) {
        std::cerr << "MultiGraphSLAM::robotPose|WARNING: no local map active, returning Identity"
                  << std::endl;
        return EstimateType::Identity();
      }
      return _current_local_map->estimate() * _robot_in_local_map;
    }

    /**
     * @brief the local pose of the robot getter
     * @return robot pose in local map
     */
    inline const EstimateType& robotInLocalMap() const {
      return _robot_in_local_map;
    }

    /**
     * @brief the local pose of the robot setter
     * @param[in] robot pose in local map
     */
    inline void setRobotInLocalMap(const EstimateType& estimate_) {
      _robot_in_local_map = estimate_;
    }

    /**
     * @brief current local map getter
     * @return current local map
     */
    inline LocalMapType* currentLocalMap() {
      return _current_local_map;
    }

    /**
     * @brief current local map setter
     * @param[in] current local map
     */
    inline void setCurrentLocalMap(LocalMapType* local_map_) {
      assert(local_map_);
      _current_local_map = local_map_;
    }

    /**
     * @brief checks if relocalization happened
     */
    bool relocalized() const {
      return (_relocalized_closure != nullptr);
    }

    void draw(ViewerCanvasPtr canvas) const override;

  protected:
    //! @brief find loop closures, side effect on the pending closures
    //! updates _detected_closures
    void loopDetect();

    //! @brief validates the closures after loopDetect()
    //! updates _valid_closures and _relocalize_closures
    //! @param[in] set of relocalization closures to be validated
    void loopValidate(GraphItemPtrSet_<LoopClosureTypePtr>& reloc_closures_);

    //! @brief optimizes the graph, if needed
    void optimize();

    //! @brief optimizes the graph, reads from _relocalize_closures
    //! @param[in] set of relocalization closures
    void relocalize(const GraphItemPtrSet_<LoopClosureTypePtr>& reloc_closures_);

    //! @brief makes a new map around current position of the robot
    //! @param[in] information matrix scale factor
    void makeNewMap(float info_scale = 1);

    //! @brief updates the flag in the local maps to reflect the status
    void adjustDrawingAttributes() const;

    EstimateType _robot_in_local_map =
      EstimateType::Identity(); /**< pose of the robot in the current local map*/
    LocalMapType* _current_local_map        = nullptr; /**< the current local map*/
    FactorGraphPtr _graph                   = nullptr; /**< the current graph */
    LoopClosureTypePtr _relocalized_closure = nullptr; /**< loop closure found for relocalization*/
    size_t num_valid_closures               = 0;       //< if 0 no optimization needed

    InformationMatrixType _default_info =
      InformationMatrixType::Identity();     /**< information matrix for local map */
    BaseSensorMessagePtr _message = nullptr; /**< the current messaeg  */

    size_t _number_of_processed_measurements = 0; /**< pose of the robot in the current local map*/
  };

} // namespace srrg2_slam_interfaces
