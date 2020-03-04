#pragma once
#include "initializers/initializer.h"
#include "local_map.h"
#include "loop_closure.h"
#include "loop_detector.h"
#include "multi_relocalizer.h"
#include "multi_tracker.h"

#include "srrg_solver/utils/factor_graph_utils/factor_graph_closure_validator.h"
#include "srrg_solver/utils/factor_graph_utils/instances.h"
#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_solver/solver_core/solver.h"
#include "srrg_system_utils/system_utils.h"
#include "srrg_viewer/drawable_base.h"

#include "local_map_splitting_criterions/local_map_splitting_criterion_rotation.hpp"
#include "local_map_splitting_criterions/local_map_splitting_criterion_translation.hpp"
#include "local_map_splitting_criterions/local_map_splitting_criterion_viewpoint.hpp"
#include "local_map_splitting_criterions/local_map_splitting_criterion_visibility.hpp"

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;
  using namespace srrg2_solver;

  template <typename LoopClosureType_>
  class MultiGraphSLAM_ : public srrg2_core::MessagePlatformSink,
                          public srrg2_core::DrawableBase,
                          public srrg2_core::Profiler {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using LoopClosureType        = LoopClosureType_;
    using LoopClosurePtrType     = std::shared_ptr<LoopClosureType>;
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
    void setGraph(FactorGraphPtr graph_);
    inline FactorGraphPtr graph() {
      return _graph;
    }

    // ds TODO pick either one of the two and banish the other forever
    bool putMessage(BaseSensorMessagePtr measurement_) override;
    inline void setMeasurement(BaseSensorMessagePtr measurement_) {
      assert(measurement_);
      _measurement = measurement_;
    }

    void compute();

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
    EstimateType robotPose() const;
    inline const EstimateType& robotPoseInCurrentLocalMap() const {
      return _robot_pose_in_local_map;
    }
    inline void setRobotPoseInCurrentLocalMap(const EstimateType& estimate_) {
      _robot_pose_in_local_map = estimate_;
    }
    inline LocalMapType* currentLocalMap() {
      return _current_local_map;
    }
    inline void setCurrentLocalMap(LocalMapType* local_map_) {
      assert(local_map_);
      _current_local_map = local_map_;
    }
    bool relocalized() const {
      return (_relocalized_closure != nullptr);
    }

    void draw(ViewerCanvasPtr canvas) const override;

  protected:
    // find loop closures, side effect on the pending closures
    // updates _detected_closures
    void loopDetect();

    // validates the closures after loopDetect()
    // updates _valid_closures and _relocalize_closures
    void loopValidate(GraphItemPtrSet_<LoopClosurePtrType>& reloc_closures_);

    // optimizes the graph, if needed
    void optimize();

    // optimizes the graph, reads from _relocalize_closures
    void relocalize(const GraphItemPtrSet_<LoopClosurePtrType>& reloc_closures_);

    // makes a new map around current position of the robot
    void makeNewMap(float info_scale = 1);

    // updates the flag in the local maps to reflect the status
    void adjustDrawingAttributes() const;

    EstimateType _robot_pose_in_local_map   = EstimateType::Identity();
    LocalMapType* _current_local_map        = nullptr;
    FactorGraphPtr _graph                   = nullptr;
    LoopClosurePtrType _relocalized_closure = nullptr;
    size_t num_valid_closures               = 0; // if 0 no optimization needed

    InformationMatrixType _default_info = InformationMatrixType::Identity();
    BaseSensorMessagePtr _measurement   = nullptr;

    // ds stats only TODO move into stats property
    size_t _number_of_processed_measurements = 0;
  };

  template <typename LoopClosureType_>
  void MultiGraphSLAM_<LoopClosureType_>::setGraph(FactorGraphPtr graph_) {
    _graph = graph_;
    _robot_pose_in_local_map.setIdentity();
    _current_local_map = nullptr;
  }

  template <typename LoopClosureType_>
  typename MultiGraphSLAM_<LoopClosureType_>::EstimateType
  MultiGraphSLAM_<LoopClosureType_>::robotPose() const {
    if (!_current_local_map) {
      std::cerr << "MultiGraphSLAM::robotPose|WARNING: no local map active, returning Identity"
                << std::endl;
      return EstimateType::Identity();
    }
    return _current_local_map->estimate() * _robot_pose_in_local_map;
  }

} // namespace srrg2_slam_interfaces
