#include "local_map_selector_breadth_first.h"
#include "srrg_slam_interfaces/local_map.h"
#include "srrg_system_utils/shell_colors.h"

#define DEBUG(var) \
  if (var)         \
  std::cerr

namespace srrg2_slam_interfaces {
  static bool loop_detector_closure_selector_debug = 0;

  template <typename SLAMAlgorithmType_>
  void LocalMapSelectorBreadthFirst_<SLAMAlgorithmType_>::compute() {
    if (!this->_slam) {
      throw std::runtime_error("LocalMapSelectorBreadthFirst_::compute| no slam selected");
    }

    // fetch parameters
    float relocalize_range_scale = this->param_relocalize_range_scale.value();
    float aggressive_relocalize_graph_distance =
      this->param_aggressive_relocalize_graph_distance.value();
    float aggressive_relocalize_graph_max_range =
      this->param_aggressive_relocalize_graph_max_range.value();
    float aggressive_relocalize_range_increase_per_edge =
      this->param_aggressive_relocalize_range_increase_per_edge.value();
    float local_map_distance = this->param_max_local_map_distance.value();

    this->_hints.clear();

    // 1.: compute a spanning tree from the local map
    //    considering only "valid" closures

    FactorGraphInterfacePtr graph = this->_slam->graph();
    if (!graph) {
      throw std::runtime_error("LocalMapSelectorBreadthFirst_::compute| _graph is NULL");
    }
    LocalMapType* source_local_map = this->_slam->currentLocalMap();
    if (!source_local_map) {
      throw std::runtime_error(
        "LocalMapSelectorBreadthFirst_::compute| _current_local_map is NULL");
    }

    graph->bindFactors();
    this->_visit.setGraph(*graph);
    std::vector<VariableBase::Id> sources;
    sources.push_back(source_local_map->graphId());
    this->_visit.setSources(sources);
    this->_visit.compute();
    EstimateType robot_pose = this->_slam->robotPose();
    //    EstimateType robot_pose_in_local_map = this->_slam->robotPoseInCurrentLocalMap();

    // 2.: while computing the tree, label each local map in range
    //    as "near" or "far"

    DEBUG(loop_detector_closure_selector_debug) << "LocalMapSelectorBreadthFirst_::compute|search: "
                                                << FG_BGREEN(source_local_map->graphId());
    EstimateType inv_pose = robot_pose.inverse(); // global pose
    //    EstimateType pose_in_current = robot_pose_in_local_map; // pose in the current map
    float max_cost = 0;
    // DEBUG(loop_detector_closure_selector_debug) << "fixed set" << std::endl;
    // for (auto it:_current_local_map->dynamic_properties.properties()){
    //   DEBUG(loop_detector_closure_selector_debug) << "p: " << it.first << std::endl;
    // }
    for (auto v : graph->variables()) {
      if (v.second == source_local_map) {
        continue;
      }
      LocalMapType* target_local_map =
        const_cast<LocalMapType*>(dynamic_cast<const LocalMapType*>(v.second));
      if (!target_local_map) {
        continue;
      }

      EstimateType delta = inv_pose * target_local_map->estimate();
      EstimateType target_in_source_initial_guess =
        source_local_map->estimate().inverse() * target_local_map->estimate();

      // get the cost
      VariableVisitEntry* entry = this->_visit.entries().at(target_local_map->graphId());
      max_cost                  = std::max(max_cost, entry->cost);

      // determine the threshold for matching (in local map units)
      float range_scale =
        (relocalize_range_scale * entry->cost * aggressive_relocalize_range_increase_per_edge) + 1;
      range_scale = std::min(range_scale, aggressive_relocalize_graph_max_range);

      float translation_threshold = local_map_distance * range_scale;
      if (delta.translation().norm() > translation_threshold) {
        continue;
      }

      // aggressive relocalization if distnace on graph greater than threshold
      if (entry->cost > aggressive_relocalize_graph_distance) {
        target_in_source_initial_guess.translation().setZero();
      }
      ClosureHintPtr hint(new ClosureHint(
        target_local_map, target_in_source_initial_guess, InformationMatrixType::Identity()));
      this->_hints.insert(hint);
    }
    DEBUG(loop_detector_closure_selector_debug) << " MAX_COST: " << max_cost << std::endl;
  }

} // namespace srrg2_slam_interfaces
#undef DEBUG
