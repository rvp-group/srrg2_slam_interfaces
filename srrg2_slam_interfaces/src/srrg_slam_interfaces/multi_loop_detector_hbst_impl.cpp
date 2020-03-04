#pragma once
#include "multi_loop_detector_hbst.h"

namespace srrg2_slam_interfaces {

#define MultiLoopDetectorHBST                                                         \
  template <typename SLAMAlgorithmType_, typename AlignerType_, typename FactorType_> \
  void MultiLoopDetectorHBST_<SLAMAlgorithmType_, AlignerType_, FactorType_>

  MultiLoopDetectorHBST::compute() {
    ThisType::_attempted_closures.clear();
    ThisType::_detected_closures.clear();
    _indices.clear();
    _correspondences_per_reference.clear();
    if (!ThisType::_slam) {
      throw std::runtime_error("MultiLoopDetectorHBST::compute|ERROR: no SLAM system set");
    }
    if (!ThisType::_slam->currentLocalMap()) {
      throw std::runtime_error("MultiLoopDetectorHBST::compute|ERROR: no local map set");
    }

    // ds populate measurement for correspondence computation w.r.t. database TODO uagh
    _current_local_map = ThisType::_slam->currentLocalMap();

    // ds compute correspondence vectors to database for current local map
    computeCorrespondences();

    // ds compute alignment for each correspondence vector
    _computeAlignments();

    // ds done
    if (!ThisType::_detected_closures.empty()) {
      std::cerr << FG_GREEN("MultiLoopDetectorHBST::compute|detected closures: ")
                << FG_GREEN(ThisType::_detected_closures.size()) << std::endl;
    }
  }

  MultiLoopDetectorHBST::addPreviousQuery() {
    if (!_current_local_map) {
      return;
    }

    // ds skip empty training request
    if (_query_matchables.empty()) {
      return;
    }

    // ds skip already added local maps (happens when we move from relocalization to mapping)
    const size_t local_map_graph_id = _current_local_map->graphId();
    assert(local_map_graph_id >= 0);
    if (_graph_id_to_database_index.find(local_map_graph_id) != _graph_id_to_database_index.end()) {
      return;
    }
    const size_t index = _database.size();
    assert(_query_matchables[0]->objects.size() == 1);
    assert(_query_matchables[0]->objects.begin()->first == index);

    // ds register database index w.r.t the local map in the graph
    _local_maps_in_database.push_back(_current_local_map);
    _graph_id_to_database_index.insert(std::make_pair(local_map_graph_id, index));
    _current_local_map = nullptr;

    // ds integrate previous query into the database
    _database.add(_query_matchables, srrg_hbst::SplittingStrategy::SplitEven);
    assert(_database.size() == _local_maps_in_database.size());
    assert(_database.size() == _graph_id_to_database_index.size());
    _query_matchables.clear();
  }

  MultiLoopDetectorHBST::computeCorrespondences() {
    if (!_current_local_map) {
      throw std::runtime_error("MultiLoopDetectorHBST::compute|ERROR: no measurements set");
    }

    // ds retrieve measurements in supported descriptor format from the slice
    SliceTypePtr slice = nullptr;
    _retrieveSliceFromAligner(slice);
    _retrieveDescriptorsFromLocalMap(
      _current_local_map, slice->param_fixed_slice_name.value(), _fixed_current_local_map);
    if (_fixed_current_local_map->empty()) {
      std::cerr << "MultiLoopDetectorHBST::compute|WARNING: query descriptor vector is empty"
                << std::endl;
      _current_local_map = nullptr;
      return;
    }

    // ds if we have to update the database configuration
    // ds this should be done only at startup as it might corrupt the database!
    if (_config_changed) {
      DatabaseType::maximum_distance_for_merge = param_maximum_distance_for_merge.value();
      DatabaseType::Node::maximum_depth        = param_maximum_depth.value();
      DatabaseType::Node::maximum_leaf_size    = param_maximum_leaf_size.value();
      DatabaseType::Node::maximum_partitioning = param_maximum_partitioning.value();
      _config_changed                          = false;
    }

    // ds check if we have unadded matchables
    if (!_query_matchables.empty()) {
      // ds we have to free the matchables from the previous call
      for (const DatabaseType::Matchable* matchable : _query_matchables) {
        delete matchable;
      }
      _query_matchables.clear();
    }

    // ds cache
    const size_t number_of_query_descriptors = _fixed_current_local_map->size();
    const PointDescriptorVectorType& points_fixed(*_fixed_current_local_map);
    _query_matchables.reserve(number_of_query_descriptors);
    assert(_current_local_map->graphId() >= 0);

    // ds determine query index - for a new local map it corresponds to the database size
    uint64_t index_query = _database.size();

    // ds check if we already registered this local map in our database (by graph identifier)
    const auto iterator = _graph_id_to_database_index.find(_current_local_map->graphId());
    if (iterator != _graph_id_to_database_index.end()) {
      // ds pick original index for matching
      index_query = iterator->second;
    }

    // ds convert the descriptors to HBST matchables, linking by increasing point index
    for (uint64_t index_descriptor = 0; index_descriptor < number_of_query_descriptors;
         ++index_descriptor) {
      if (points_fixed[index_descriptor].status == POINT_STATUS::Valid) {
        _query_matchables.emplace_back(new DatabaseType::Matchable(
          index_descriptor, points_fixed[index_descriptor].descriptor(), index_query));
      }
    }

    // ds query the database for matches - match addition has to be triggered externally
    DatabaseType::MatchVectorMap matches_per_reference;
    _database.match(_query_matchables,
                    matches_per_reference,
                    ThisType::param_maximum_descriptor_distance.value());

    // ds filter match vector according to closure constraints
    _indices.reserve(matches_per_reference.size());
    _correspondences_per_reference.reserve(matches_per_reference.size());
    for (const auto& entry : matches_per_reference) {
      // ds check if age difference between the query and reference candidate is sufficiently high
      if (std::fabs(index_query - entry.first) >
          param_minimum_age_difference_to_candidates.value()) {
        const DatabaseType::MatchVector& matches = entry.second;
        const size_t& number_of_matches          = matches.size();
        if (number_of_matches > ThisType::param_relocalize_min_inliers.value()) {
          const size_t index_reference = entry.first;

          // ds register filtered candidates for optional external manipulation
          CorrespondenceVector correspondences;
          _computeCorrespondencesFromMatches(matches, correspondences);
          _correspondences_per_reference.insert(
            std::make_pair(index_reference, std::move(correspondences)));
          _indices.emplace_back(index_reference);
        }
      }
    }
  }

  MultiLoopDetectorHBST::_computeAlignments() {
    if (!_fixed_current_local_map) {
      throw std::runtime_error(
        "MultiLoopDetectorHBST::computeAlignments|ERROR: current local map not set");
    }

    // ds retrieve aligner and its solver unit
    std::shared_ptr<AlignerType> aligner = ThisType::param_relocalize_aligner.value();
    if (!aligner) {
      throw std::runtime_error(
        "MultiLoopDetectorHBST::computeAlignments|ERROR: aligner not set (cannot "
        "compute spatial relation between query and database)");
    }
    if (!aligner->param_solver.value()) {
      throw std::runtime_error(
        "MultiLoopDetectorHBST::computeAlignments|ERROR: solver not set (cannot "
        "compute spatial relation between query and database)");
    }

    // ds cache graph, must be available
    FactorGraphInterfacePtr graph = ThisType::_slam->graph();
    if (!graph) {
      throw std::runtime_error("MultiLoopDetectorHBST::computeAlignments|ERROR: graph not set");
    }

    // ds cache current pose
    const EstimateType& pose_in_current = ThisType::_slam->robotPoseInCurrentLocalMap();

    // ds parse moving slice name - must be the same for all reference local maps
    SliceTypePtr slice = nullptr;
    _retrieveSliceFromAligner(slice);
    const std::string slice_point_cloud_name_moving = slice->param_moving_slice_name.value();
    assert(slice->param_robustifier.value());

    // ds for all query - reference pairs with sufficient correspondences
    for (auto& loop_closure_candidate : _correspondences_per_reference) {
      const size_t& index_reference         = loop_closure_candidate.first;
      CorrespondenceVector& correspondences = loop_closure_candidate.second;

      // ds check if target cannot be satisfied before computing
      if (correspondences.size() < ThisType::param_relocalize_min_inliers.value()) {
        continue;
      }

      // ds grab reference local map from SLAM system
      assert(index_reference < _local_maps_in_database.size());
      LocalMapType* reference_local_map = _local_maps_in_database[index_reference];
      assert(reference_local_map);
      assert(_current_local_map != reference_local_map);

      // ds retrieve descriptors from reference local map to compute relative transform
      _retrieveDescriptorsFromLocalMap(
        reference_local_map, slice_point_cloud_name_moving, _moving_past_local_map);

      // ds compute relative transform between query and reference directly with the solver
      // ds populate factors with HBST correspondences TODO refactor
      assert(_fixed_current_local_map);
      assert(_moving_past_local_map);
      // solver->clearFactorIterators();
      // std::vector<FactorBaseType*> factors;
      // factors.reserve(correspondences.size());
      // for (const Correspondence& correspondence : correspondences) {
      //   assert(static_cast<size_t>(correspondence.fixed_idx) < points_fixed.size());
      //   assert(static_cast<size_t>(correspondence.moving_idx) < points_moving.size());
      //   FactorType* factor(new FactorType());
      //   factor->setInformationMatrix(InformationMatrixType::Identity());
      //   factor->setEnabled(true);
      //   factor->setRobustifier(slice->param_robustifier.template getRawPtr<RobustifierBase>());
      //   factor->bindFixed(&points_fixed[correspondence.fixed_idx]);
      //   factor->bindMoving(&points_moving[correspondence.moving_idx]);
      //   factors.emplace_back(factor);
      // }
      // assert(factors.size() == correspondences.size());
      // solver->addFactorContainer(factors);
      std::shared_ptr<FactorCorrespondenceDrivenType> factor(new FactorCorrespondenceDrivenType());
      factor->setFixed(*_fixed_current_local_map);
      factor->setMoving(*_moving_past_local_map);
      factor->setCorrespondences(correspondences);
      factor->setVariableId(0, 0);
      factor->setRobustifier(slice->param_robustifier.template getRawPtr<RobustifierBase>());

      // ds hook up to solver TODO refactor
      std::shared_ptr<VariableType> variable_fixed_from_moving(new VariableType());
      variable_fixed_from_moving->setGraphId(0);
      variable_fixed_from_moving->setEstimate(EstimateType::Identity());
      FactorGraphPtr single_pose_graph(new FactorGraph());
      single_pose_graph->addVariable(variable_fixed_from_moving);
      single_pose_graph->addFactor(factor);
      aligner->param_solver->setGraph(single_pose_graph);

      // ds we keep the correspondences locked during optimization TODO refactor
      for (size_t i = 0; i < static_cast<size_t>(aligner->param_max_iterations.value()); ++i) {
        aligner->param_solver->compute();
      }

      // ds attempt to add loop closure (still has to pass some heuristics)
      const EstimateType estimate_fixed_from_moving(variable_fixed_from_moving->estimate());
      _addLoopClosure(pose_in_current,
                      estimate_fixed_from_moving,
                      aligner->param_solver->iterationStats().back(),
                      _current_local_map,
                      reference_local_map,
                      correspondences);
    }
  }

  MultiLoopDetectorHBST::_computeCorrespondencesFromMatches(
    const DatabaseType::MatchVector& matches_,
    CorrespondenceVector& correspondences_) const {
    correspondences_.clear();

    // ds convert matches into correspondence candidates by best distance TODO add Lowe check
    std::unordered_map<size_t /*reference*/, Correspondence> candidates;
    candidates.reserve(matches_.size());
    for (const DatabaseType::Match& match : matches_) {
      // ds only pick non-ambiguous matches
      if (match.object_references.size() == 1) {
        const size_t& index_descriptor_reference = match.object_references[0];
        auto iterator                            = candidates.find(index_descriptor_reference);
        if (iterator != candidates.end()) {
          if (match.distance < iterator->second.response) {
            iterator->second.response   = match.distance;
            iterator->second.fixed_idx  = match.object_query;
            iterator->second.moving_idx = index_descriptor_reference;
          }
        } else {
          candidates.insert(std::make_pair(
            index_descriptor_reference,
            Correspondence(match.object_query, index_descriptor_reference, match.distance)));
        }
      }
    }

    // ds register filtered candidates
    correspondences_.reserve(matches_.size());
    for (auto& candidate : candidates) {
      correspondences_.emplace_back(std::move(candidate.second));
    }
  }

  MultiLoopDetectorHBST::_retrieveSliceFromAligner(SliceTypePtr& slice_) {
    // ds retrieve slices to process from aligner unit
    std::shared_ptr<AlignerType> aligner = ThisType::param_relocalize_aligner.value();
    if (!aligner) {
      throw std::runtime_error(
        "MultiLoopDetectorHBST::_retrieveSliceFromAligner|ERROR: aligner not set (cannot "
        "compute spatial relation between query and database)");
    }
    if (aligner->param_slice_processors.empty()) {
      throw std::runtime_error(
        "MultiLoopDetectorHBST::_retrieveSliceFromAligner|ERROR: no slices available");
    }
    if (aligner->param_slice_processors.size() != 1) {
      std::cerr << "MultiLoopDetectorHBST::_retrieveSliceFromAligner|WARNING: received "
                << aligner->param_slice_processors.size()
                << " slices "
                   "for alignment, current implementation only supports 1"
                << std::endl;
    }

    // ds pick the first slice for loop closing TODO alternatives?
    slice_ = aligner->param_slice_processors.template getSharedPtr<SliceType>(0);
    assert(slice_);
  }

  MultiLoopDetectorHBST::_retrieveDescriptorsFromLocalMap(
    LocalMapType* local_map_,
    const std::string& slice_point_cloud_name_,
    const PointDescriptorVectorType*& descriptors_) {
    assert(local_map_);
    if (local_map_->properties().empty()) {
      throw std::runtime_error("MultiLoopDetectorHBST::_retrieveDescriptorsFromLocalMap|ERROR: "
                               "local map contains no properties");
    }

    // ds parse point cloud property name from properties
    DescriptorPropertyType* property =
      dynamic_cast<DescriptorPropertyType*>(local_map_->property(slice_point_cloud_name_));
    if (!property) {
      throw std::runtime_error("MultiLoopDetectorHBST::_retrieveDescriptorsFromLocalMap|ERROR: "
                               "unable to retrieve measurements property");
    }
    descriptors_ = property->value();
    if (!descriptors_) {
      throw std::runtime_error("MultiLoopDetectorHBST::_retrieveDescriptorsFromLocalMap|ERROR: "
                               "unable to retrieve measurements from property");
    }
  }

  MultiLoopDetectorHBST::_addLoopClosure(const EstimateType& pose_in_current_,
                                         const EstimateType& estimate_,
                                         const IterationStats& solver_statistics_,
                                         LocalMapType* local_map_current_,
                                         LocalMapType* local_map_reference_,
                                         CorrespondenceVector& correspondences_) {
    // ds set optimization statistics
    const size_t& number_of_inliers = solver_statistics_.num_inliers;
    const size_t number_of_correspondences =
      number_of_inliers + solver_statistics_.num_outliers + solver_statistics_.num_suppressed;
    const float average_chi_per_inlier = solver_statistics_.chi_inliers / number_of_inliers;
    const float inlier_ratio = static_cast<float>(number_of_inliers) / number_of_correspondences;
    assert(number_of_correspondences == correspondences_.size());

    // ds skip closure if statistics are insufficient
    if (number_of_inliers < ThisType::param_relocalize_min_inliers.value() ||
        average_chi_per_inlier > ThisType::param_relocalize_max_chi_inliers.value() ||
        inlier_ratio < ThisType::param_relocalize_min_inliers_ratio.value()) {
      //      std::cerr << "MultiLoopDetectorHBST::compute|dropped closure: # inliers: "
      //                << number_of_inliers << " chi: " << average_chi_per_inlier
      //                << " inlier ratio: " << inlier_ratio << std::endl;
      return;
    }

    std::cerr << "MultiLoopDetectorHBST::compute|detected loop closure: [source ID: "
              << local_map_current_->graphId()
              << "] -> [target ID: " << local_map_reference_->graphId()
              << "] inliers: " << number_of_inliers << " (ratio: " << inlier_ratio
              << ", avg chi: " << average_chi_per_inlier << ")" << std::endl;
    const EstimateType pose_in_target = estimate_.inverse() * pose_in_current_;

    // ds reduce weight along z (we mainly want to correct on x and y) TODO make selectable
    // ds this assumes local maps generated for a forward facing camera
    LoopInformationMatrixType information_matrix(LoopInformationMatrixType::Identity());
    information_matrix(EstimateType::Dim - 1, EstimateType::Dim - 1) = 1e-3;

    // ds assemble loop closure object
    std::shared_ptr<LoopClosureType> closure(
      new LoopClosureType(-1, // ds is set later by validator (I guess?),
                          local_map_current_,
                          local_map_reference_ /*target*/,
                          estimate_,
                          information_matrix,
                          pose_in_target,
                          average_chi_per_inlier,
                          number_of_inliers,
                          number_of_correspondences,
                          std::move(correspondences_)));
    ThisType::_detected_closures.push_back(closure);
  }

} // namespace srrg2_slam_interfaces
