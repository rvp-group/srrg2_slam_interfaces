#include "multi_aligner.h"

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;
  using namespace srrg2_solver;

  template <typename VariableType_>
  void MultiAlignerBase_<VariableType_>::setupAligner() {
    // tg clear factor graph
    _graph->clear();
    // tg instanciate a variable which represent the aligner estimate
    // and add it to the graph
    VariableBasePtr variable(new VariableType);
    variable->setGraphId(0);
    _graph->addVariable(variable);
    // tg add a factor for each slice in the aligner
    const size_t& number_of_slices = param_slice_processors.size();
    for (size_t sit = 0; sit < number_of_slices; ++sit) {
      MultiAlignerSliceProcessorPtrType s = param_slice_processors.value(sit);
      s->initializeFactor();
      _graph->addFactor(s->factor());
    }
    _slices_changed_flag = false;
  }

  template <typename VariableType_>
  void MultiAlignerBase_<VariableType_>::setFixed(PropertyContainerBase* fixed_scene_) {
    BaseType::setFixed(fixed_scene_);
    for (size_t sit = 0; sit < param_slice_processors.size(); ++sit) {
      MultiAlignerSliceProcessorPtrType s = param_slice_processors.value(sit);
      s->setFixed(fixed_scene_);
    }
  }

  // sets the moving scene, isolating the channels
  template <typename VariableType_>
  void MultiAlignerBase_<VariableType_>::setMoving(PropertyContainerBase* moving_scene_) {
    BaseType::setMoving(moving_scene_);
    for (size_t sit = 0; sit < param_slice_processors.size(); ++sit) {
      MultiAlignerSliceProcessorPtrType s = param_slice_processors.value(sit);
      s->setMoving(moving_scene_);
    }
  }

  template <typename VariableType_>
  void MultiAlignerBase_<VariableType_>::setEstimate(
    const typename MultiAlignerBase_<VariableType_>::EstimateType& est_) {
    if (!param_solver.value()) {
      throw std::runtime_error("MultiAlignerBase_::setEstimate()| no solver");
    }
    VariableType* variable = dynamic_cast<VariableType*>(_graph->variable(0));
    variable->setEstimate(est_);
  }

  template <typename VariableType_>
  const typename MultiAlignerBase_<VariableType_>::EstimateType&
  MultiAlignerBase_<VariableType_>::estimate() const {
    if (!param_solver.value()) {
      throw std::runtime_error("MultiAlignerBase_::estimate()| no solver");
    }
    const VariableType* variable = dynamic_cast<const VariableType*>(_graph->variable(0));
    return variable->estimate();
  }

  template <typename VariableType_>
  void MultiAlignerBase_<VariableType_>::setClampRobustifiers() {
    // ds swap out and bind inlier-only robustifiers to each slice TODO refactor this
    _robustifiers_original_per_slice.reserve(param_slice_processors.size());
    for (size_t sit = 0; sit < param_slice_processors.size(); ++sit) {
      MultiAlignerSliceProcessorPtrType slice = param_slice_processors.value(sit);
      RobustifierBasePtr robustifier_current(slice->param_robustifier.value());
      _robustifiers_original_per_slice.emplace_back(robustifier_current);

      // ds only replace valid robustifiers
      if (robustifier_current) {
        RobustifierBasePtr robustifier_clamp(RobustifierBasePtr(new RobustifierClamp()));
        robustifier_clamp->param_chi_threshold.setValue(
          robustifier_current->param_chi_threshold.value());
        slice->param_robustifier.setValue(robustifier_clamp);
        slice->bindRobustifier();
      }
    }
  }

  template <typename VariableType_>
  void MultiAlignerBase_<VariableType_>::restoreRobustifiers() {
    // ds swap back the original robustifiers
    for (size_t sit = 0; sit < param_slice_processors.size(); ++sit) {
      MultiAlignerSliceProcessorPtrType slice = param_slice_processors.value(sit);
      slice->param_robustifier.setValue(_robustifiers_original_per_slice[sit]);
      slice->bindRobustifier();
    }
  }

  template <typename VariableType_>
  void MultiAlignerBase_<VariableType_>::preCompute() {
    const size_t& number_of_slices = param_slice_processors.size();
    // ds bind robustifiers to each slice and perform slicewise aligner initializations
    // ds odometry priors (e.g. motion model) are only effective as last slice(s)
    // ds TODO add a proper policy for this
    for (size_t sit = 0; sit < number_of_slices; ++sit) {
      MultiAlignerSliceProcessorPtrType s = param_slice_processors.value(sit);
      s->bindRobustifier();
      s->init(this);
    }
  }

  template <typename VariableType_>
  void MultiAlignerBase_<VariableType_>::compute() {
    if (!param_solver.value()) {
      throw std::runtime_error("MultiAlignerBase_::compute()| no solver");
    }

    std::shared_ptr<AlignerTerminationCriteriaBase> term_crit =
      this->param_termination_criteria.value();

    if (term_crit) {
      term_crit->init(this);
    }
    this->_iteration_stats.clear();
    param_solver->clearIterationStats();

    // tg setup the factor graph for the aligner, each slide is a factor
    if (_slices_changed_flag) {
      setupAligner();
    }
    // tg force the solver to recompute structure
    param_solver->setGraph(_graph);

    // ia precompute: in this case just bind robustifiers in each aligner-slice
    preCompute();

    // ds run solver on current problem
    runSolver(this->param_max_iterations.value(), term_crit);

    // ds if no iteration was performed
    if (this->_iteration_stats.empty()) {
      this->_status = AlignerBase::Fail;
      return;
    }

    // ds if we have insufficient inliers -> return altered status
    const IterationStats& stats = this->_iteration_stats.back();
    if (stats.num_inliers < param_min_num_inliers.value()) {
      this->_status = AlignerBase::NotEnoughInliers;
      return;
    }

    // ia post compute: in this case run inliers only and eventually purge correspondences
    postCompute();

    // ds make sure the solver's final isometry estimate is healthy and propagate it to the slices
    EstimateType transform_estimate = estimate();
    fixTransform(transform_estimate);
    setEstimate(transform_estimate);
    this->_status = AlignerBase::Success;
  }

  template <typename VariableType_>
  void MultiAlignerBase_<VariableType_>::postCompute() {
    // ds if inlier only runs are enabled (at this point we should have enough inliers)
    if (param_enable_inlier_only_runs.value()) {
      // ds swap out and bind inlier-only robustifiers to each slice TODO refactor this
      setClampRobustifiers();

      // ds run inlier only iterations
      runSolver(this->param_max_iterations.value(), this->param_termination_criteria.value());

      // ds swap back the original robustifiers
      restoreRobustifiers();
    }

    // ds purge correspondences if desired (based on last iteration)
    if (param_keep_only_inlier_correspondences.value()) {
      pruneCorrespondences();
    }
  }

  template <typename VariableType_>
  void MultiAlignerBase_<VariableType_>::pruneCorrespondences() {
    const FactorStatsVector& factor_stats(param_solver->measurementStats());

    // ds check for inconsistency: correspondences concatenation must have equal length as factors
    size_t total_number_of_correspondences = 0;
    for (size_t sit = 0; sit < param_slice_processors.size(); ++sit) {
      MultiAlignerSliceProcessorPtrType s = param_slice_processors.value(sit);
      total_number_of_correspondences += s->numCorrespondences();
    }
    if (factor_stats.size() != total_number_of_correspondences) {
      // ds unable to prune invalid correspondences
      std::cerr << "MultiAlignerBase::runSolver|WARNING: size of factor stats = "
                << factor_stats.size()
                << " mismatches total size of correspondences = " << total_number_of_correspondences
                << std::endl;
      std::cerr << "MultiAlignerBase::runSolver|WARNING: correspondences will not be pruned"
                << std::endl;
      return;
    }

    // ds purge correspondences for each slice (in order!)
    // ds since we have a reference of the correspondences we can directly modify them
    size_t total_number_of_inlier_correspondences = 0;
    size_t index_factor                           = 0;
    for (size_t sit = 0; sit < param_slice_processors.size(); ++sit) {
      MultiAlignerSliceProcessorPtrType s = param_slice_processors.value(sit);
      if (s->numCorrespondences()) {
        CorrespondenceVector& correspondences = s->correspondences();
        size_t index_inlier                   = 0;
        for (size_t index = 0; index < correspondences.size(); ++index) {
          if (factor_stats[index_factor].status == FactorStats::Status::Inlier) {
            correspondences[index_inlier] = std::move(correspondences[index]);
            ++index_inlier;
          }
          ++index_factor;
        }
        correspondences.resize(index_inlier);
        total_number_of_inlier_correspondences += index_inlier;
      }
    }
    assert(index_factor == factor_stats.size());
    assert(index_factor == total_number_of_correspondences);
    //      std::cerr << "MultiAlignerBase::runSolver|inlier correspondences: "
    //                << total_number_of_inlier_correspondences << "/" <<
    //                total_number_of_correspondences
    //                << std::endl;
  }

  template <typename VariableType_>
  void MultiAlignerBase_<VariableType_>::runSolver(
    const size_t& number_of_iterations_,
    const std::shared_ptr<AlignerTerminationCriteriaBase> termination_criterion_) {
    // ds backup in case of failure
    const EstimateType& initial_guess = estimate();
    EstimateType updated_estimate     = initial_guess;
    // ds run iterations
    for (size_t i = 0; i < number_of_iterations_; ++i) {
      // ds correspondence buffer for factor statistics based pruning (e.g. keep inliers only)
      bool association_good = computeAssociationsSlices(updated_estimate);
      if (!association_good) {
        this->_status = AlignerBase::NotEnoughCorrespondences;
        setEstimate(updated_estimate);
        break;
      }
      param_solver->compute();
      this->_iteration_stats.insert(this->_iteration_stats.end(),
                                    param_solver->iterationStats().begin(),
                                    param_solver->iterationStats().end());

      if (param_solver->status() == SolverBase::Success) {
        updated_estimate = estimate();
        this->setEstimate(updated_estimate);
      }

      // ds check for early termination
      if (termination_criterion_ && termination_criterion_->hasToStop()) {
        break;
      }
    }
  }

  template <typename VariableType_>
  void MultiAlignerBase_<VariableType_>::storeCorrespondences() {
    for (size_t i = 0; i < param_slice_processors.size(); ++i) {
      MultiAlignerSliceProcessorPtrType slice = param_slice_processors.value(i);
      assert(slice);
      slice->storeCorrespondences();
    }
  }

  // sets the moving scene, isolating the channels
  template <typename VariableType_>
  int MultiAlignerBase_<VariableType_>::numCorrespondences() {
    int num_correspondences = 0;
    for (size_t sit = 0; sit < param_slice_processors.size(); ++sit) {
      MultiAlignerSliceProcessorPtrType s = param_slice_processors.value(sit);
      int c                               = s->numCorrespondences();
      if (c >= 0) {
        num_correspondences += c;
      }
    }
    return num_correspondences;
  }

  template <typename VariableType_>
  void MultiAlignerBase_<VariableType_>::setPlatform(PlatformPtr platform_) {
    BaseType::setPlatform(platform_);
    for (size_t i = 0; i < param_slice_processors.size(); ++i) {
      MultiAlignerSliceProcessorPtrType slice = param_slice_processors.value(i);
      assert(slice);
      slice->setPlatform(platform_);
    }
  }

  template <typename VariableType_>
  void MultiAlignerBase_<VariableType_>::draw(srrg2_core::ViewerCanvasPtr canvas_) const {
    for (size_t i = 0; i < param_slice_processors.size(); ++i) {
      MultiAlignerSliceProcessorPtrType slice = param_slice_processors.value(i);
      slice->draw(canvas_);
    }
  }

} // namespace srrg2_slam_interfaces
