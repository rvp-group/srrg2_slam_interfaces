#pragma once
#include "aligner.h"
#include "aligner_slice_processor_base.h"
#include "srrg_config/property_configurable_vector.h"
#include <srrg_solver/solver_core/factor_graph.h>
#include <srrg_solver/variables_and_factors/types_2d/variable_se2_ad.h>
#include <srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h>

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;
  using namespace srrg2_solver;

  /**
   * @brief base class for a multi-aligner. It registers a moving on a fixed for each slice and
   * optimizes the registration graph. Built on top of a VariableType (usually SE2/SE3 Variable) and
   * stores multi-cue measurements as PropertyContainers. They will be demuxed by the slices
   * themselves.
   */
  template <typename VariableType_>
  class MultiAlignerBase_ : public Aligner_<typename VariableType_::EstimateType,
                                            PropertyContainerBase,
                                            PropertyContainerBase> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! @brief usings
    using VariableType     = VariableType_;
    using BaseVariableType = typename VariableType::BaseVariableType;
    using EstimateType     = typename VariableType::EstimateType;
    using BaseType         = Aligner_<EstimateType, PropertyContainerBase, PropertyContainerBase>;
    using AlignerSliceProcessorType    = AlignerSliceProcessorBase_<BaseVariableType>;
    using AlignerSliceProcessorTypePtr = std::shared_ptr<AlignerSliceProcessorType>;

    PARAM_VECTOR(PropertyConfigurableVector_<AlignerSliceProcessorType>,
                 slice_processors,
                 "slices",
                 &(this->_slices_changed_flag));

    PARAM(PropertyConfigurable_<Solver>,
          solver,
          "this solver",
          std::shared_ptr<Solver>(new Solver),
          nullptr);

    PARAM(PropertyInt, min_num_inliers, "minimum number ofinliers", 10, nullptr);

    PARAM(PropertyBool,
          enable_inlier_only_runs,
          "toggles additional inlier only runs if sufficient inliers are available",
          false,
          nullptr);

    PARAM(PropertyBool,
          keep_only_inlier_correspondences,
          "toggles removal of correspondences which factors are not inliers in the last iteration",
          false,
          nullptr);

    //! @brief object life - ctor
    MultiAlignerBase_() : _graph(new FactorGraph) {
      param_solver->param_max_iterations.value().clear();
      param_solver->param_max_iterations.pushBack(1);
      VariableBasePtr variable(new VariableType);
      variable->setGraphId(0);
      _graph->addVariable(variable);
    }

    //! @brief object life - dtor
    virtual ~MultiAlignerBase_() {
    }

    /**
     * @brief sets the fixed scene and provides it to each slice processor
     * @param[in] fixed_scene_: property container holding the fixed data slice
     */
    void setFixed(PropertyContainerBase* fixed_scene_) override;

    /**
     * @brief sets the fixed scene and provides it to each slice processor
     * @param[in] fixed_scene_: property container holding the fixed data slice
     */
    void setMoving(PropertyContainerBase* moving_scene_) override;

    void setMovingInFixed(const EstimateType& moving_in_fixed_) override;

    const EstimateType& movingInFixed() const override;

    //! @brief exposed compute, pretty articulated. it calls pre/post computes and then perfoms the
    //!        registration. this is achieved ICP style:
    //!        - eventually apply robustifiers (for each slide) -> preCompute()
    //!        - compute correspondences for each slide and construct the graph
    //!        - call solver
    //!        - eventually do something else like inliers only registration -> postCompute()
    //!        in here, we call: preCompute() -> runSolver() -> postCompute();
    void compute() override;

    //! @brief stores associations in an auxiliary data buffer that can be re-used during merge
    void storeCorrespondences() override;

    int numCorrespondences() override;

    //! @brief propagate platform to all internal users as well
    void setPlatform(PlatformPtr platform_) override;

    //! @brief visualize current pose-to-measurements correspondences in perspective
    void draw(srrg2_core::ViewerCanvasPtr canvas_) const override;

  protected:
    //! @brief auxiliary function: removes correspondences that involve outliers (post registration)
    //!        to enhance posterior phases like merging
    void _pruneCorrespondences();
    //! @brief auxiliary function: swap out and bind inlier-only robustifiers to each slice
    void _setClampRobustifiers();
    //! @brief auxiliary function: swap back the original robustifiers (after inliers only run)
    void _restoreRobustifiers();
    //! @brief clears the graph and rebuilds it from scratch interrogating all the slices
    void _setupAligner();

    //! @brief calls solver compute number_of_iterations_ times or until
    //!        termination criterion is triggered
    void _runSolver(const size_t& number_of_iterations_,
                    const std::shared_ptr<AlignerTerminationCriteriaBase> termination_criterion_);

    //! @brief auxiliary function that computes the associations in each slice from the current
    //! guess
    inline bool _computeCorrespondencesPerSlices(const EstimateType& moving_in_fixed_) {
      bool association_good = false;

      // ia do the dance for each slice
      const size_t number_of_slices = param_slice_processors.size();
      for (size_t index_slice = 0; index_slice < number_of_slices; ++index_slice) {
        AlignerSliceProcessorTypePtr slice = param_slice_processors.value(index_slice);
        slice->setMovingInFixed(moving_in_fixed_);
        slice->computeCorrespondences();
        association_good |= slice->correspondencesGood();
      }
      return association_good;
    }

    //! @brief auxiliary function to perform pre computation
    virtual void _preCompute();
    //! @brief auxiliary function to perform post computation
    virtual void _postCompute();

  protected:
    //! @brief auxiliary attributes
    FactorGraphPtr _graph     = nullptr; /**< registration graph */
    bool _slices_changed_flag = true;    /**< control flag for changed slices */
    std::vector<RobustifierBasePtr> _robustifiers_original_per_slice; /**< vector of robustifiers */
  };

  using MultiAligner2D    = MultiAlignerBase_<VariableSE2RightAD>;
  using MultiAligner3D    = MultiAlignerBase_<VariableSE3EulerRightAD>;
  using MultiAligner2DPtr = std::shared_ptr<MultiAligner2D>;
  using MultiAligner3DPtr = std::shared_ptr<MultiAligner3D>;

  // ds TODO converge 3D pipelines to use this aligner type
  using MultiAligner3DQR    = MultiAlignerBase_<VariableSE3QuaternionRightAD>;
  using MultiAligner3DQRPtr = std::shared_ptr<MultiAligner3DQR>;

} // namespace srrg2_slam_interfaces
