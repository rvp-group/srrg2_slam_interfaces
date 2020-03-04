#pragma once
#include "aligner.h"
#include "multi_aligner_slice.h"
#include "multi_aligner_slice_prior.h"
#include "srrg_config/property_configurable_vector.h"
#include <srrg_solver/solver_core/factor_graph.h>
#include <srrg_solver/variables_and_factors/types_2d/variable_se2_ad.h>
#include <srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h>

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;
  using namespace srrg2_solver;

  //! @brief base class for each multi-aligner. it does everything but still you can inherit from
  //!        this class if you need something very specific (see matchables)
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
    using MultiAlignerSliceProcessorType    = MultiAlignerSliceBase_<BaseVariableType>;
    using MultiAlignerSliceProcessorPtrType = std::shared_ptr<MultiAlignerSliceProcessorType>;

    //! @brief all parameters are here
  public:
    PARAM_VECTOR(PropertyConfigurableVector_<MultiAlignerSliceProcessorType>,
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

    //! @brief public methods
  public:
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

    //! @brief sets the fixed scene, isolating the channels
    void setFixed(PropertyContainerBase* fixed_scene_) override;

    //! @brief sets the moving scene, isolating the channels
    void setMoving(PropertyContainerBase* moving_scene_) override;

    //! @brief sets the estimate to the module, propagating it to the solver
    //!        (overrides the solver's variable) - NO INVERSION OR WHATEVER
    void setEstimate(const EstimateType& est_) override;

    //! @brief returns the current estimate, retrieving it from the solver - NO INVERSION OR
    //!        WHATEVER
    const EstimateType& estimate() const override;

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

    // srrg TODO remove this
    int numCorrespondences() override;

    // ds propagate platform to all internal users as well
    void setPlatform(PlatformPtr platform_) override;

    // ds visualize current pose-to-measurements correspondences in perspective
    void draw(srrg2_core::ViewerCanvasPtr canvas_) const override;

  protected:
    //! @brief auxiliary function: removes correspondences that involve outliers (post registration)
    //!        to enhance posterior phases like merging
    void pruneCorrespondences();
    //! @brief auxiliary function: swap out and bind inlier-only robustifiers to each slice
    void setClampRobustifiers();
    //! @brief auxiliary function: swap back the original robustifiers (after inliers only run)
    void restoreRobustifiers();
    //! @brief clears the graph and rebuilds it from scratch interrogating all the slices
    void setupAligner();

    //! @brief calls solver compute number_of_iterations_ times or until
    //!        termination criterion is triggered
    // ia I THINK THAT THIS SHIT IS SICK
    //    BECAUSE WE USE ALWAYS THE SAME ESTIMATE TO COMPUTE THE CORRESPONDENCES
    void runSolver(const size_t& number_of_iterations_,
                   const std::shared_ptr<AlignerTerminationCriteriaBase> termination_criterion_);

    // ia auxiliary function that computes the associations in each slice from the current guess
    inline bool computeAssociationsSlices(const EstimateType& guess_) {
      bool association_good = false;

      // ia do the dance for each slice
      const size_t number_of_slices = param_slice_processors.size();
      for (size_t index_slice = 0; index_slice < number_of_slices; ++index_slice) {
        MultiAlignerSliceProcessorPtrType slice = param_slice_processors.value(index_slice);
        slice->setEstimate(guess_);
        slice->computeAssociation();
        association_good |= slice->associationGood();
      }
      return association_good;
    }

    //! @brief auxiliary function to perform pre/post computation
    virtual void preCompute();
    virtual void postCompute();

  protected:
    //! @brief auxiliary attributes
    FactorGraphPtr _graph;
    std::vector<RobustifierBasePtr> _robustifiers_original_per_slice;
    bool _slices_changed_flag = true;
  };

  using MultiAligner2D    = MultiAlignerBase_<VariableSE2RightAD>;
  using MultiAligner3D    = MultiAlignerBase_<VariableSE3EulerRightAD>;
  using MultiAligner2DPtr = std::shared_ptr<MultiAligner2D>;
  using MultiAligner3DPtr = std::shared_ptr<MultiAligner3D>;

  // ds TODO converge 3D pipelines to use this aligner type
  using MultiAligner3DQR    = MultiAlignerBase_<VariableSE3QuaternionRightAD>;
  using MultiAligner3DQRPtr = std::shared_ptr<MultiAligner3DQR>;

} // namespace srrg2_slam_interfaces
