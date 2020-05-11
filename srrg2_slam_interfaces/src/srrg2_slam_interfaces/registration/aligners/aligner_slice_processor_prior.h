#pragma once
#include "aligner_slice_processor_base.h"

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;
  using namespace srrg2_solver;

  /**
   * @brief aligner slice processor prior class
   * Inserts in the graph a prior measurement (e.g. odometry, IMU)
   */
  template <typename FactorType_, typename FixedType_, typename MovingType_>
  class AlignerSliceProcessorPrior_
    : public AlignerSliceProcessorBase_<
        typename VariableTypeAt_<typename FactorType_::VariableTupleType,
                                 0>::VariableType::BaseVariableType> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using FactorType    = FactorType_;
    using FactorTypePtr = std::shared_ptr<FactorType>;
    using VariableType =
      typename VariableTypeAt_<typename FactorType::VariableTupleType, 0>::VariableType;
    using BaseVariableType = typename VariableType::BaseVariableType;
    using BaseType         = AlignerSliceProcessorBase_<BaseVariableType>;
    using EstimateType     = typename VariableType::EstimateType;

    using FixedType  = FixedType_;
    using MovingType = MovingType_;

    friend class FactorBase;

    template <typename T>
    friend class MultiAlignerBase_;

    AlignerSliceProcessorPrior_() : _factor(new FactorType) {
    }
    virtual ~AlignerSliceProcessorPrior_() {
    }

  protected:
    void initializeFactor() override {
      _factor.reset(new FactorType);
      if (_fixed_slice) {
        bindFixed();
      }
      if (_moving_slice) {
        bindMoving();
      }
    }
    // propagates the transform to the inner modules
    void setMovingInFixed(const EstimateType& est_) override {
    }

    FactorBasePtr factor() override;

    virtual CorrespondenceVector* computeCorrespondences() override {
      setupFactor();
      return nullptr;
    }

    virtual void storeCorrespondences() override {
      // ds TODO implement or make THIS derive from the aligner slice instead of the base?
    }

    virtual bool correspondencesGood() const override {
      return true;
    }

    /**
     * @brief performs basic checks for data integrity
     * @param[in] message: name of the class
     */
    void sanityCheck(const std::string& message = "AlignerSliceProcessorPrior_");

    int numCorrespondences() const override {
      return 1;
    }

    virtual bool isPrior() override {
      return true;
    }

    // initializes the factor based on the values of _fixed_slice and _moving_slice
    virtual void setupFactor() = 0;
    virtual void bindRobustifier() {
      _factor->setRobustifier(BaseType::param_robustifier.value().get());
    }
    virtual void bindFixed() override;
    virtual void bindMoving() override;

    FixedType* _fixed_slice =
      nullptr; /**< casted fixed slice (e.g. in tracker case, sensor specific measurement) */
    MovingType* _moving_slice =
      nullptr; /**< casted moving slice (e.g. in tracker case, local map points) */

    int _count            = 0;       /**< some kind of count */
    FactorTypePtr _factor = nullptr; /**< produced factor */
  };

} // namespace srrg2_slam_interfaces
