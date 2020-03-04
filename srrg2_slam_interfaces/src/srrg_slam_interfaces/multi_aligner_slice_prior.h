#pragma once
#include "multi_aligner_slice.h"

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;
  using namespace srrg2_solver;

  template <typename FactorType_, typename FixedType_, typename MovingType_>
  class MultiAlignerSlicePrior_
    : public MultiAlignerSliceBase_<
        typename VariableTypeAt_<typename FactorType_::VariableTupleType,
                                 0>::VariableType::BaseVariableType> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using FactorType    = FactorType_;
    using FactorTypePtr = std::shared_ptr<FactorType>;
    using VariableType =
      typename VariableTypeAt_<typename FactorType_::VariableTupleType, 0>::VariableType;
    using BaseVariableType = typename VariableType::BaseVariableType;
    using EstimateType     = typename VariableType::EstimateType;

    using FixedType  = FixedType_;
    using MovingType = MovingType_;

    friend class FactorBase;

    template <typename T>
    friend class MultiAlignerBase_;

    MultiAlignerSlicePrior_() : _factor(new FactorType) {
    }
    virtual ~MultiAlignerSlicePrior_() {
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
    void setEstimate(const EstimateType& est_) override {
    }

    FactorBasePtr factor() override {
      _factor->setVariableId(0, 0);
      return _factor;
    }

    virtual CorrespondenceVector* computeAssociation() override {
      setupFactor();
      return nullptr;
    }

    virtual void storeCorrespondences() override {
      // ds TODO implement or make THIS derive from the aligner slice instead of the base?
    }

    virtual bool associationGood() const override {
      return true;
    }

    void sanityCheck(const std::string& message = "MultiAlignerSlicePrior_");

    int numCorrespondences() const override {
      return 0;
    }

    FixedType* _fixed_slice   = nullptr;
    MovingType* _moving_slice = nullptr;
    int _count                = 0;
    FactorTypePtr _factor     = nullptr;

    // initializes the factor based on the values of _fixed_slice and _moving_slice
    virtual void setupFactor() = 0;
    virtual void bindRobustifier() {
      _factor->setRobustifier(this->param_robustifier.value().get());
    }
    virtual void bindFixed() override;
    virtual void bindMoving() override;
  };

} // namespace srrg2_slam_interfaces
