#pragma once
#include "correspondence_finder.h"
#include <srrg_config/property_configurable.h>
#include <srrg_messages/messages/base_sensor_message.h>
#include <srrg_solver/solver_core/solver_stats.h>
#include <srrg_viewer/drawable_base.h>

namespace srrg2_slam_interfaces {
  class AlignerBase;
  class AlignerTerminationCriteriaBase;

  // termination criteria for an aligner
  // to be specialized in the remaining classes
  class AlignerTerminationCriteriaBase : public srrg2_core::Configurable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    friend class AlignerBase;
    virtual void init(AlignerBase* aligner_);
    virtual bool hasToStop() = 0;

  protected:
    AlignerBase* _aligner;
  };

  class AlignerBase : public srrg2_core::Configurable,
                      public srrg2_core::PlatformUser,
                      public srrg2_core::DrawableBase {
    friend class AlignerTerminationCriteriaBase;

  public:
    //! @brief default ctor
    AlignerBase() {
      // ia nothin to do
    }

    //! @brief default dtor
    virtual ~AlignerBase() {
      // ia nothin to do
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PARAM(srrg2_core::PropertyInt, max_iterations, "maximum number of iterations", 10, 0);
    PARAM(srrg2_core::PropertyConfigurable_<AlignerTerminationCriteriaBase>,
          termination_criteria,
          "termination criteria, not set=max iterations",
          0,
          0);

    enum Status { Success = 0, NotEnoughCorrespondences = 1, NotEnoughInliers = 2, Fail = 3 };

    virtual void compute() = 0;
    Status status() const {
      return _status;
    }

  protected:
    Status _status = Fail;
  };

  // Mother of aligners (typed)
  // FixedType and MovingType can be anything
  template <typename TransformType_, typename FixedType_, typename MovingType_>
  class Aligner_ : public AlignerBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using TransformType = TransformType_;
    using MovingType    = MovingType_;
    using FixedType     = FixedType_;

    //! @brief default ctor
    Aligner_() {
    }

    //! @brief default dtor
    virtual ~Aligner_() {
    }

    virtual inline void setFixed(FixedType* fixed_) {
      _fixed              = fixed_;
      _fixed_changed_flag = true;
    }
    virtual inline void setMoving(MovingType* moving_) {
      _moving              = moving_;
      _moving_changed_flag = true;
    }
    virtual void setEstimate(const TransformType_& transform) = 0;
    virtual const TransformType& estimate() const             = 0;
    const srrg2_solver::IterationStatsVector& iterationStats() const {
      return _iteration_stats;
    }

    //! stores associations in an auxiliary data buffer that can be re-used during merge
    virtual void storeCorrespondences() {
      // ds no effect if not overridden by target aligner TODO enforce implementation?
    }
    virtual int numCorrespondences() = 0;

  protected:
    MovingType* _moving       = 0;
    bool _moving_changed_flag = true;
    FixedType* _fixed         = 0;
    bool _fixed_changed_flag  = true;
    srrg2_solver::IterationStatsVector _iteration_stats;
  };

} // namespace srrg2_slam_interfaces
