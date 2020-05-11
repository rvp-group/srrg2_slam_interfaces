#pragma once
#include "aligner_termination_criteria.h"
#include <srrg_data_structures/platform.h>
#include <srrg_messages/messages/base_sensor_message.h>
#include <srrg_solver/solver_core/solver_stats.h>
#include <srrg_viewer/drawable_base.h>

namespace srrg2_slam_interfaces {

  /**
   * @brief minimal interface to Aligners
   */
  class AlignerBase : public srrg2_core::Configurable,
                      public srrg2_core::PlatformUser,
                      public srrg2_core::DrawableBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    friend class AlignerTerminationCriteriaBase;

    /**
     * @brief possible aligner status
     */
    enum Status {
      Success                  = 0 /**< the two entities are successfully aligned */,
      NotEnoughCorrespondences = 1 /**< correspondences where not enough to estimate the motion */,
      NotEnoughInliers         = 2 /**< the measurements giver where full of outliers */,
      Fail                     = 3 /**< other kind of failure */
    };

    PARAM(srrg2_core::PropertyInt, max_iterations, "maximum number of iterations", 10, 0);
    PARAM(srrg2_core::PropertyConfigurable_<AlignerTerminationCriteriaBase>,
          termination_criteria,
          "termination criteria, not set=max iterations",
          0,
          0);

    //! @brief default ctor
    AlignerBase() = default;

    //! @brief default dtor
    virtual ~AlignerBase() = default;

    /**
     * @brief compute the alignment that brings the moving entity on the fixed one
     */
    virtual void compute() = 0;
    /**
     * @brief aligner status getter
     * @return one of the possible status depending on the output status
     */
    Status status() const {
      return _status;
    }

  protected:
    Status _status = Fail;
  };

  /**
   * @brief minimal typed interface to Aligners
   * FixedType and MovingType can be anything
   */

  template <typename EstimateType_, typename FixedType_, typename MovingType_>
  class Aligner_ : public AlignerBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using EstimateType = EstimateType_;
    using MovingType   = MovingType_;
    using FixedType    = FixedType_;

    Aligner_() = default;

    virtual ~Aligner_() = default;

    /**
     * @brief set the fixed measurement
     * E.g. in the case of tracking this is the measurement from the sensor
     *      in the case of relocalization, this is the other graph's local map
     * toggle the fixed modification flag
     * @param[in] fixed_: fixed measurement
     */
    virtual inline void setFixed(FixedType* fixed_) {
      _fixed              = fixed_;
      _fixed_changed_flag = true;
    }

    /**
     * @brief set the moving measurement
     * E.g. in the case of tracking this is the current clipped local map
     *      in the case of relocalization, this is the current graph's local map/current measurement
     * toggle the fixed modification flag
     * @param[in] moving_: moving measurement
     */
    virtual inline void setMoving(MovingType* moving_) {
      _moving              = moving_;
      _moving_changed_flag = true;
    }

    /**
     * @brief set moving in fixed initial guess
     * E.g. in the case of tracking this is the inverted odometry motion
     */
    virtual void setMovingInFixed(const EstimateType& moving_in_fixed_) = 0;
    /**
     * @brief estimated moving in fixed
     * @return the estimated transform
     */
    virtual const EstimateType& movingInFixed() const = 0;

    /**
     * @brief solver statistics getter
     * @return a vector of statistics
     */
    const srrg2_solver::IterationStatsVector& iterationStats() const {
      return _iteration_stats;
    }

    //! @brief stores associations in an auxiliary data buffer that can be re-used during merge
    virtual void storeCorrespondences() {
      // ds no effect if not overridden by target aligner TODO enforce implementation?
    }
    /**
     * @brief numbers of correspondences getter
     * @return number of correspondences computed in the last iteration step
     */
    virtual int numCorrespondences() = 0;

  protected:
    MovingType* _moving = 0; /**< moving measurement*/
    FixedType* _fixed   = 0; /**< fixed measurement*/

    // srrg control variables
    bool _moving_changed_flag = true; /**< moving control variable (true if _moving reset)*/
    bool _fixed_changed_flag  = true; /**< fixed control variable (true if _fixed reset)*/

    srrg2_solver::IterationStatsVector
      _iteration_stats; /**< whole statistic chunk for every iteration of the solver. It stacks in a
                           single vector all the info of the solver since per compute() call */
  };

} // namespace srrg2_slam_interfaces
