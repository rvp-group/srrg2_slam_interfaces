#pragma once
#include "local_map_splitting_criterion_base.hpp"

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;
  using namespace srrg2_solver;

  //! note that the visibility based splitting criterion is coupled with the target number of tracks
  //! rule of thumb: # tracks >= param_minimum_tracked_point_ratio*param_maximum_number_of_points
  template <typename SlamAlgorithmType_>
  class LocalMapSplittingCriterionVisbility_
    : public LocalMapSplittingCriterionBase_<SlamAlgorithmType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using LocalMapType       = typename SlamAlgorithmType_::LocalMapType;
    using TrackerPtrType     = std::shared_ptr<typename SlamAlgorithmType_::TrackerType>;
    using RelocalizerPtrType = std::shared_ptr<typename SlamAlgorithmType_::RelocalizerType>;
    using ThisType           = LocalMapSplittingCriterionVisbility_<SlamAlgorithmType_>;
    PARAM(PropertyFloat,
          maximum_number_of_points,
          "maximum number of points to be contained in the current local map",
          1000,
          nullptr);
    PARAM(PropertyFloat,
          minimum_tracked_point_ratio,
          "minimum portion of points of the current local map that has to be tracked",
          0.1,
          nullptr);
    virtual ~LocalMapSplittingCriterionVisbility_() {
    }

    virtual void compute() override {
      ThisType::_has_to_split = false;
      if (!ThisType::_slam) {
        throw std::runtime_error("LocalMapSplittingCriterionVisbility|SLAM algorithm not set");
      }
      TrackerPtrType tracker = ThisType::_slam->param_tracker.value();
      if (!tracker) {
        throw std::runtime_error("LocalMapSplittingCriterionVisbility|SLAM tracker not set");
      }

      // ds cache local map and check if not available
      const LocalMapType* local_map = ThisType::_slam->currentLocalMap();
      if (!local_map || local_map->properties().empty()) {
        std::cerr << "LocalMapSplittingCriterionVisbility|WARNING: no local map available, no "
                     "split decision will be made"
                  << std::endl;
        _previous_number_of_points = 0;
        return;
      }
      const size_t current_number_of_points = local_map->numberOfPoints();

      // ds if the previous number of points was not reset (not in a new local map)
      if (_previous_number_of_points != 0) {
        assert(current_number_of_points >= _previous_number_of_points);
        const size_t delta = current_number_of_points - _previous_number_of_points;

        // ds if the current number of points is above the limit
        // ds the delta is 0 for pure relocalization and hence will not trigger this condition
        if (delta != 0 && current_number_of_points > param_maximum_number_of_points.value()) {
          ThisType::_has_to_split    = true;
          _previous_number_of_points = 0;
          return;
        }
      }
      _previous_number_of_points = current_number_of_points;

      // ds evaluate iteration statistics to determine inlier tracks for a split decision
      IterationStatsVector iteration_statistics(0);

      // ds if the system just relocalized
      if (ThisType::_slam->relocalized()) {
        RelocalizerPtrType relocalizer = ThisType::_slam->param_relocalizer.value();
        if (relocalizer && relocalizer->param_aligner.value()) {
          // ds compute inlier ratio from relocalizer's aligner iteration statistics
          // ds the statistics are identical to the tracker one's in case they share the aligner
          iteration_statistics = relocalizer->param_aligner->iterationStats();
        }
      } else {
        // ds compute inlier ratio from tracker's aligner iteration statistics
        iteration_statistics = tracker->iterationStats();
      }
      if (iteration_statistics.empty()) {
        std::cerr << "LocalMapSplittingCriterionVisbility|WARNING: no iteration stats available, "
                     "no split decision will be made"
                  << std::endl;
        return;
      }
      const float inlier_tracking_ratio =
        static_cast<float>(iteration_statistics.back().num_inliers) / current_number_of_points;

      // ds if the tracking ratio is too low we have to generate a new local map
      if (inlier_tracking_ratio < param_minimum_tracked_point_ratio.value()) {
        ThisType::_has_to_split    = true;
        _previous_number_of_points = 0;
      }
    }

  protected:
    //! heuristic used to determine premature splitting (without interaction)
    size_t _previous_number_of_points = 0;
  };

} // namespace srrg2_slam_interfaces
