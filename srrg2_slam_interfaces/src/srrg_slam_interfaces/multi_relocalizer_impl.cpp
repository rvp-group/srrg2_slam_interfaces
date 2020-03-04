#include "multi_relocalizer.h"

#define DEBUG(var) \
  if (var)         \
  std::cerr

static bool relocalizer_debug = true;

namespace srrg2_slam_interfaces {

  template <typename SlamAlgorithmType_>
  void MultiRelocalizer_<SlamAlgorithmType_>::compute() {
    ThisType::_relocalized_closure = nullptr;
    ThisType::_pose_in_local_map   = EstimateType::Identity();
    ThisType::_relocalize_map      = nullptr;
    auto tracker                   = ThisType::_slam->param_tracker.value();
    if (!tracker) {
      throw std::runtime_error("MultiRelocalizer::compute|no tracker set");
    }

    auto aligner = param_aligner.value();

    //    std::cerr << FG_YELLOW("MultiRelocalizer| attempting relocalization on ");
    //    std::cerr << ThisType::_closure_candidates->size() << " local maps" << std::endl;

    if (!aligner) {
      std::cerr << FG_YELLOW(
                     "MultiRelocalizer::compute|WARNING: no aligner set, computing best closure "
                     "based purely on detector statistics")
                << std::endl;
      std::shared_ptr<LoopClosureType> best_closure = nullptr;
      for (auto c : *(ThisType::_closure_candidates)) {
        if (c->pose_in_target.translation().norm() < ThisType::param_max_translation.value() &&
            (!best_closure || (c->num_correspondences > best_closure->num_correspondences &&
                               c->chi_inliers < best_closure->chi_inliers))) {
          best_closure = c;
        }
      }
      // we set the as new map,  the best match close enough
      if (best_closure) {
        ThisType::_relocalized_closure = best_closure;
        ThisType::_relocalize_map      = best_closure->target();
        ThisType::_pose_in_local_map   = best_closure->pose_in_target;
      }

    } else {
      DEBUG(relocalizer_debug) << "MultiRelocalizer_::compute|search: [\n";

      aligner->setFixed(&tracker->measurementScene());
      float best_chi_average = 1e10;

      for (auto c : *(ThisType::_closure_candidates)) {
        LocalMapType* target_local_map = c->target();
        DEBUG(relocalizer_debug) << "(" << target_local_map->graphId();
        if (c->pose_in_target.translation().norm() > ThisType::param_max_translation.value()) {
          DEBUG(relocalizer_debug) << ", " << FG_BRED("MAX_TRANSITION DROP: ")
                                   << c->pose_in_target.translation().norm() << ")\n";
          continue;
        }

        aligner->setMoving(&target_local_map->dynamic_properties);
        aligner->setEstimate(c->pose_in_target.inverse());
        aligner->compute();
        if (aligner->status() != AlignerBase::Success) {
          DEBUG(relocalizer_debug)
            << ", " << FG_BRED("ALIGNER DROP [code: " << aligner->status() << "]") << ")\n";
          continue;
        }
        const IterationStats& istat = aligner->iterationStats().back();

        int num_correspondences = aligner->numCorrespondences();
        int num_inliers         = istat.num_inliers;
        float chi_inliers       = istat.chi_inliers / num_inliers;
        DEBUG(relocalizer_debug) << ", " << num_correspondences;
        DEBUG(relocalizer_debug) << ", " << num_inliers;
        DEBUG(relocalizer_debug) << ", " << chi_inliers;

        // srrg this should be done in the slice
        if (num_inliers < param_relocalize_min_inliers.value()) {
          DEBUG(relocalizer_debug)
            << ", " << FG_BRED("NUM_CHI_INLIERS DROP: ") << num_inliers << ")\n";
          continue;
        }

        if (chi_inliers > param_relocalize_max_chi_inliers.value()) {
          DEBUG(relocalizer_debug)
            << ", " << FG_BRED("MAX_CHI_INLIERS DROP: ") << chi_inliers << ")\n";
          continue;
        }

        float min_inliers_ratio = (float) num_inliers / (float) num_correspondences;
        DEBUG(relocalizer_debug) << ", " << min_inliers_ratio;
        if (min_inliers_ratio < param_relocalize_min_inliers_ratio.value()) {
          DEBUG(relocalizer_debug)
            << ", " << FG_BRED("MIN_INLIERS_RATIO DROP: ") << min_inliers_ratio << ")\n";
          continue;
        }

        DEBUG(relocalizer_debug) << ", " << FG_BGREEN("ACCEPT") << ")\n";
        else {
          std::cerr << FG_BGREEN("ACCEPT") << std::endl;
        }
        const float curr_average = istat.chi_inliers / istat.num_inliers;

        if (curr_average < best_chi_average) {
          ThisType::_relocalize_map      = target_local_map;
          ThisType::_pose_in_local_map   = aligner->estimate().inverse();
          best_chi_average               = curr_average;
          ThisType::_relocalized_closure = c;

          // ds store associations for merging (can still be overwritten)
          aligner->storeCorrespondences();
        }
      }
      DEBUG(relocalizer_debug) << "]" << std::endl;
    }
    if (ThisType::_relocalize_map) {
      std::cerr << FG_GREEN(" TARGET: ") << ThisType::_relocalize_map->graphId() << std::endl;
      //          std::cerr << FG_YELLOW("LaserSLAM2D::RELOCALIZED") << std::endl;
    }
  }
} // namespace srrg2_slam_interfaces

#undef DEBUG
