#include "aligner_termination_criteria_standard.h"

namespace srrg2_slam_interfaces {

  template <typename AlignerType_>
  void AlignerTerminationCriteriaStandard_<AlignerType_>::init(AlignerBase* aligner_) {
    AlignerTerminationCriteriaBase::init(aligner_);
    _typed_aligner = dynamic_cast<AlignerType_*>(this->_aligner);
    assert(_typed_aligner && ("aligner not compatible with termination criteria"));
    _num_iteration = 0;
    int window     = param_window_size.value();
    _num_correspondences_anal.reset(window);
    _num_inliers_anal.reset(window);
    _num_outliers_anal.reset(window);
    _chi_norm_anal.reset(window);
  }

  template <typename AlignerType_>
  bool AlignerTerminationCriteriaStandard_<AlignerType_>::hasToStop() {
    ++_num_iteration;
    const srrg2_solver::IterationStats& current_stats = _typed_aligner->iterationStats().back();
    const float epsilon                               = param_chi_epsilon.value();

    int ncorr = _typed_aligner->numCorrespondences();
    int ninl  = current_stats.num_inliers;
    int nout  = current_stats.num_outliers;
    float chi = current_stats.chi_inliers / ninl;
    if (!ninl)
      return false;

    _num_correspondences_anal.addSample(ncorr);
    _num_inliers_anal.addSample(ninl);
    _num_outliers_anal.addSample(nout);
    _chi_norm_anal.addSample(chi);

    int window = param_window_size.value();
    if (_num_correspondences_anal.numSamples() < window) {
      return false;
    }
    if (_num_correspondences_anal.range() > param_num_correspondences_range.value()) {
      return false;
    }
    if (_num_inliers_anal.range() > param_num_inliers_range.value()) {
      return false;
    }

    if (_num_outliers_anal.range() > param_num_outliers_range.value()) {
      return false;
    }
    // cerr << "o: " << chi
    //      << ":" <<_chi_norm_anal.min()
    //      << ":" << _chi_norm_anal.max()
    //      << ":" << _chi_norm_anal.range()/_chi_norm_anal.max() <<  " " << endl;
    if (_chi_norm_anal.range() / _chi_norm_anal.max() > epsilon) {
      return false;
    }

    return true;
  }

} // namespace srrg2_slam_interfaces
