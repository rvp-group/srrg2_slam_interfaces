#include "aligner.h"

namespace srrg2_slam_interfaces {

  void AlignerTerminationCriteriaBase::init(AlignerBase* aligner_) {
    _aligner = aligner_;
  }

} // namespace srrg2_slam_interfaces
