#pragma once
#include "multi_loop_detector_brute_force.h" //ds reusing parameters
#include <srrg_hbst/types/binary_tree.hpp>
#include <srrg_solver/solver_core/factor.h>

namespace srrg2_slam_interfaces {

  template <typename SLAMAlgorithmType_, typename AlignerType_, typename FactorType_>
  class MultiLoopDetectorHBST_
    : public MultiLoopDetectorBruteForce_<SLAMAlgorithmType_, AlignerType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using LoopClosureType  = typename SLAMAlgorithmType_::LoopClosureType;
    using LocalMapType     = typename SLAMAlgorithmType_::LocalMapType;
    using EstimateType     = typename LocalMapType::EstimateType;
    using AlignerType      = AlignerType_;
    using VariableType     = typename AlignerType::VariableType;
    using BaseVariableType = typename VariableType::BaseVariableType;
    using FactorBaseType   = Factor_<BaseVariableType>;
    using SolverType       = srrg2_solver::Solver;
    using ThisType         = MultiLoopDetectorHBST_<SLAMAlgorithmType_, AlignerType_, FactorType_>;
    using SliceType        = MultiAlignerSliceBase_<BaseVariableType>;
    using SliceTypePtr     = std::shared_ptr<SliceType>;

    //! supported descriptor types (if cast to these types fails, detection fails)
    using PointDescriptorType       = PointIntensityDescriptor_<EstimateType::Dim, float>;
    using PointDescriptorVectorType = PointIntensityDescriptorVectorCloud<EstimateType::Dim, float>;
    using DescriptorPropertyType    = Property_<PointDescriptorVectorType*>;

    //! required for directly calling the generic solver TODO infer from somewhere
    using FactorType = FactorType_;
    using FactorCorrespondenceDrivenType =
      FactorCorrespondenceDriven_<FactorType, PointDescriptorVectorType, PointDescriptorVectorType>;
    using InformationMatrixType = typename FactorType::InformationMatrixType;

    //! information matrix type for loop closure (does not have to correspond with factor's)
    using LoopInformationMatrixType = typename LoopClosureType::InformationMatrixType;

    //! selected database type (currently set at compile time)
    using DatabaseType = srrg_hbst::BinaryTree256<uint64_t>;

    PARAM(PropertyFloat,
          maximum_descriptor_distance,
          "maximum permitted descriptor distance for a match",
          25.0f,
          nullptr);
    PARAM(PropertyUnsignedInt,
          maximum_leaf_size,
          "maximum size of a leaf (i.e. number of descriptors) required before splitting",
          100,
          &_config_changed);
    PARAM(PropertyFloat,
          maximum_partitioning,
          "maximum partitioning tolerance (i.e. 0.0 would be a perfect split)",
          0.1,
          &_config_changed);
    PARAM(PropertyUnsignedInt,
          maximum_depth,
          "maximum permitted tree depth (maximum = descriptor bit size)",
          16,
          &_config_changed);
    PARAM(PropertyFloat,
          maximum_distance_for_merge,
          "maximum descriptor distance permitted for internal merging (HBST)",
          0,
          &_config_changed);
    PARAM(PropertyUnsignedInt,
          minimum_age_difference_to_candidates,
          "minimum required age difference between query and database reference",
          0,
          &_config_changed);

    virtual ~MultiLoopDetectorHBST_() {
      _database.clear(true);
    }

    //! sets the current local map (not used in autonomy)
    void setCurrentLocalMap(LocalMapType* local_map_) {
      _current_local_map = local_map_;
    }

    //! calls {computeCorrespondences, computeAlignments}
    virtual void compute() override;

    //! updates database to graph id mapping for last processed local map
    virtual void addPreviousQuery() override;

    //! helper function that is intended also for external use if no alignment is desired
    //! this method populates the _correspondences_per_reference and _indices buffers
    virtual void computeCorrespondences();

    //! returns correspondences between query and reference at index
    CorrespondenceVector correspondences(const size_t& index_) const {
      return _correspondences_per_reference.at(index_);
    }

    //! returns indices for all references whose number of matches satisfy:
    //! - > relocalize_min_inliers
    //! - > relocalize_min_inliers_ratio
    const std::vector<size_t>& indices() const {
      return _indices;
    }

  protected:
    //! computes the aligment between all active closure candidates
    void _computeAlignments();

    void _computeCorrespondencesFromMatches(const DatabaseType::MatchVector& matches_,
                                            CorrespondenceVector& correspondences_) const;

    void _retrieveSliceFromAligner(SliceTypePtr& slice_);

    void _retrieveDescriptorsFromLocalMap(LocalMapType* local_map_,
                                          const std::string& slice_point_cloud_name_,
                                          const PointDescriptorVectorType*& descriptors_);

    void _addLoopClosure(const EstimateType& pose_in_current_,
                         const EstimateType& estimate_,
                         const IterationStats& solver_statistics_,
                         LocalMapType* local_map_current_,
                         LocalMapType* local_map_reference_,
                         CorrespondenceVector& correspondences_);

    bool _config_changed = true;

    //! currently set local map (overwritten if picked automatically from SLAM system)
    LocalMapType* _current_local_map = nullptr;

    //! converted query matchables from previous query, buffered for delayed addition
    DatabaseType::MatchableVector _query_matchables;

    //! the HBST database containing descriptors arranged in a binary tree
    //! since the descriptor bit size has to be determined at compile we lock it for 256 bits ftm
    DatabaseType _database;

    //! mapping from local map graph identifiers to database indices (monotonically increasing)
    //! this is required since we might merge/relocalize local maps without adding them to the graph
    std::unordered_map<size_t, size_t> _graph_id_to_database_index;

    // ds local maps referenced in the database (monotonically increasing)
    std::vector<LocalMapType*> _local_maps_in_database;

    //! match vector buffer - only valid for last query!!! TODO eventually purge
    std::vector<size_t> _indices;
    std::unordered_map<size_t, CorrespondenceVector> _correspondences_per_reference;

    //! instantiated local map points
    const PointDescriptorVectorType* _fixed_current_local_map;
    const PointDescriptorVectorType* _moving_past_local_map;
  };

} // namespace srrg2_slam_interfaces
