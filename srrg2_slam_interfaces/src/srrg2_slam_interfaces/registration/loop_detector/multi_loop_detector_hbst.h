#pragma once
#include "multi_loop_detector_brute_force.h" //ds reusing parameters
#include <srrg_hbst/types/binary_tree.hpp>
#include <srrg_solver/solver_core/factor.h>

namespace srrg2_slam_interfaces {

  /**
   * @brief Appearance-based loop detector (for place recognition)
   */
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
    using SliceProcessorType    = AlignerSliceProcessorBase_<BaseVariableType>;
    using SliceProcessorTypePtr = std::shared_ptr<SliceProcessorType>;

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

    PARAM(srrg2_core::PropertyFloat,
          maximum_descriptor_distance,
          "maximum permitted descriptor distance for a match",
          25.0f,
          nullptr);
    PARAM(srrg2_core::PropertyUnsignedInt,
          maximum_leaf_size,
          "maximum size of a leaf (i.e. number of descriptors) required before splitting",
          100,
          &_config_changed);
    PARAM(srrg2_core::PropertyFloat,
          maximum_partitioning,
          "maximum partitioning tolerance (i.e. 0.0 would be a perfect split)",
          0.1,
          &_config_changed);
    PARAM(srrg2_core::PropertyUnsignedInt,
          maximum_depth,
          "maximum permitted tree depth (maximum = descriptor bit size)",
          16,
          &_config_changed);
    PARAM(srrg2_core::PropertyFloat,
          maximum_distance_for_merge,
          "maximum descriptor distance permitted for internal merging (HBST)",
          0,
          &_config_changed);
    PARAM(srrg2_core::PropertyUnsignedInt,
          minimum_age_difference_to_candidates,
          "minimum required age difference between query and database reference",
          0,
          &_config_changed);

    virtual ~MultiLoopDetectorHBST_() {
      _database.clear(true);
    }

    /**
     * @brief sets the current local map (not used in autonomy)
     * @param[in] current local map
     */
    void setCurrentLocalMap(LocalMapType* local_map_) {
      _current_local_map = local_map_;
    }

    //! @brief calls {computeCorrespondences, computeAlignments}
    virtual void compute() override;

    //! @brief updates database to graph id mapping for last processed local map
    virtual void addPreviousQuery() override;

    //! @brief helper function that is intended also for external use if no alignment is desired
    //! this method populates the _correspondences_per_reference and _indices buffers
    virtual void computeCorrespondences();

    /**
     * @brief returns correspondences between query and reference at index
     * @param[in] index of the reference
     * @return correspondences computed at reference index
     */
    CorrespondenceVector correspondences(const size_t& index_) const {
      return _correspondences_per_reference.at(index_);
    }

    //! @brief returns indices for all references whose number of matches satisfy:
    //! - > relocalize_min_inliers
    //! - > relocalize_min_inliers_ratio
    //! @return vector of indices
    const std::vector<size_t>& indices() const {
      return _indices;
    }

  protected:
    //! @brief computes the aligment between all active closure candidates
    void _computeAlignments();
    /**
     * @brief compute the correspondences indices for give matches
     * @param[in] matches_: vector of matches
     * @param[out] correspondences: vector of correspondences
     */
    void _computeCorrespondencesFromMatches(const DatabaseType::MatchVector& matches_,
                                            CorrespondenceVector& correspondences_) const;

    /**
     * @brief slice getter from aligner
     * @param[out] the slice needed
     */
    void _retrieveSliceFromAligner(SliceProcessorTypePtr& slice_);

    /**
     * @brief getter for descriptors in local map
     * @param[in] local_map: target local map
     * @oaram[in] slice_name: choose the slice you want to check in given local map
     * @param[out] descriptors: set of descriptors for points in local map @ slice_name
     */
    void _retrieveDescriptorsFromLocalMap(LocalMapType* local_map_,
                                          const std::string& slice_point_cloud_name_,
                                          const PointDescriptorVectorType*& descriptors_);

    /**
     * @brief adds a new loop closure to the poll
     * @param[in] pose_in_current_: pose of the robot in the current local map
     * @param[in] measurement_: offset between source and target local maps
     * @param[in] solver_statistics_: statistics of the solver
     * @param[in] local_map_current_: source local map for closure
     * @param[in] local_map_reference_: target local map for closure
     * @param[in] correspondences_: correspondences for closure
     */
    void _addLoopClosure(const EstimateType& pose_in_current_,
                         const EstimateType& measurement_,
                         const IterationStats& solver_statistics_,
                         LocalMapType* local_map_current_,
                         LocalMapType* local_map_reference_,
                         const CorrespondenceVector& correspondences_);

    bool _config_changed = true; /**< config change control flag*/

    LocalMapType* _current_local_map =
      nullptr; /**<currently set local map (overwritten if picked automatically from SLAM system)*/

    DatabaseType::MatchableVector _query_matchables; /**< converted query matchables from previous
                                                      * query, buffered for delayed addition
                                                      */

    DatabaseType _database; /**< the HBST database containing descriptors arranged in a binary tree
    since the descriptor bit size has to be determined at compile we lock it for 256 bits ftm*/
    std::unordered_map<size_t, size_t>
      _graph_id_to_database_index; /**< mapping from local map graph identifiers to database indices
                                    * (monotonically increasing) this is required since we might
                                    * merge/relocalize local maps without adding them to the graph
                                    */

    std::vector<LocalMapType*> _local_maps_in_database; /**<local maps referenced in the database
                                                           (monotonically increasing)*/

    std::vector<size_t>
      _indices; /**<match vector buffer - only valid for last query!!! TODO eventually purge*/
    std::unordered_map<size_t, CorrespondenceVector>
      _correspondences_per_reference; /**< correspondences stored per reference index*/

    const PointDescriptorVectorType* _fixed_current_local_map =
      nullptr; /**<instantiated fixed local map points*/
    const PointDescriptorVectorType* _moving_past_local_map =
      nullptr; /**<instantiated moving local map points*/
  };

} // namespace srrg2_slam_interfaces
