// Copyright 2010-2018 Google LLC
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// @file
#ifndef OR_TOOLS_CONSTRAINT_SOLVER_ROUTING_INDEX_MANAGER_H_
#define OR_TOOLS_CONSTRAINT_SOLVER_ROUTING_INDEX_MANAGER_H_

#include <tuple>
#include <vector>

#include "ortools/base/int_type_indexed_vector.h"
#include "ortools/base/logging.h"
#include "ortools/constraint_solver/routing_types.h"

namespace operations_research {

/// @brief Manager for any NodeIndex <-> variable index conversion.
/// @details The routing solver uses variable indices internally and through
/// its API.
/// These variable indices are tricky to manage directly because one Node can
/// correspond to a multitude of variables, depending on the number of times
/// they appear in the model, and if they're used as start and/or end points.
/// This class aims to simplify variable index usage, allowing users to use
/// NodeIndex instead.
///
/// Usage:
/// @code
///   auto starts_ends = ...;  // These are NodeIndex.
///   RoutingIndexManager manager(/*nodes*/10, /*vehicles*/4, starts_ends);
///   RoutingModel model(manager);
/// @endcode
/// Then, use 'manager.NodeToIndex(node)' whenever 'model' requires a variable
/// index.
class RoutingIndexManager {
 public:
  typedef RoutingNodeIndex NodeIndex;
  static const int64 kUnassigned;

  /// @brief Creates a NodeIndex to variable index mapping.
  /// @param num_nodes Number of nodes in the problem.
  /// @param num_vehicles Number of vehicles in the problem.
  /// @param depot @ref GetStartIndex "start" and @ref GetEndIndex "end" NodeIndex for all vehicles.
  RoutingIndexManager(int num_nodes, int num_vehicles, NodeIndex depot);
  /// @brief Creates a NodeIndex to variable index mapping.
  /// @param num_nodes Number of nodes in the problem.
  /// @param num_vehicles Number of vehicles in the problem.
  /// @param starts Array containing the start NodeIndex for each vehicle.
  /// @param ends Array containing the end NodeIndex for each vehicle.
  /// @note @b starts and @b ends arrays must have @b exactly @ref num_vehicles elements.
  RoutingIndexManager(int num_nodes, int num_vehicles,
                      const std::vector<NodeIndex>& starts,
                      const std::vector<NodeIndex>& ends);
  /// @brief Creates a NodeIndex to variable index mapping.
  /// @param num_nodes Number of nodes in the problem.
  /// @param num_vehicles Number of vehicles in the problem.
  /// @param starts_ends Array containing a pair [start,end] NodeIndex for each vehicle.
  /// @note @b starts_ends arrays must have @b exactly @ref num_vehicles elements.
  RoutingIndexManager(
      int num_nodes, int num_vehicles,
      const std::vector<std::pair<NodeIndex, NodeIndex> >& starts_ends);

  ~RoutingIndexManager() {}

  int num_nodes() const { return num_nodes_; }
  int num_vehicles() const { return num_vehicles_; }
  int num_indices() const { return index_to_node_.size(); }
  int64 GetStartIndex(int vehicle) const { return vehicle_to_start_[vehicle]; }
  int64 GetEndIndex(int vehicle) const { return vehicle_to_end_[vehicle]; }
  /// Gets the corresponding index from a NodeIndex.
  /// @param node the requested node index.
  /// @return The associated variable index.
  int64 NodeToIndex(NodeIndex node) const {
    DCHECK_GE(node.value(), 0);
    DCHECK_LT(node.value(), node_to_index_.size());
    return node_to_index_[node];
  }
  std::vector<int64> NodesToIndices(const std::vector<NodeIndex>& nodes) const;
  /// Gets the corresponding NodeIndex from a varaible index.
  /// @param index the requested variable index.
  /// @return The associated NodeIndex.
  NodeIndex IndexToNode(int64 index) const {
    DCHECK_GE(index, 0);
    DCHECK_LT(index, index_to_node_.size());
    return index_to_node_[index];
  }
  // TODO(user): Remove when removal of NodeIndex from RoutingModel is
  // complete.
  int num_unique_depots() const { return num_unique_depots_; }
  std::vector<NodeIndex> GetIndexToNodeMap() const { return index_to_node_; }
  gtl::ITIVector<NodeIndex, int64> GetNodeToIndexMap() const {
    return node_to_index_;
  }

 private:
  void Initialize(
      int num_nodes, int num_vehicles,
      const std::vector<std::pair<NodeIndex, NodeIndex> >& starts_ends);

  std::vector<NodeIndex> index_to_node_;
  gtl::ITIVector<NodeIndex, int64> node_to_index_;
  std::vector<int64> vehicle_to_start_;
  std::vector<int64> vehicle_to_end_;
  int num_nodes_;
  int num_vehicles_;
  int num_unique_depots_;
};

}  // namespace operations_research

#endif  // OR_TOOLS_CONSTRAINT_SOLVER_ROUTING_INDEX_MANAGER_H_
