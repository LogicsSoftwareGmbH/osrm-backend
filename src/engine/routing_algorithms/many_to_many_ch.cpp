#include "engine/routing_algorithms/many_to_many.hpp"
#include "engine/routing_algorithms/routing_base_ch.hpp"

#include <boost/assert.hpp>
#include <ranges>

#include <vector>

namespace osrm::engine::routing_algorithms
{

namespace ch
{

inline bool addLoopWeight(const DataFacade<ch::Algorithm> &facade,
                          const NodeID node,
                          EdgeWeight &weight,
                          EdgeDuration &duration,
                          EdgeDistance &distance,
                          EdgeDistance &urban)
{ // Special case for CH when contractor creates a loop edge node->node
    BOOST_ASSERT(weight < EdgeWeight{0});

    const auto loop_weight = ch::getLoopMetric<EdgeWeight>(facade, node);
    if (std::get<0>(loop_weight) != INVALID_EDGE_WEIGHT)
    {
        const auto new_weight_with_loop = weight + std::get<0>(loop_weight);
        if (new_weight_with_loop >= EdgeWeight{0})
        {
            weight = new_weight_with_loop;
            auto result = ch::getLoopMetric<EdgeDuration>(facade, node);
            duration += std::get<0>(result);
            distance += std::get<1>(result);
            urban += std::get<2>(result);
            return true;
        }
    }

    // No loop found or adjusted weight is negative
    return false;
}

// Variants of insertSourceInHeap/insertTargetInHeap (routing_base.hpp) that
// additionally seed the phantom's urban_meters fraction. Exact, not an
// approximation: a phantom sits on a single edge-based node whose classes are
// uniform along the segment, so its urban share is ratio * distance fraction.
template <typename ManyToManyQueryHeap>
void insertSourceInHeapWithUrbanSeed(const DataFacade<Algorithm> &facade,
                                     ManyToManyQueryHeap &heap,
                                     const PhantomNodeCandidates &source_candidates)
{
    const bool has_urban = facade.HasUrbanData();
    const auto urban_seed = [&](const SegmentID &segment, const EdgeDistance distance)
    {
        if (!has_urban)
        {
            return EdgeDistance{0};
        }
        const auto ratio = facade.GetUrbanRatio(facade.GetClassData(segment.id));
        return to_alias<EdgeDistance>(ratio * from_alias<float>(distance));
    };

    for (const auto &phantom_node : source_candidates)
    {
        if (phantom_node.IsValidForwardSource())
        {
            heap.Insert(phantom_node.forward_segment_id.id,
                        EdgeWeight{0} - phantom_node.GetForwardWeightPlusOffset(),
                        {phantom_node.forward_segment_id.id,
                         EdgeDuration{0} - phantom_node.GetForwardDuration(),
                         EdgeDistance{0} - phantom_node.GetForwardDistance(),
                         EdgeDistance{0} - urban_seed(phantom_node.forward_segment_id,
                                                      phantom_node.GetForwardDistance())});
        }
        if (phantom_node.IsValidReverseSource())
        {
            heap.Insert(phantom_node.reverse_segment_id.id,
                        EdgeWeight{0} - phantom_node.GetReverseWeightPlusOffset(),
                        {phantom_node.reverse_segment_id.id,
                         EdgeDuration{0} - phantom_node.GetReverseDuration(),
                         EdgeDistance{0} - phantom_node.GetReverseDistance(),
                         EdgeDistance{0} - urban_seed(phantom_node.reverse_segment_id,
                                                      phantom_node.GetReverseDistance())});
        }
    }
}

template <typename ManyToManyQueryHeap>
void insertTargetInHeapWithUrbanSeed(const DataFacade<Algorithm> &facade,
                                     ManyToManyQueryHeap &heap,
                                     const PhantomNodeCandidates &target_candidates)
{
    const bool has_urban = facade.HasUrbanData();
    const auto urban_seed = [&](const SegmentID &segment, const EdgeDistance distance)
    {
        if (!has_urban)
        {
            return EdgeDistance{0};
        }
        const auto ratio = facade.GetUrbanRatio(facade.GetClassData(segment.id));
        return to_alias<EdgeDistance>(ratio * from_alias<float>(distance));
    };

    for (const auto &phantom_node : target_candidates)
    {
        if (phantom_node.IsValidForwardTarget())
        {
            heap.Insert(phantom_node.forward_segment_id.id,
                        phantom_node.GetForwardWeightPlusOffset(),
                        {phantom_node.forward_segment_id.id,
                         phantom_node.GetForwardDuration(),
                         phantom_node.GetForwardDistance(),
                         urban_seed(phantom_node.forward_segment_id,
                                    phantom_node.GetForwardDistance())});
        }
        if (phantom_node.IsValidReverseTarget())
        {
            heap.Insert(phantom_node.reverse_segment_id.id,
                        phantom_node.GetReverseWeightPlusOffset(),
                        {phantom_node.reverse_segment_id.id,
                         phantom_node.GetReverseDuration(),
                         phantom_node.GetReverseDistance(),
                         urban_seed(phantom_node.reverse_segment_id,
                                    phantom_node.GetReverseDistance())});
        }
    }
}

template <bool DIRECTION>
void relaxOutgoingEdges(
    const DataFacade<Algorithm> &facade,
    const typename SearchEngineData<Algorithm>::ManyToManyQueryHeap::HeapNode &heapNode,
    typename SearchEngineData<Algorithm>::ManyToManyQueryHeap &query_heap,
    const PhantomNodeCandidates &)
{
    if (stallAtNode<DIRECTION>(facade, heapNode, query_heap))
    {
        return;
    }

    const bool has_urban = facade.HasUrbanData();

    for (auto edge : facade.GetAdjacentEdgeRange(heapNode.node))
    {
        const auto &data = facade.GetEdgeData(edge);
        if (DIRECTION == FORWARD_DIRECTION ? data.forward : data.backward)
        {
            const NodeID to = facade.GetTarget(edge);
            const auto edge_weight = data.weight;

            const auto edge_duration = data.duration;
            const auto edge_distance = data.distance;
            const auto edge_urban = has_urban ? facade.GetUrbanMeters(edge) : EdgeDistance{0};

            BOOST_ASSERT_MSG(edge_weight > EdgeWeight{0}, "edge_weight invalid");
            const auto to_weight = heapNode.weight + edge_weight;
            const auto to_duration = heapNode.data.duration + to_alias<EdgeDuration>(edge_duration);
            const auto to_distance = heapNode.data.distance + edge_distance;
            const auto to_urban = heapNode.data.urban + edge_urban;

            const auto toHeapNode = query_heap.GetHeapNodeIfWasInserted(to);
            // New Node discovered -> Add to Heap + Node Info Storage
            if (!toHeapNode)
            {
                query_heap.Insert(
                    to, to_weight, {heapNode.node, to_duration, to_distance, to_urban});
            }
            // Found a shorter Path -> Update weight and set new parent
            else if (std::tie(to_weight, to_duration) <
                     std::tie(toHeapNode->weight, toHeapNode->data.duration))
            {
                toHeapNode->data = {heapNode.node, to_duration, to_distance, to_urban};
                toHeapNode->weight = to_weight;
                query_heap.DecreaseKey(*toHeapNode);
            }
        }
    }
}

void forwardRoutingStep(const DataFacade<Algorithm> &facade,
                        const std::size_t row_index,
                        const std::size_t number_of_targets,
                        typename SearchEngineData<Algorithm>::ManyToManyQueryHeap &query_heap,
                        const std::vector<NodeBucket> &search_space_with_buckets,
                        std::vector<EdgeWeight> &weights_table,
                        std::vector<EdgeDuration> &durations_table,
                        std::vector<EdgeDistance> &distances_table,
                        std::vector<EdgeDistance> &urban_table,
                        std::vector<NodeID> &middle_nodes_table,
                        const PhantomNodeCandidates &candidates)
{
    // Take a copy of the extracted node because otherwise could be modified later if toHeapNode is
    // the same
    const auto heapNode = query_heap.DeleteMinGetHeapNode();

    // Check if each encountered node has an entry
    const auto &bucket_list = std::equal_range(search_space_with_buckets.begin(),
                                               search_space_with_buckets.end(),
                                               heapNode.node,
                                               NodeBucket::Compare());
    for (const auto &current_bucket : std::ranges::subrange(bucket_list.first, bucket_list.second))
    {
        // Get target id from bucket entry
        const auto column_index = current_bucket.column_index;
        const auto target_weight = current_bucket.weight;
        const auto target_duration = current_bucket.duration;
        const auto target_distance = current_bucket.distance;
        const auto target_urban = current_bucket.urban;

        auto &current_weight = weights_table[row_index * number_of_targets + column_index];

        EdgeDistance nulldistance = {0};
        EdgeDistance nullurban = {0};

        auto &current_duration = durations_table[row_index * number_of_targets + column_index];
        auto &current_distance =
            distances_table.empty() ? nulldistance
                                    : distances_table[row_index * number_of_targets + column_index];
        auto &current_urban = urban_table.empty()
                                  ? nullurban
                                  : urban_table[row_index * number_of_targets + column_index];

        // Check if new weight is better
        auto new_weight = heapNode.weight + target_weight;
        auto new_duration = heapNode.data.duration + target_duration;
        auto new_distance = heapNode.data.distance + target_distance;
        auto new_urban = heapNode.data.urban + target_urban;

        if (new_weight < EdgeWeight{0})
        {
            if (addLoopWeight(
                    facade, heapNode.node, new_weight, new_duration, new_distance, new_urban))
            {
                current_weight = std::min(current_weight, new_weight);
                current_duration = std::min(current_duration, new_duration);
                current_distance = std::min(current_distance, new_distance);
                current_urban = std::min(current_urban, new_urban);
                middle_nodes_table[row_index * number_of_targets + column_index] = heapNode.node;
            }
        }
        else if (std::tie(new_weight, new_duration) < std::tie(current_weight, current_duration))
        {
            current_weight = new_weight;
            current_duration = new_duration;
            current_distance = new_distance;
            current_urban = new_urban;
            middle_nodes_table[row_index * number_of_targets + column_index] = heapNode.node;
        }
    }

    relaxOutgoingEdges<FORWARD_DIRECTION>(facade, heapNode, query_heap, candidates);
}

void backwardRoutingStep(const DataFacade<Algorithm> &facade,
                         const unsigned column_index,
                         typename SearchEngineData<Algorithm>::ManyToManyQueryHeap &query_heap,
                         std::vector<NodeBucket> &search_space_with_buckets,
                         const PhantomNodeCandidates &candidates)
{
    // Take a copy (no ref &) of the extracted node because otherwise could be modified later if
    // toHeapNode is the same
    const auto heapNode = query_heap.DeleteMinGetHeapNode();

    // Store settled nodes in search space bucket
    search_space_with_buckets.emplace_back(heapNode.node,
                                           heapNode.data.parent,
                                           column_index,
                                           heapNode.weight,
                                           heapNode.data.duration,
                                           heapNode.data.distance,
                                           heapNode.data.urban);

    relaxOutgoingEdges<REVERSE_DIRECTION>(facade, heapNode, query_heap, candidates);
}

} // namespace ch

template <>
std::pair<std::vector<EdgeDuration>, std::vector<EdgeDistance>>
manyToManySearch(SearchEngineData<ch::Algorithm> &engine_working_data,
                 const DataFacade<ch::Algorithm> &facade,
                 const std::vector<PhantomNodeCandidates> &candidates_list,
                 const std::vector<std::size_t> &source_indices,
                 const std::vector<std::size_t> &target_indices,
                 const bool calculate_distance,
                 std::vector<EdgeDistance> *urban_meters_table)
{
    const auto number_of_sources = source_indices.size();
    const auto number_of_targets = target_indices.size();
    const auto number_of_entries = number_of_sources * number_of_targets;

    const bool calculate_urban = urban_meters_table != nullptr && facade.HasUrbanData();

    std::vector<EdgeWeight> weights_table(number_of_entries, INVALID_EDGE_WEIGHT);
    std::vector<EdgeDuration> durations_table(number_of_entries, MAXIMAL_EDGE_DURATION);
    std::vector<EdgeDistance> distances_table(calculate_distance ? number_of_entries : 0,
                                              MAXIMAL_EDGE_DISTANCE);
    std::vector<EdgeDistance> urban_table(calculate_urban ? number_of_entries : 0,
                                          MAXIMAL_EDGE_DISTANCE);
    std::vector<NodeID> middle_nodes_table(number_of_entries, SPECIAL_NODEID);

    std::vector<NodeBucket> search_space_with_buckets;

    // Populate buckets with paths from all accessible nodes to destinations via backward searches
    for (std::uint32_t column_index = 0; column_index < target_indices.size(); ++column_index)
    {
        const auto index = target_indices[column_index];
        const auto &target_candidates = candidates_list[index];

        engine_working_data.InitializeOrClearManyToManyThreadLocalStorage(
            facade.GetNumberOfNodes());
        auto &query_heap = *(engine_working_data.many_to_many_heap);
        ch::insertTargetInHeapWithUrbanSeed(facade, query_heap, target_candidates);

        // Explore search space
        while (!query_heap.Empty())
        {
            backwardRoutingStep(
                facade, column_index, query_heap, search_space_with_buckets, target_candidates);
        }
    }

    // Order lookup buckets
    std::sort(search_space_with_buckets.begin(), search_space_with_buckets.end());

    // Find shortest paths from sources to all accessible nodes
    for (std::uint32_t row_index = 0; row_index < source_indices.size(); ++row_index)
    {
        const auto source_index = source_indices[row_index];
        const auto &source_candidates = candidates_list[source_index];

        // Clear heap and insert source nodes
        engine_working_data.InitializeOrClearManyToManyThreadLocalStorage(
            facade.GetNumberOfNodes());
        auto &query_heap = *(engine_working_data.many_to_many_heap);
        ch::insertSourceInHeapWithUrbanSeed(facade, query_heap, source_candidates);

        // Explore search space
        while (!query_heap.Empty())
        {
            forwardRoutingStep(facade,
                               row_index,
                               number_of_targets,
                               query_heap,
                               search_space_with_buckets,
                               weights_table,
                               durations_table,
                               distances_table,
                               urban_table,
                               middle_nodes_table,
                               source_candidates);
        }
    }

    if (urban_meters_table != nullptr)
    {
        *urban_meters_table = std::move(urban_table);
    }

    return std::make_pair(std::move(durations_table), std::move(distances_table));
}

} // namespace osrm::engine::routing_algorithms
