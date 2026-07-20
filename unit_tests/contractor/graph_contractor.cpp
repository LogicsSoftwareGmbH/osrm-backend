#include "contractor/graph_contractor.hpp"

#include "contractor/contractor_graph.hpp"
#include "contractor/graph_contractor_adaptors.hpp"
#include "extractor/edge_based_edge.hpp"
#include "helper.hpp"

#include <boost/test/unit_test.hpp>

#include <tbb/global_control.h>
#include <tuple>

using namespace osrm;
using namespace osrm::contractor;
using namespace osrm::unit_test;

namespace
{
// like makeGraph, but with a per-edge urban_meters payload
using UrbanTestEdge = std::tuple<unsigned, unsigned, int, float>;
ContractorGraph makeUrbanGraph(const std::vector<UrbanTestEdge> &edges)
{
    std::vector<ContractorEdge> input_edges;
    auto id = 0u;
    auto max_id = 0u;
    for (const auto &[start, target, weight, urban] : edges)
    {
        max_id = std::max(std::max(start, target), max_id);
        input_edges.push_back(ContractorEdge{
            start,
            target,
            ContractorEdgeData{
                {weight}, {weight * 2}, {100.f}, 0, id++, false, true, true, {urban}}});
        input_edges.push_back(ContractorEdge{
            target,
            start,
            ContractorEdgeData{
                {weight}, {weight * 2}, {100.f}, 0, id++, false, true, true, {urban}}});
    }
    std::sort(input_edges.begin(), input_edges.end());

    return ContractorGraph(max_id + 1, input_edges);
}
} // namespace

#define HAS(a, b) BOOST_CHECK(query_graph.FindEdge(a, b) != SPECIAL_EDGEID);
#define NOT(a, b) BOOST_CHECK(query_graph.FindEdge(a, b) == SPECIAL_EDGEID);

BOOST_AUTO_TEST_SUITE(graph_contractor)

BOOST_AUTO_TEST_CASE(contract_graph)
{
    {
        /*
         *  0 - 1
         *
         *  1
         *  |
         *  0
         */
        const ContractorGraph g = makeGraph({{0, 1, 1}}); // start, target, weight

        auto query_graph = g;
        contractGraph(query_graph, {{1}, {1}});

        HAS(0, 1)
        NOT(1, 0)
    }

    {
        /*
         *  0 - 1 - 2
         *
         *    1
         *   / \
         *  0   2
         */

        const ContractorGraph g = makeGraph({{0, 1, 1}, // start, target, weight
                                             {1, 2, 1}});

        auto query_graph = g;
        contractGraph(query_graph, {{1}, {1}, {1}});

        HAS(0, 1)
        HAS(2, 1)

        NOT(1, 0)
        NOT(1, 2)
        NOT(2, 0)
        NOT(0, 2)
    }

    {
        /*
         *  0 - 1
         *   \ /
         *    2
         *
         *    2
         *   /|
         *  1 |
         *   \|
         *    0
         */

        const ContractorGraph g = makeGraph({{0, 1, 1}, // start, target, weight
                                             {1, 2, 1},
                                             {0, 2, 1}});

        auto query_graph = g;
        contractGraph(query_graph, {{1}, {1}, {1}});

        HAS(0, 1)
        HAS(0, 2)
        HAS(1, 2)

        NOT(1, 0)
        NOT(2, 0)
        NOT(2, 1)
    }

    {
        /*
         *  0 - 1
         *  |   |
         *  3 - 2
         *
         *      3
         *    / |
         *  1   |
         *  | X |
         *  0   2
         */

        const ContractorGraph g = makeGraph({{0, 1, 1}, // start, target, weight
                                             {1, 2, 1},
                                             {2, 3, 1},
                                             {3, 0, 1}});

        auto query_graph = g;
        contractGraph(query_graph, {{1}, {1}, {1}, {1}});

        HAS(0, 1)
        HAS(0, 3)
        HAS(2, 1)
        HAS(2, 3)
        HAS(1, 3)

        NOT(1, 0)
        NOT(3, 0)
        NOT(1, 2)
        NOT(3, 2)
        NOT(3, 1)

        NOT(0, 2)
        NOT(2, 0)
    }
}

BOOST_AUTO_TEST_CASE(contract_excludable_graph)
{
    {
        /*
         *  Same as above but 0 is uncontractible
         *
         *  0 - 1
         *  |   |
         *  3 - 2
         *
         *  0
         *  | \
         *  |   2
         *  | X |
         *  1   3
         */

        const ContractorGraph g = makeGraph({{0, 1, 1}, // start, target, weight
                                             {1, 2, 1},
                                             {2, 3, 1},
                                             {3, 0, 1}});

        auto [query_graph, ignore, ignore_urban] = contractExcludableGraph(
            g, {{1}, {1}, {1}, {1}}, {{true, true, true, true}, {false, true, true, true}});

        HAS(1, 0)
        HAS(1, 2)
        HAS(3, 0)
        HAS(3, 2)
        HAS(2, 0)

        NOT(0, 1)
        NOT(2, 1)
        NOT(0, 3)
        NOT(2, 3)
        NOT(0, 2)

        NOT(1, 3)
        NOT(3, 1)
    }
}

BOOST_AUTO_TEST_CASE(urban_meters_derivation)
{
    /*
     *  toContractorGraph derives urban_meters = node_urban_ratio[source] * distance
     *  for a turn edge; without a ratio vector the payload stays zero.
     */
    using extractor::EdgeBasedEdge;

    const std::vector<float> node_urban_ratio = {1.0f, 0.5f, 0.0f};

    std::vector<EdgeBasedEdge> input_edges;
    // source, target, turn_id, weight, duration, distance, forward, backward
    input_edges.emplace_back(
        0, 1, 0u, EdgeWeight{10}, EdgeDuration{10}, EdgeDistance{100.f}, true, false);
    input_edges.emplace_back(
        1, 2, 1u, EdgeWeight{10}, EdgeDuration{10}, EdgeDistance{60.f}, true, false);

    auto graph = toContractorGraph(3, input_edges, &node_urban_ratio);
    const auto urban = [&](NodeID from, NodeID to)
    {
        const auto edge = graph.FindEdge(from, to);
        BOOST_REQUIRE(edge != SPECIAL_EDGEID);
        return from_alias<float>(graph.GetEdgeData(edge).urban_meters);
    };

    BOOST_CHECK_EQUAL(urban(0, 1), 100.f); // ratio 1.0 * distance 100
    BOOST_CHECK_EQUAL(urban(1, 2), 30.f);  // ratio 0.5 * distance 60

    auto plain_graph = toContractorGraph(3, input_edges);
    const auto plain_edge = plain_graph.FindEdge(0, 1);
    BOOST_REQUIRE(plain_edge != SPECIAL_EDGEID);
    BOOST_CHECK_EQUAL(from_alias<float>(plain_graph.GetEdgeData(plain_edge).urban_meters), 0.f);
}

BOOST_AUTO_TEST_CASE(urban_meters_parallel_edge_min_merge)
{
    /*
     *  Parallel edges are deduplicated fieldwise with std::min; urban_meters
     *  follows distance's pre-existing semantics.
     */
    using extractor::EdgeBasedEdge;

    const std::vector<float> node_urban_ratio = {1.0f, 0.0f};

    std::vector<EdgeBasedEdge> input_edges;
    input_edges.emplace_back(
        0, 1, 0u, EdgeWeight{10}, EdgeDuration{10}, EdgeDistance{100.f}, true, false);
    input_edges.emplace_back(
        0, 1, 1u, EdgeWeight{20}, EdgeDuration{20}, EdgeDistance{40.f}, true, false);

    auto graph = toContractorGraph(2, input_edges, &node_urban_ratio);
    const auto edge = graph.FindEdge(0, 1);
    BOOST_REQUIRE(edge != SPECIAL_EDGEID);
    BOOST_CHECK_EQUAL(from_alias<float>(graph.GetEdgeData(edge).distance), 40.f);
    BOOST_CHECK_EQUAL(from_alias<float>(graph.GetEdgeData(edge).urban_meters), 40.f);
}

BOOST_AUTO_TEST_CASE(urban_meters_shortcut_sum)
{
    /*
     *  0 - 1 - 2 with only 1 contractable: the forced 0-2 shortcut carries the
     *  sum of the bypassed edges' urban_meters.
     */
    auto graph = makeUrbanGraph({{0, 1, 1, 70.f}, // start, target, weight, urban
                                 {1, 2, 1, 40.f}});

    contractGraph(graph, {false, true, false}, {{1}, {1}, {1}});

    const auto shortcut = graph.FindEdge(0, 2);
    BOOST_REQUIRE(shortcut != SPECIAL_EDGEID);
    BOOST_CHECK(graph.GetEdgeData(shortcut).shortcut);
    BOOST_CHECK_EQUAL(from_alias<float>(graph.GetEdgeData(shortcut).urban_meters), 110.f);
}

BOOST_AUTO_TEST_CASE(urban_meters_vector_alignment)
{
    /*
     *  0 - 1 - 2, all contractable: the leaves contract first (see the
     *  contract_graph case above), so both original edges survive as upward
     *  edges and the returned urban vector must line up with the query graph's
     *  edge array positionally.
     */
    const auto g = makeUrbanGraph({{0, 1, 1, 70.f}, // start, target, weight, urban
                                   {1, 2, 1, 40.f}});

    auto [query_graph, filters, urban_meters] =
        contractExcludableGraph(g, {{1}, {1}, {1}}, {{true, true, true}});

    BOOST_REQUIRE_EQUAL(urban_meters.size(), query_graph.GetNumberOfEdges());

    const auto edge_01 = query_graph.FindEdge(0, 1);
    const auto edge_21 = query_graph.FindEdge(2, 1);
    BOOST_REQUIRE(edge_01 != SPECIAL_EDGEID);
    BOOST_REQUIRE(edge_21 != SPECIAL_EDGEID);
    BOOST_CHECK_EQUAL(from_alias<float>(urban_meters[edge_01]), 70.f);
    BOOST_CHECK_EQUAL(from_alias<float>(urban_meters[edge_21]), 40.f);
}

BOOST_AUTO_TEST_SUITE_END()
