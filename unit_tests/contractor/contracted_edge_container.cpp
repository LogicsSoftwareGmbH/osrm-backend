#include "contractor/contracted_edge_container.hpp"

#include "../common/range_tools.hpp"

#include <boost/test/unit_test.hpp>

using namespace osrm;
using namespace osrm::contractor;

namespace osrm::contractor
{

bool operator!=(const QueryEdge &lhs, const QueryEdge &rhs) { return !(lhs == rhs); }

std::ostream &operator<<(std::ostream &out, const QueryEdge::EdgeData &data)
{
    out << "{" << data.turn_id << ", " << data.shortcut << ", " << data.duration << ", "
        << data.distance << ", " << data.weight << ", " << data.forward << ", " << data.backward
        << "}";
    return out;
}

std::ostream &operator<<(std::ostream &out, const QueryEdge &edge)
{
    out << "{" << edge.source << ", " << edge.target << ", " << edge.data << "}";
    return out;
}
} // namespace osrm::contractor

BOOST_AUTO_TEST_SUITE(contracted_edge_container)

BOOST_AUTO_TEST_CASE(merge_edge_of_multiple_graph)
{
    ContractedEdgeContainer container;

    std::vector<UrbanQueryEdge> edges;
    edges.push_back(UrbanQueryEdge{0, 1, {1, false, {3}, {3}, {6}, true, false}, {10.f}});
    edges.push_back(UrbanQueryEdge{1, 2, {2, false, {3}, {3}, {6}, true, false}, {20.f}});
    edges.push_back(UrbanQueryEdge{2, 0, {3, false, {3}, {3}, {6}, false, true}, {30.f}});
    edges.push_back(UrbanQueryEdge{2, 1, {4, false, {3}, {3}, {6}, false, true}, {40.f}});
    container.Insert(edges);

    edges.clear();
    edges.push_back(UrbanQueryEdge{0, 1, {1, false, {3}, {3}, {6}, true, false}, {10.f}});
    edges.push_back(UrbanQueryEdge{1, 2, {2, false, {3}, {3}, {6}, true, false}, {20.f}});
    edges.push_back(UrbanQueryEdge{2, 0, {3, false, {12}, {12}, {24}, false, true}, {31.f}});
    edges.push_back(UrbanQueryEdge{2, 1, {4, false, {12}, {12}, {24}, false, true}, {41.f}});
    container.Merge(edges);

    edges.clear();
    edges.push_back(UrbanQueryEdge{1, 4, {5, false, {3}, {3}, {6}, true, false}, {50.f}});
    container.Merge(edges);

    std::vector<QueryEdge> reference_edges;
    reference_edges.push_back(UrbanQueryEdge{0, 1, {1, false, {3}, {3}, {6}, true, false}});
    reference_edges.push_back(UrbanQueryEdge{1, 2, {2, false, {3}, {3}, {6}, true, false}});
    reference_edges.push_back(UrbanQueryEdge{1, 4, {5, false, {3}, {3}, {6}, true, false}});
    reference_edges.push_back(UrbanQueryEdge{2, 0, {3, false, {3}, {3}, {6}, false, true}});
    reference_edges.push_back(UrbanQueryEdge{2, 0, {3, false, {12}, {12}, {24}, false, true}});
    reference_edges.push_back(UrbanQueryEdge{2, 1, {4, false, {3}, {3}, {6}, false, true}});
    reference_edges.push_back(UrbanQueryEdge{2, 1, {4, false, {12}, {12}, {24}, false, true}});
    CHECK_EQUAL_COLLECTIONS(container.edges, reference_edges);

    // QueryEdge::operator== ignores urban_meters, so lock the payload down
    // explicitly: it must travel with its edge through Insert/Merge and the
    // final ordering (deduplicated edges keep the first graph's value)
    const std::vector<float> reference_urban = {10.f, 20.f, 50.f, 30.f, 31.f, 40.f, 41.f};
    BOOST_REQUIRE_EQUAL(container.edges.size(), reference_urban.size());
    for (std::size_t i = 0; i < container.edges.size(); ++i)
    {
        BOOST_CHECK_EQUAL(from_alias<float>(container.edges[i].urban_meters), reference_urban[i]);
    }

    auto filters = container.MakeEdgeFilters();
    BOOST_CHECK_EQUAL(filters.size(), 2);

    REQUIRE_SIZE_RANGE(filters[0], 7);
    CHECK_EQUAL_RANGE(filters[0], true, true, false, true, true, true, true);

    REQUIRE_SIZE_RANGE(filters[1], 7);
    CHECK_EQUAL_RANGE(filters[1], true, true, true, true, false, true, false);
}

BOOST_AUTO_TEST_CASE(merge_edge_of_multiple_disjoint_graph)
{
    ContractedEdgeContainer container;

    std::vector<UrbanQueryEdge> edges;
    edges.push_back(UrbanQueryEdge{0, 1, {1, false, {3}, {3}, {6}, true, false}});
    edges.push_back(UrbanQueryEdge{1, 2, {2, false, {3}, {3}, {6}, true, false}});
    edges.push_back(UrbanQueryEdge{2, 0, {3, false, {12}, {12}, {24}, false, true}});
    edges.push_back(UrbanQueryEdge{2, 1, {4, false, {12}, {12}, {24}, false, true}});
    container.Merge(edges);

    edges.clear();
    edges.push_back(UrbanQueryEdge{1, 4, {5, false, {3}, {3}, {6}, true, false}});
    container.Merge(edges);

    std::vector<QueryEdge> reference_edges;
    reference_edges.push_back(UrbanQueryEdge{0, 1, {1, false, {3}, {3}, {6}, true, false}});
    reference_edges.push_back(UrbanQueryEdge{1, 2, {2, false, {3}, {3}, {6}, true, false}});
    reference_edges.push_back(UrbanQueryEdge{1, 4, {5, false, {3}, {3}, {6}, true, false}});
    reference_edges.push_back(UrbanQueryEdge{2, 0, {3, false, {12}, {12}, {24}, false, true}});
    reference_edges.push_back(UrbanQueryEdge{2, 1, {4, false, {12}, {12}, {24}, false, true}});
    CHECK_EQUAL_COLLECTIONS(container.edges, reference_edges);

    auto filters = container.MakeEdgeFilters();
    BOOST_CHECK_EQUAL(filters.size(), 2);

    REQUIRE_SIZE_RANGE(filters[0], 5);
    CHECK_EQUAL_RANGE(filters[0], true, true, false, true, true);

    REQUIRE_SIZE_RANGE(filters[1], 5);
    CHECK_EQUAL_RANGE(filters[1], false, false, true, false, false);
}

BOOST_AUTO_TEST_SUITE_END()
