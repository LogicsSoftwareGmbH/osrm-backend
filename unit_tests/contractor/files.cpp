#include "contractor/files.hpp"
#include "contractor/graph_contractor_adaptors.hpp"

#include "../common/range_tools.hpp"
#include "../common/temporary_file.hpp"
#include "helper.hpp"

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(tar)

using namespace osrm;
using namespace osrm::contractor;
using namespace osrm::unit_test;

BOOST_AUTO_TEST_CASE(read_write_hsgr)
{
    auto reference_connectivity_checksum = 0xDEADBEEF;
    std::vector<TestEdge> edges = {TestEdge{0, 1, 3},
                                   TestEdge{0, 5, 1},
                                   TestEdge{1, 3, 3},
                                   TestEdge{1, 4, 1},
                                   TestEdge{3, 1, 1},
                                   TestEdge{4, 3, 1},
                                   TestEdge{5, 1, 1}};
    auto reference_graph = QueryGraph{6, toEdges<QueryEdge>(makeGraph(edges))};
    std::vector<std::vector<bool>> reference_filters = {
        {false, false, true, true, false, false, true},
        {true, false, true, false, true, false, true},
        {false, false, false, false, false, false, false},
        {true, true, true, true, true, true, true},
    };

    std::unordered_map<std::string, ContractedMetric> reference_metrics = {
        {"duration", {std::move(reference_graph), std::move(reference_filters)}}};

    TemporaryFile tmp{TEST_DATA_DIR "/read_write_hsgr_test.osrm.hsgr"};
    contractor::files::writeGraph(tmp.path, reference_metrics, reference_connectivity_checksum);

    unsigned connectivity_checksum;

    std::unordered_map<std::string, ContractedMetric> metrics = {{"duration", {}}};
    contractor::files::readGraph(tmp.path, metrics, connectivity_checksum);

    BOOST_CHECK_EQUAL(connectivity_checksum, reference_connectivity_checksum);
    BOOST_CHECK_EQUAL(metrics["duration"].edge_filter.size(),
                      reference_metrics["duration"].edge_filter.size());
    CHECK_EQUAL_COLLECTIONS(metrics["duration"].edge_filter[0],
                            reference_metrics["duration"].edge_filter[0]);
    CHECK_EQUAL_COLLECTIONS(metrics["duration"].edge_filter[1],
                            reference_metrics["duration"].edge_filter[1]);
    CHECK_EQUAL_COLLECTIONS(metrics["duration"].edge_filter[2],
                            reference_metrics["duration"].edge_filter[2]);
    CHECK_EQUAL_COLLECTIONS(metrics["duration"].edge_filter[3],
                            reference_metrics["duration"].edge_filter[3]);
}

BOOST_AUTO_TEST_CASE(read_write_urban)
{
    const std::uint32_t reference_connectivity_checksum = 0xDEADBEEF;
    const std::uint32_t reference_config_identity = 0xC0FFEE42;
    const std::vector<EdgeDistance> reference_urban_meters = {
        EdgeDistance{0.f}, EdgeDistance{12.5f}, EdgeDistance{100.f}, EdgeDistance{37.25f}};
    extractor::UrbanClassWeights reference_weights{};
    reference_weights[0] = 1.0f;
    reference_weights[1] = 0.5f;

    TemporaryFile tmp{TEST_DATA_DIR "/read_write_urban_test.osrm.urban"};
    contractor::files::writeUrbanData(tmp.path,
                                      "duration",
                                      reference_urban_meters,
                                      reference_weights,
                                      reference_connectivity_checksum,
                                      reference_config_identity);

    std::vector<EdgeDistance> urban_meters;
    std::vector<float> class_weights;
    std::uint32_t connectivity_checksum = 0;
    std::uint32_t config_identity = 0;
    contractor::files::readUrbanData(
        tmp.path, "duration", urban_meters, class_weights, connectivity_checksum, config_identity);

    // the two headers bind the side-car to the .osrm.hsgr written in the same
    // run and to the .osrm.urban_config its payload was derived from; storage
    // compares both before accepting the file
    BOOST_CHECK_EQUAL(connectivity_checksum, reference_connectivity_checksum);
    BOOST_CHECK_EQUAL(config_identity, reference_config_identity);

    BOOST_REQUIRE_EQUAL(urban_meters.size(), reference_urban_meters.size());
    for (std::size_t i = 0; i < urban_meters.size(); ++i)
    {
        BOOST_CHECK_EQUAL(from_alias<float>(urban_meters[i]),
                          from_alias<float>(reference_urban_meters[i]));
    }

    BOOST_REQUIRE_EQUAL(class_weights.size(), reference_weights.size());
    for (std::size_t i = 0; i < class_weights.size(); ++i)
    {
        BOOST_CHECK_EQUAL(class_weights[i], reference_weights[i]);
    }
}

BOOST_AUTO_TEST_SUITE_END()
