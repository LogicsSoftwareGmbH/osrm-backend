#include "storage/urban_validation.hpp"

#include "contractor/files.hpp"
#include "extractor/files.hpp"

#include "../common/temporary_file.hpp"

#include <boost/test/unit_test.hpp>

#include <optional>
#include <vector>

BOOST_AUTO_TEST_SUITE(urban_validation)

using namespace osrm;
using namespace osrm::storage;

namespace
{
// A complete, mutually consistent set of urban side-car files plus the two
// stock files the validation predicates compare them against. Individual tests
// then break exactly one binding and check the file is rejected.
struct UrbanFixture
{
    TemporaryFile properties_file, config_file, urban_file, hsgr_file;
    extractor::ProfileProperties properties;
    extractor::UrbanClassWeights weights{};
    std::uint32_t identity = 0;
    const std::uint32_t checksum = 0xDEADBEEF;
    const std::vector<EdgeDistance> urban_meters = {
        EdgeDistance{10.f}, EdgeDistance{20.f}, EdgeDistance{30.f}, EdgeDistance{40.f}};

    UrbanFixture()
    {
        properties.SetClassName(0, "urban");
        properties.SetClassName(1, "suburban");
        weights[0] = 1.0f;
        weights[1] = 0.5f;
        identity = extractor::computeUrbanConfigIdentity(properties, weights);

        extractor::files::writeProfileProperties(properties_file.path, properties);
        extractor::files::writeUrbanConfig(config_file.path, weights, identity);
        contractor::files::writeUrbanData(
            urban_file.path, "duration", urban_meters, weights, checksum, identity);
        writeFakeGraph(checksum, urban_meters.size());
    }

    // the side-car predicate only reads the connectivity checksum block and the
    // per-metric edge-array element count, so a minimal tar with those two
    // records stands in for a real .osrm.hsgr
    void writeFakeGraph(const std::uint32_t graph_checksum, const std::uint64_t edge_count)
    {
        tar::FileWriter writer{hsgr_file.path, tar::FileWriter::GenerateFingerprint};
        writer.WriteElementCount64("/ch/connectivity_checksum", 1);
        writer.WriteFrom("/ch/connectivity_checksum", graph_checksum);
        writer.WriteElementCount64("/ch/metrics/duration/contracted_graph/edge_array", edge_count);
    }
};
} // namespace

BOOST_AUTO_TEST_CASE(config_accepted_when_identity_matches_properties)
{
    UrbanFixture f;
    const auto identity = urbanConfigIdentityIfValid(f.config_file.path, f.properties_file.path);
    BOOST_REQUIRE(identity.has_value());
    BOOST_CHECK_EQUAL(*identity, f.identity);
}

BOOST_AUTO_TEST_CASE(config_rejected_when_absent)
{
    UrbanFixture f;
    BOOST_CHECK(!urbanConfigIdentityIfValid(f.config_file.path.string() + ".does_not_exist",
                                            f.properties_file.path));
}

BOOST_AUTO_TEST_CASE(config_rejected_on_tampered_identity)
{
    UrbanFixture f;
    extractor::files::writeUrbanConfig(f.config_file.path, f.weights, f.identity + 1);
    BOOST_CHECK(!urbanConfigIdentityIfValid(f.config_file.path, f.properties_file.path));
}

BOOST_AUTO_TEST_CASE(rejected_when_class_mapping_changed)
{
    // the stock-extract-over-a-fork-directory scenario: a re-extract that knows
    // nothing about the side-cars rewrites .osrm.properties with its own class
    // mapping and leaves both urban files behind — bit indices into the LUT are
    // meaningless under the new mapping, so both files must be rejected
    UrbanFixture f;
    extractor::ProfileProperties other;
    other.SetClassName(0, "toll");
    other.SetClassName(1, "motorway");
    extractor::files::writeProfileProperties(f.properties_file.path, other);

    BOOST_CHECK(!urbanConfigIdentityIfValid(f.config_file.path, f.properties_file.path));
    BOOST_CHECK(!urbanSideCarMatchesGraph(
        f.urban_file.path, f.hsgr_file.path, f.properties_file.path, std::nullopt));
}

BOOST_AUTO_TEST_CASE(side_car_accepted_when_all_bindings_match)
{
    UrbanFixture f;
    BOOST_CHECK(urbanSideCarMatchesGraph(
        f.urban_file.path, f.hsgr_file.path, f.properties_file.path, f.identity));
    // also valid without a config next to it: the embedded LUT self-validates
    // against the properties
    BOOST_CHECK(urbanSideCarMatchesGraph(
        f.urban_file.path, f.hsgr_file.path, f.properties_file.path, std::nullopt));
}

BOOST_AUTO_TEST_CASE(side_car_rejected_when_absent)
{
    UrbanFixture f;
    BOOST_CHECK(!urbanSideCarMatchesGraph(f.urban_file.path.string() + ".does_not_exist",
                                          f.hsgr_file.path,
                                          f.properties_file.path,
                                          f.identity));
}

BOOST_AUTO_TEST_CASE(side_car_rejected_on_connectivity_checksum_mismatch)
{
    UrbanFixture f;
    f.writeFakeGraph(f.checksum + 1, f.urban_meters.size());
    BOOST_CHECK(!urbanSideCarMatchesGraph(
        f.urban_file.path, f.hsgr_file.path, f.properties_file.path, f.identity));
}

BOOST_AUTO_TEST_CASE(side_car_rejected_on_edge_count_mismatch)
{
    UrbanFixture f;
    f.writeFakeGraph(f.checksum, f.urban_meters.size() + 1);
    BOOST_CHECK(!urbanSideCarMatchesGraph(
        f.urban_file.path, f.hsgr_file.path, f.properties_file.path, f.identity));
}

BOOST_AUTO_TEST_CASE(side_car_rejected_when_derived_from_other_weights)
{
    // weight-only re-extract without a re-contract: graph and edge counts are
    // unchanged, but the config next to the side-car now carries a different
    // identity — the contracted payload no longer matches the LUT that would
    // win at query time
    UrbanFixture f;
    auto changed = f.weights;
    changed[1] = 0.9f;
    const auto changed_identity = extractor::computeUrbanConfigIdentity(f.properties, changed);
    BOOST_REQUIRE(changed_identity != f.identity);
    BOOST_CHECK(!urbanSideCarMatchesGraph(
        f.urban_file.path, f.hsgr_file.path, f.properties_file.path, changed_identity));
}

BOOST_AUTO_TEST_SUITE_END()
