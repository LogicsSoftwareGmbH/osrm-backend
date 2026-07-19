#include <boost/test/unit_test.hpp>

#include "coordinates.hpp"
#include "fixture.hpp"
#include "waypoint_check.hpp"

#include "osrm/table_parameters.hpp"

#include "osrm/coordinate.hpp"
#include "osrm/json_container.hpp"
#include "osrm/osrm.hpp"
#include "osrm/status.hpp"

osrm::Status run_table_json(const osrm::OSRM &osrm,
                            const osrm::TableParameters &params,
                            osrm::json::Object &json_result,
                            bool use_json_only_api)
{
    if (use_json_only_api)
    {
        return osrm.Table(params, json_result);
    }
    osrm::engine::api::ResultT result = osrm::json::Object();
    auto rc = osrm.Table(params, result);
    json_result = std::get<osrm::json::Object>(result);
    return rc;
}

BOOST_AUTO_TEST_SUITE(table)

void test_table_three_coords_one_source_one_dest_matrix(bool use_json_only_api)
{
    using namespace osrm;

    auto osrm = getOSRM(OSRM_TEST_DATA_DIR "/ch/monaco.osrm");

    TableParameters params;
    params.coordinates.push_back(get_dummy_location());
    params.coordinates.push_back(get_dummy_location());
    params.coordinates.push_back(get_dummy_location());
    params.sources.push_back(0);
    params.destinations.push_back(2);
    params.annotations = TableParameters::AnnotationsType::All;

    json::Object json_result;
    const auto rc = run_table_json(osrm, params, json_result, use_json_only_api);

    BOOST_CHECK(rc == Status::Ok || rc == Status::Error);
    const auto code = std::get<json::String>(json_result.values.at("code")).value;
    BOOST_CHECK_EQUAL(code, "Ok");

    // check that returned durations error is expected size and proportions
    // this test expects a 1x1 matrix
    const auto &durations_array = std::get<json::Array>(json_result.values.at("durations")).values;
    BOOST_CHECK_EQUAL(durations_array.size(), params.sources.size());
    for (unsigned int i = 0; i < durations_array.size(); i++)
    {
        const auto durations_matrix = std::get<json::Array>(durations_array[i]).values;
        BOOST_CHECK_EQUAL(durations_matrix.size(),
                          params.sources.size() * params.destinations.size());
    }

    // check that returned distances error is expected size and proportions
    // this test expects a 1x1 matrix
    const auto &distances_array = std::get<json::Array>(json_result.values.at("distances")).values;
    BOOST_CHECK_EQUAL(distances_array.size(), params.sources.size());
    for (unsigned int i = 0; i < distances_array.size(); i++)
    {
        const auto distances_matrix = std::get<json::Array>(distances_array[i]).values;
        BOOST_CHECK_EQUAL(distances_matrix.size(),
                          params.sources.size() * params.destinations.size());
    }

    // check destinations array of waypoint objects
    const auto &destinations_array =
        std::get<json::Array>(json_result.values.at("destinations")).values;
    BOOST_CHECK_EQUAL(destinations_array.size(), params.destinations.size());
    for (const auto &destination : destinations_array)
    {
        BOOST_CHECK(waypoint_check(destination));
    }
    // check sources array of waypoint objects
    const auto &sources_array = std::get<json::Array>(json_result.values.at("sources")).values;
    BOOST_CHECK_EQUAL(sources_array.size(), params.sources.size());
    for (const auto &source : sources_array)
    {
        BOOST_CHECK(waypoint_check(source));
    }
}
BOOST_AUTO_TEST_CASE(test_table_three_coords_one_source_one_dest_matrix_old_api)
{
    test_table_three_coords_one_source_one_dest_matrix(true);
}
BOOST_AUTO_TEST_CASE(test_table_three_coords_one_source_one_dest_matrix_new_api)
{
    test_table_three_coords_one_source_one_dest_matrix(false);
}

void test_table_three_coords_one_source_one_dest_matrix_no_waypoints(bool use_json_only_api)
{
    using namespace osrm;

    auto osrm = getOSRM(OSRM_TEST_DATA_DIR "/ch/monaco.osrm");

    TableParameters params;
    params.skip_waypoints = true;
    params.coordinates.push_back(get_dummy_location());
    params.coordinates.push_back(get_dummy_location());
    params.coordinates.push_back(get_dummy_location());
    params.sources.push_back(0);
    params.destinations.push_back(2);
    params.annotations = TableParameters::AnnotationsType::All;

    json::Object json_result;
    const auto rc = run_table_json(osrm, params, json_result, use_json_only_api);

    BOOST_CHECK(rc == Status::Ok || rc == Status::Error);
    const auto code = std::get<json::String>(json_result.values.at("code")).value;
    BOOST_CHECK_EQUAL(code, "Ok");

    // check that returned durations error is expected size and proportions
    // this test expects a 1x1 matrix
    const auto &durations_array = std::get<json::Array>(json_result.values.at("durations")).values;
    BOOST_CHECK_EQUAL(durations_array.size(), params.sources.size());
    for (unsigned int i = 0; i < durations_array.size(); i++)
    {
        const auto durations_matrix = std::get<json::Array>(durations_array[i]).values;
        BOOST_CHECK_EQUAL(durations_matrix.size(),
                          params.sources.size() * params.destinations.size());
    }

    // check that returned distances error is expected size and proportions
    // this test expects a 1x1 matrix
    const auto &distances_array = std::get<json::Array>(json_result.values.at("distances")).values;
    BOOST_CHECK_EQUAL(distances_array.size(), params.sources.size());
    for (unsigned int i = 0; i < distances_array.size(); i++)
    {
        const auto distances_matrix = std::get<json::Array>(distances_array[i]).values;
        BOOST_CHECK_EQUAL(distances_matrix.size(),
                          params.sources.size() * params.destinations.size());
    }

    // waypoint arrays should be missing
    BOOST_CHECK(!json_result.values.contains("destinations"));
    BOOST_CHECK(!json_result.values.contains("sources"));
}
BOOST_AUTO_TEST_CASE(test_table_three_coords_one_source_one_dest_matrix_no_waypoints_old_api)
{
    test_table_three_coords_one_source_one_dest_matrix_no_waypoints(true);
}
BOOST_AUTO_TEST_CASE(test_table_three_coords_one_source_one_dest_matrix_no_waypoints_new_api)
{
    test_table_three_coords_one_source_one_dest_matrix_no_waypoints(false);
}

void test_table_three_coords_one_source_matrix(bool use_json_only_api)
{
    using namespace osrm;

    auto osrm = getOSRM(OSRM_TEST_DATA_DIR "/ch/monaco.osrm");

    TableParameters params;
    params.coordinates.push_back(get_dummy_location());
    params.coordinates.push_back(get_dummy_location());
    params.coordinates.push_back(get_dummy_location());
    params.sources.push_back(0);
    engine::api::ResultT result = json::Object();

    json::Object json_result;
    const auto rc = run_table_json(osrm, params, json_result, use_json_only_api);

    BOOST_CHECK(rc == Status::Ok || rc == Status::Error);
    const auto code = std::get<json::String>(json_result.values.at("code")).value;
    BOOST_CHECK_EQUAL(code, "Ok");

    // check that returned durations error is expected size and proportions
    // this test expects a 1x3 matrix
    const auto &durations_array = std::get<json::Array>(json_result.values.at("durations")).values;
    BOOST_CHECK_EQUAL(durations_array.size(), params.sources.size());
    for (unsigned int i = 0; i < durations_array.size(); i++)
    {
        const auto durations_matrix = std::get<json::Array>(durations_array[i]).values;
        BOOST_CHECK_EQUAL(std::get<json::Number>(durations_matrix[i]).value, 0);
        BOOST_CHECK_EQUAL(durations_matrix.size(),
                          params.sources.size() * params.coordinates.size());
    }
    // check destinations array of waypoint objects
    const auto &destinations_array =
        std::get<json::Array>(json_result.values.at("destinations")).values;
    BOOST_CHECK_EQUAL(destinations_array.size(), params.coordinates.size());
    for (const auto &destination : destinations_array)
    {
        BOOST_CHECK(waypoint_check(destination));
    }
    // check sources array of waypoint objects
    const auto &sources_array = std::get<json::Array>(json_result.values.at("sources")).values;
    BOOST_CHECK_EQUAL(sources_array.size(), params.sources.size());
    for (const auto &source : sources_array)
    {
        BOOST_CHECK(waypoint_check(source));
    }
}
BOOST_AUTO_TEST_CASE(test_table_three_coords_one_source_matrix_old_api)
{
    test_table_three_coords_one_source_matrix(true);
}
BOOST_AUTO_TEST_CASE(test_table_three_coords_one_source_matrix_new_api)
{
    test_table_three_coords_one_source_matrix(false);
}

void test_table_three_coordinates_matrix(bool use_json_only_api)
{
    using namespace osrm;

    auto osrm = getOSRM(OSRM_TEST_DATA_DIR "/ch/monaco.osrm");

    TableParameters params;
    params.coordinates.push_back(get_dummy_location());
    params.coordinates.push_back(get_dummy_location());
    params.coordinates.push_back(get_dummy_location());
    params.annotations = TableParameters::AnnotationsType::Duration;

    json::Object json_result;
    const auto rc = run_table_json(osrm, params, json_result, use_json_only_api);

    BOOST_CHECK(rc == Status::Ok || rc == Status::Error);
    const auto code = std::get<json::String>(json_result.values.at("code")).value;
    BOOST_CHECK_EQUAL(code, "Ok");

    // check that returned durations error is expected size and proportions
    // this test expects a 3x3 matrix
    const auto &durations_array = std::get<json::Array>(json_result.values.at("durations")).values;
    BOOST_CHECK_EQUAL(durations_array.size(), params.coordinates.size());
    for (unsigned int i = 0; i < durations_array.size(); i++)
    {
        const auto durations_matrix = std::get<json::Array>(durations_array[i]).values;
        BOOST_CHECK_EQUAL(std::get<json::Number>(durations_matrix[i]).value, 0);
        BOOST_CHECK_EQUAL(durations_matrix.size(), params.coordinates.size());
    }
    const auto &destinations_array =
        std::get<json::Array>(json_result.values.at("destinations")).values;
    for (const auto &destination : destinations_array)
    {
        BOOST_CHECK(waypoint_check(destination));
    }
    const auto &sources_array = std::get<json::Array>(json_result.values.at("sources")).values;
    BOOST_CHECK_EQUAL(sources_array.size(), params.coordinates.size());
    for (const auto &source : sources_array)
    {
        BOOST_CHECK(waypoint_check(source));
    }
}
BOOST_AUTO_TEST_CASE(test_table_three_coordinates_matrix_old_api)
{
    test_table_three_coordinates_matrix(true);
}
BOOST_AUTO_TEST_CASE(test_table_three_coordinates_matrix_new_api)
{
    test_table_three_coordinates_matrix(false);
}

// See https://github.com/Project-OSRM/osrm-backend/pull/3992
void test_table_no_segment_for_some_coordinates(bool use_json_only_api)
{
    using namespace osrm;

    auto osrm = getOSRM(OSRM_TEST_DATA_DIR "/ch/monaco.osrm");

    TableParameters params;
    params.coordinates.push_back(get_dummy_location());
    params.coordinates.push_back(get_dummy_location());
    // resembles query option: `&radiuses=0;`
    params.radiuses.push_back(std::make_optional(0.));
    params.radiuses.push_back(std::nullopt);

    json::Object json_result;
    const auto rc = run_table_json(osrm, params, json_result, use_json_only_api);

    BOOST_CHECK(rc == Status::Error);
    const auto code = std::get<json::String>(json_result.values.at("code")).value;
    BOOST_CHECK_EQUAL(code, "NoSegment");
    const auto message = std::get<json::String>(json_result.values.at("message")).value;
    BOOST_CHECK_EQUAL(message, "Could not find a matching segment for coordinate 0");
}
BOOST_AUTO_TEST_CASE(test_table_no_segment_for_some_coordinates_old_api)
{
    test_table_no_segment_for_some_coordinates(true);
}
BOOST_AUTO_TEST_CASE(test_table_no_segment_for_some_coordinates_new_api)
{
    test_table_no_segment_for_some_coordinates(false);
}

BOOST_AUTO_TEST_CASE(test_table_serialiaze_fb)
{
    using namespace osrm;

    auto osrm = getOSRM(OSRM_TEST_DATA_DIR "/ch/monaco.osrm");

    TableParameters params;
    params.coordinates.push_back(get_dummy_location());
    params.coordinates.push_back(get_dummy_location());
    params.coordinates.push_back(get_dummy_location());
    params.sources.push_back(0);
    params.destinations.push_back(2);
    params.annotations = TableParameters::AnnotationsType::All;

    engine::api::ResultT result = flatbuffers::FlatBufferBuilder();

    const auto rc = osrm.Table(params, result);

    BOOST_CHECK(rc == Status::Ok || rc == Status::Error);

    auto &fb_result = std::get<flatbuffers::FlatBufferBuilder>(result);
    auto fb = engine::api::fbresult::GetFBResult(fb_result.GetBufferPointer());
    BOOST_CHECK(!fb->error());
    BOOST_CHECK(fb->table() != nullptr);

    // check that returned durations error is expected size and proportions
    // this test expects a 1x1 matrix
    BOOST_CHECK(fb->table()->durations() != nullptr);
    auto durations_array = fb->table()->durations();
    BOOST_CHECK_EQUAL(durations_array->size(), params.sources.size() * params.destinations.size());

    // check that returned distances error is expected size and proportions
    // this test expects a 1x1 matrix
    BOOST_CHECK(fb->table()->distances() != nullptr);
    auto distances_array = fb->table()->distances();
    BOOST_CHECK_EQUAL(distances_array->size(), params.sources.size() * params.destinations.size());

    // check destinations array of waypoint objects
    const auto &destinations_array = fb->table()->destinations();
    BOOST_CHECK_EQUAL(destinations_array->size(), params.destinations.size());
    for (const auto destination : *destinations_array)
    {
        BOOST_CHECK(waypoint_check(destination));
    }
    // check sources array of waypoint objects
    const auto &sources_array = fb->waypoints();
    BOOST_CHECK_EQUAL(sources_array->size(), params.sources.size());
    for (const auto source : *sources_array)
    {
        BOOST_CHECK(waypoint_check(source));
    }
}

BOOST_AUTO_TEST_CASE(test_table_serialiaze_fb_no_waypoints)
{
    using namespace osrm;

    auto osrm = getOSRM(OSRM_TEST_DATA_DIR "/ch/monaco.osrm");

    TableParameters params;
    params.skip_waypoints = true;
    params.coordinates.push_back(get_dummy_location());
    params.coordinates.push_back(get_dummy_location());
    params.coordinates.push_back(get_dummy_location());
    params.sources.push_back(0);
    params.destinations.push_back(2);
    params.annotations = TableParameters::AnnotationsType::All;

    engine::api::ResultT result = flatbuffers::FlatBufferBuilder();

    const auto rc = osrm.Table(params, result);

    BOOST_CHECK(rc == Status::Ok || rc == Status::Error);

    auto &fb_result = std::get<flatbuffers::FlatBufferBuilder>(result);
    auto fb = engine::api::fbresult::GetFBResult(fb_result.GetBufferPointer());
    BOOST_CHECK(!fb->error());
    BOOST_CHECK(fb->table() != nullptr);

    // check that returned durations error is expected size and proportions
    // this test expects a 1x1 matrix
    BOOST_CHECK(fb->table()->durations() != nullptr);
    auto durations_array = fb->table()->durations();
    BOOST_CHECK_EQUAL(durations_array->size(), params.sources.size() * params.destinations.size());

    // check that returned distances error is expected size and proportions
    // this test expects a 1x1 matrix
    BOOST_CHECK(fb->table()->distances() != nullptr);
    auto distances_array = fb->table()->distances();
    BOOST_CHECK_EQUAL(distances_array->size(), params.sources.size() * params.destinations.size());

    BOOST_CHECK(fb->table()->destinations() == nullptr);
    BOOST_CHECK(fb->waypoints() == nullptr);
}

void test_table_urban_share(bool use_json_only_api)
{
    using namespace osrm;

    auto osrm = getOSRM(OSRM_TEST_DATA_DIR "/ch/monaco.osrm");

    TableParameters params;
    for (const auto &location : get_locations_in_big_component())
    {
        params.coordinates.push_back(location);
    }
    params.annotations = TableParameters::AnnotationsType::Duration |
                         TableParameters::AnnotationsType::UrbanShare;

    json::Object json_result;
    const auto rc = run_table_json(osrm, params, json_result, use_json_only_api);
    BOOST_REQUIRE(rc == Status::Ok);

    const auto code = std::get<json::String>(json_result.values.at("code")).value;
    BOOST_CHECK_EQUAL(code, "Ok");

    // distances were computed internally as the share's denominator but not requested
    BOOST_CHECK(json_result.values.find("distances") == json_result.values.end());

    const auto &urban_rows = std::get<json::Array>(json_result.values.at("urban_shares")).values;
    BOOST_REQUIRE_EQUAL(urban_rows.size(), params.coordinates.size());
    std::size_t numeric_cells = 0;
    for (std::size_t row = 0; row < urban_rows.size(); ++row)
    {
        const auto &cells = std::get<json::Array>(urban_rows[row]).values;
        BOOST_REQUIRE_EQUAL(cells.size(), params.coordinates.size());
        for (std::size_t column = 0; column < cells.size(); ++column)
        {
            if (row == column)
            {
                // zero-length path has no denominator
                BOOST_CHECK(std::holds_alternative<json::Null>(cells[column]));
                continue;
            }
            if (std::holds_alternative<json::Number>(cells[column]))
            {
                ++numeric_cells;
                const auto share = std::get<json::Number>(cells[column]).value;
                BOOST_CHECK(share >= 0.);
                BOOST_CHECK(share <= 1.);
            }
        }
    }
    // the big-component locations are mutually reachable
    BOOST_CHECK_EQUAL(numeric_cells,
                      params.coordinates.size() * (params.coordinates.size() - 1));
}

BOOST_AUTO_TEST_CASE(test_table_urban_share_old_api) { test_table_urban_share(true); }
BOOST_AUTO_TEST_CASE(test_table_urban_share_new_api) { test_table_urban_share(false); }

BOOST_AUTO_TEST_CASE(test_table_annotations_all_has_no_urban_share)
{
    using namespace osrm;

    auto osrm = getOSRM(OSRM_TEST_DATA_DIR "/ch/monaco.osrm");

    TableParameters params;
    for (const auto &location : get_locations_in_big_component())
    {
        params.coordinates.push_back(location);
    }
    params.annotations = TableParameters::AnnotationsType::All;

    json::Object json_result;
    const auto rc = run_table_json(osrm, params, json_result, true);
    BOOST_REQUIRE(rc == Status::Ok);

    // All predates urban_share and must stay Duration|Distance so existing
    // clients' responses do not change
    BOOST_CHECK(json_result.values.find("durations") != json_result.values.end());
    BOOST_CHECK(json_result.values.find("distances") != json_result.values.end());
    BOOST_CHECK(json_result.values.find("urban_shares") == json_result.values.end());
}

BOOST_AUTO_TEST_CASE(test_table_urban_share_asymmetric)
{
    using namespace osrm;

    auto osrm = getOSRM(OSRM_TEST_DATA_DIR "/ch/monaco.osrm");

    // reference: the full 3x3 matrix over the big-component locations
    TableParameters full_params;
    for (const auto &location : get_locations_in_big_component())
    {
        full_params.coordinates.push_back(location);
    }
    full_params.annotations = TableParameters::AnnotationsType::UrbanShare;

    json::Object full_result;
    BOOST_REQUIRE(osrm.Table(full_params, full_result) == Status::Ok);
    const auto &full_rows = std::get<json::Array>(full_result.values.at("urban_shares")).values;

    // sources != destinations: the matrix must be n_sources x n_destinations and
    // indexed [source_row][destination_column] like durations — a transposed or
    // mis-shaped implementation fails here
    TableParameters params;
    params.coordinates = full_params.coordinates;
    params.sources = {2};
    params.destinations = {0, 2};
    params.annotations = TableParameters::AnnotationsType::UrbanShare;

    json::Object json_result;
    BOOST_REQUIRE(osrm.Table(params, json_result) == Status::Ok);

    const auto &rows = std::get<json::Array>(json_result.values.at("urban_shares")).values;
    BOOST_REQUIRE_EQUAL(rows.size(), 1);
    const auto &cells = std::get<json::Array>(rows[0]).values;
    BOOST_REQUIRE_EQUAL(cells.size(), 2);

    // cell (source 2 -> destination 0) must equal full[2][0]
    const auto &full_cells = std::get<json::Array>(full_rows[2]).values;
    BOOST_REQUIRE(std::holds_alternative<json::Number>(cells[0]));
    BOOST_REQUIRE(std::holds_alternative<json::Number>(full_cells[0]));
    BOOST_CHECK_EQUAL(std::get<json::Number>(cells[0]).value,
                      std::get<json::Number>(full_cells[0]).value);

    // source 2 -> destination 2 is a diagonal cell: null
    BOOST_CHECK(std::holds_alternative<json::Null>(cells[1]));
}

BOOST_AUTO_TEST_CASE(test_table_urban_share_not_implemented_on_mld)
{
    using namespace osrm;

    auto osrm =
        getOSRM(OSRM_TEST_DATA_DIR "/mld/monaco.osrm", osrm::EngineConfig::Algorithm::MLD);

    TableParameters params;
    for (const auto &location : get_locations_in_big_component())
    {
        params.coordinates.push_back(location);
    }
    params.annotations = TableParameters::AnnotationsType::UrbanShare;

    json::Object json_result;
    const auto rc = run_table_json(osrm, params, json_result, true);
    BOOST_REQUIRE(rc == Status::Error);
    const auto code = std::get<json::String>(json_result.values.at("code")).value;
    BOOST_CHECK_EQUAL(code, "NotImplemented");
}

BOOST_AUTO_TEST_CASE(test_table_urban_share_serialize_fb)
{
    using namespace osrm;

    auto osrm = getOSRM(OSRM_TEST_DATA_DIR "/ch/monaco.osrm");

    TableParameters params;
    for (const auto &location : get_locations_in_big_component())
    {
        params.coordinates.push_back(location);
    }
    params.annotations = TableParameters::AnnotationsType::Duration |
                         TableParameters::AnnotationsType::UrbanShare;

    engine::api::ResultT result = flatbuffers::FlatBufferBuilder();
    const auto rc = osrm.Table(params, result);
    BOOST_REQUIRE(rc == Status::Ok);

    auto &fb_result = std::get<flatbuffers::FlatBufferBuilder>(result);
    auto fb = engine::api::fbresult::GetFBResult(fb_result.GetBufferPointer());
    BOOST_CHECK(!fb->error());
    BOOST_REQUIRE(fb->table() != nullptr);
    BOOST_REQUIRE(fb->table()->urban_shares() != nullptr);

    const auto *urban_shares = fb->table()->urban_shares();
    const auto number_of_coordinates = params.coordinates.size();
    BOOST_REQUIRE_EQUAL(urban_shares->size(), number_of_coordinates * number_of_coordinates);
    for (std::size_t row = 0; row < number_of_coordinates; ++row)
    {
        for (std::size_t column = 0; column < number_of_coordinates; ++column)
        {
            const auto share = urban_shares->Get(row * number_of_coordinates + column);
            if (row == column)
            {
                // -1 is the flatbuffers null sentinel; 0 is a legitimate rural share
                BOOST_CHECK_EQUAL(share, -1.f);
            }
            else
            {
                BOOST_CHECK(share >= 0.f);
                BOOST_CHECK(share <= 1.f);
            }
        }
    }
}

BOOST_AUTO_TEST_SUITE_END()
