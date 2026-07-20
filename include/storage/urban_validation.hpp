#ifndef OSRM_STORAGE_URBAN_VALIDATION_HPP
#define OSRM_STORAGE_URBAN_VALIDATION_HPP

#include "extractor/files.hpp"
#include "extractor/urban_classes.hpp"

#include "storage/serialization.hpp"
#include "storage/tar.hpp"

#include "util/exception.hpp"
#include "util/log.hpp"

#include <cstdint>
#include <filesystem>
#include <iterator>
#include <optional>
#include <string>
#include <vector>

namespace osrm::storage
{

// The urban side-car files live next to the stock-format files but are only
// meaningful relative to the exact preprocessing run that produced their
// neighbours: the .osrm.urban_config LUT is indexed by the class-name -> bit
// mapping of one specific extract, and .osrm.urban is positionally parallel to
// one specific .osrm.hsgr edge array. A partial re-preprocessing by a tool
// that knows nothing about the side-cars (e.g. a stock osrm-extract or
// osrm-contract) can leave stale copies behind, so nothing here is trusted on
// existence alone: every load path validates both files and degrades to
// "no urban data" (with a warning) instead of serving plausible but wrong
// shares.

// Returns the config identity when .osrm.urban_config provably belongs to the
// extract products next to it — its recorded identity must equal the identity
// recomputed from the class-name mapping in .osrm.properties plus its own
// weights — and std::nullopt otherwise. Absence of the file is silent; any
// mismatch or read failure warns.
inline std::optional<std::uint32_t>
urbanConfigIdentityIfValid(const std::filesystem::path &config_path,
                           const std::filesystem::path &properties_path)
{
    if (!std::filesystem::exists(config_path))
    {
        return std::nullopt;
    }
    try
    {
        if (!std::filesystem::exists(properties_path))
        {
            throw util::exception("no " + properties_path.string() + " to match it against");
        }

        extractor::UrbanClassWeights weights{};
        std::uint32_t recorded_identity = 0;
        extractor::files::readUrbanConfig(config_path, weights, recorded_identity);

        extractor::ProfileProperties properties;
        extractor::files::readProfileProperties(properties_path, properties);

        const auto expected = extractor::computeUrbanConfigIdentity(properties, weights);
        if (recorded_identity != expected)
        {
            throw util::exception("config identity " + std::to_string(recorded_identity) +
                                  " does not match " + std::to_string(expected) +
                                  " recomputed from " + properties_path.string());
        }
        return recorded_identity;
    }
    catch (const std::exception &e)
    {
        util::Log(logWARNING) << "Ignoring stale urban config " << config_path << ": " << e.what()
                              << ". Re-run osrm-extract to regenerate it.";
        return std::nullopt;
    }
}

// Returns true iff .osrm.urban provably belongs to the preprocessing products
// next to it: the connectivity checksum and per-metric edge counts it recorded
// must match .osrm.hsgr, its embedded LUT must carry the identity of the class
// mapping in .osrm.properties, and — when a valid .osrm.urban_config is
// present (pass its identity) — the payload must have been derived from
// exactly that config: a weight-only re-extract without a re-contract changes
// neither the graph nor the counts, but it does change the identity.
inline bool urbanSideCarMatchesGraph(const std::filesystem::path &urban_path,
                                     const std::filesystem::path &hsgr_path,
                                     const std::filesystem::path &properties_path,
                                     const std::optional<std::uint32_t> active_config_identity)
{
    if (!std::filesystem::exists(urban_path))
    {
        return false;
    }
    try
    {
        if (!std::filesystem::exists(hsgr_path))
        {
            throw util::exception("no " + hsgr_path.string() + " to match it against");
        }
        if (!std::filesystem::exists(properties_path))
        {
            throw util::exception("no " + properties_path.string() + " to match it against");
        }

        tar::FileReader urban_reader(urban_path, tar::FileReader::VerifyFingerprint);
        tar::FileReader hsgr_reader(hsgr_path, tar::FileReader::VerifyFingerprint);

        std::uint32_t urban_checksum = 0;
        std::uint32_t graph_checksum = 0;
        urban_reader.ReadInto("/urban/connectivity_checksum", urban_checksum);
        hsgr_reader.ReadInto("/ch/connectivity_checksum", graph_checksum);
        if (urban_checksum != graph_checksum)
        {
            throw util::exception("connectivity checksum " + std::to_string(urban_checksum) +
                                  " does not match " + std::to_string(graph_checksum) + " in " +
                                  hsgr_path.string());
        }

        std::vector<tar::FileReader::FileEntry> entries;
        urban_reader.List(std::back_inserter(entries));
        const std::string prefix = "/ch/metrics/";
        const std::string suffix = "/urban_meters";
        for (const auto &entry : entries)
        {
            if (entry.name.size() > prefix.size() + suffix.size() &&
                entry.name.compare(0, prefix.size(), prefix) == 0 &&
                entry.name.compare(entry.name.size() - suffix.size(), suffix.size(), suffix) == 0)
            {
                const auto metric = entry.name.substr(
                    prefix.size(), entry.name.size() - prefix.size() - suffix.size());
                const auto urban_count = urban_reader.ReadElementCount64(entry.name);
                const auto edge_count = hsgr_reader.ReadElementCount64(
                    prefix + metric + "/contracted_graph/edge_array");
                if (urban_count != edge_count)
                {
                    throw util::exception("metric " + metric + " has " +
                                          std::to_string(urban_count) +
                                          " urban_meters entries but the graph has " +
                                          std::to_string(edge_count) + " edges");
                }
            }
        }

        std::uint32_t recorded_identity = 0;
        urban_reader.ReadInto("/urban/config_identity", recorded_identity);

        std::vector<float> embedded;
        serialization::read(urban_reader, "/common/urban_class_weights", embedded);
        extractor::UrbanClassWeights embedded_weights{};
        if (embedded.size() != embedded_weights.size())
        {
            throw util::exception("unexpected urban_class_weights size " +
                                  std::to_string(embedded.size()));
        }
        std::copy(embedded.begin(), embedded.end(), embedded_weights.begin());

        extractor::ProfileProperties properties;
        extractor::files::readProfileProperties(properties_path, properties);
        const auto expected = extractor::computeUrbanConfigIdentity(properties, embedded_weights);
        if (recorded_identity != expected)
        {
            throw util::exception("config identity " + std::to_string(recorded_identity) +
                                  " does not match " + std::to_string(expected) +
                                  " recomputed from " + properties_path.string());
        }

        if (active_config_identity && *active_config_identity != recorded_identity)
        {
            throw util::exception("payload was derived from urban weights with identity " +
                                  std::to_string(recorded_identity) +
                                  " but the .osrm.urban_config next to it has " +
                                  std::to_string(*active_config_identity));
        }

        return true;
    }
    catch (const std::exception &e)
    {
        util::Log(logWARNING) << "Ignoring stale urban side-car " << urban_path << ": " << e.what()
                              << ". Re-run osrm-contract to regenerate it.";
        return false;
    }
}

} // namespace osrm::storage

#endif // OSRM_STORAGE_URBAN_VALIDATION_HPP
