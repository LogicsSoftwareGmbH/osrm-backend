#ifndef OSRM_CONTRACTOR_FILES_HPP
#define OSRM_CONTRACTOR_FILES_HPP

#include "contractor/serialization.hpp"

#include "extractor/urban_classes.hpp"

#include <unordered_map>

namespace osrm::contractor::files
{
// reads .osrm.urban
inline void readUrbanData(const std::filesystem::path &path,
                          const std::string &metric_name,
                          std::vector<EdgeDistance> &urban_meters,
                          extractor::UrbanClassWeights &class_weights)
{
    const auto fingerprint = storage::tar::FileReader::VerifyFingerprint;
    storage::tar::FileReader reader{path, fingerprint};

    std::vector<float> weights;
    storage::serialization::read(reader, "/common/urban_class_weights", weights);
    if (weights.size() != class_weights.size())
    {
        throw util::exception("Unexpected urban_class_weights size in " + path.string());
    }
    std::copy(weights.begin(), weights.end(), class_weights.begin());

    storage::serialization::read(
        reader, "/ch/metrics/" + metric_name + "/urban_meters", urban_meters);
}

// writes .osrm.urban: the per-edge urban_meters vector positionally parallel
// to the metric's contracted edge array, plus the class-weight LUT needed at
// query time for phantom-node seeding
inline void writeUrbanData(const std::filesystem::path &path,
                           const std::string &metric_name,
                           const std::vector<EdgeDistance> &urban_meters,
                           const extractor::UrbanClassWeights &class_weights)
{
    const auto fingerprint = storage::tar::FileWriter::GenerateFingerprint;
    storage::tar::FileWriter writer{path, fingerprint};

    const std::vector<float> weights(class_weights.begin(), class_weights.end());
    storage::serialization::write(writer, "/common/urban_class_weights", weights);

    storage::serialization::write(
        writer, "/ch/metrics/" + metric_name + "/urban_meters", urban_meters);
}
// reads .osrm.hsgr file
template <typename ContractedMetricT>
inline void readGraph(const std::filesystem::path &path,
                      std::unordered_map<std::string, ContractedMetricT> &metrics,
                      std::uint32_t &connectivity_checksum)
{
    static_assert(std::is_same<ContractedMetric, ContractedMetricT>::value ||
                      std::is_same<ContractedMetricView, ContractedMetricT>::value,
                  "metric must be of type ContractedMetric<>");

    const auto fingerprint = storage::tar::FileReader::VerifyFingerprint;
    storage::tar::FileReader reader{path, fingerprint};

    reader.ReadInto("/ch/connectivity_checksum", connectivity_checksum);

    for (auto &pair : metrics)
    {
        serialization::read(reader, "/ch/metrics/" + pair.first, pair.second);
    }
}

// writes .osrm.hsgr file
template <typename ContractedMetricT>
inline void writeGraph(const std::filesystem::path &path,
                       const std::unordered_map<std::string, ContractedMetricT> &metrics,
                       const std::uint32_t connectivity_checksum)
{
    static_assert(std::is_same<ContractedMetric, ContractedMetricT>::value ||
                      std::is_same<ContractedMetricView, ContractedMetricT>::value,
                  "metric must be of type ContractedMetric<>");
    const auto fingerprint = storage::tar::FileWriter::GenerateFingerprint;
    storage::tar::FileWriter writer{path, fingerprint};

    writer.WriteElementCount64("/ch/connectivity_checksum", 1);
    writer.WriteFrom("/ch/connectivity_checksum", connectivity_checksum);

    for (const auto &pair : metrics)
    {
        serialization::write(writer, "/ch/metrics/" + pair.first, pair.second);
    }
}
} // namespace osrm::contractor::files

#endif
