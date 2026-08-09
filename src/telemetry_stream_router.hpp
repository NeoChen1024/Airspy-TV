#pragma once

#include <nlohmann/json_fwd.hpp>

#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace airspy_tv {

struct TelemetryStreamSpec {
    std::string key;
    std::filesystem::path path;
    std::string record_type;
    int schema_version{};
};

class TelemetryStreamRouter {
  public:
    TelemetryStreamRouter(std::filesystem::path directory,
                          std::vector<TelemetryStreamSpec> streams);
    ~TelemetryStreamRouter() noexcept;

    TelemetryStreamRouter(const TelemetryStreamRouter &) = delete;
    TelemetryStreamRouter &operator=(const TelemetryStreamRouter &) = delete;

    void write(const std::string &key, const nlohmann::json &record);
    void write_batch(const std::string &key,
                     std::span<const nlohmann::json> records);
    void flush();

    [[nodiscard]] const std::vector<TelemetryStreamSpec> &specs() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace airspy_tv
