#pragma once

#include "airspy_tv/transport_stream.hpp"

#include <cstdint>
#include <memory>
#include <span>
#include <string>

namespace airspy_tv {

class MpvPlayer {
  public:
    MpvPlayer();
    ~MpvPlayer() noexcept;

    MpvPlayer(const MpvPlayer &) = delete;
    MpvPlayer &operator=(const MpvPlayer &) = delete;
    MpvPlayer(MpvPlayer &&) = delete;
    MpvPlayer &operator=(MpvPlayer &&) = delete;

    bool initialize(std::string &error);
    void shutdown();

    void set_source_active(bool active);
    void submit(std::span<const std::uint8_t> transport_stream);
    void select_service(const TransportService &service);
    void clear_service();

    void set_volume(float volume);
    void set_muted(bool muted);
    [[nodiscard]] float volume() const;
    [[nodiscard]] bool muted() const;

    void poll_events();
    [[nodiscard]] bool ready() const;
    [[nodiscard]] std::string status() const;

    // Must be called with the OpenGL context used by initialize() current.
    // Returns the OpenGL texture containing the most recent video frame.
    [[nodiscard]] std::uint32_t render(int width, int height);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace airspy_tv
