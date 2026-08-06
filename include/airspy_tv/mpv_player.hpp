#pragma once

#include "airspy_tv/transport_stream.hpp"

#include <cstdint>
#include <memory>
#include <span>
#include <string>

namespace airspy_tv {

// Live playback telemetry sampled from libmpv and the player's own TS queue.
// All fields are monotonic or point-in-time snapshots safe to read from the
// UI thread while the decoder feeds the player.
struct PlaybackTelemetry {
    double playback_time_s{};
    // libmpv's audio-vs-video presentation offset in milliseconds; positive
    // means audio is ahead of video.
    double avsync_ms{};
    std::int64_t dropped_frames{};
    std::int64_t vo_dropped_frames{};
    std::size_t queued_bytes{};
    std::size_t queue_capacity{};
    // True while the player's own queue is below its recovery watermark.
    // Playback starts/resumes at 2 MiB and re-enters buffering at 1 MiB.
    bool buffering{};
    // Stream-level seams received from the decoder (fec_region_reset,
    // stream_end, retune) — distinct from per-packet TEI corruption.
    std::uint64_t discontinuities{};
    bool paused{};
};

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
    // Controlled recovery for the stream-level seams reported by the decoder
    // (see TransportDiscontinuity): retunes restart the demuxer, stream ends
    // play out the queued tail and hit EOF, FEC-region resets are left to the
    // demuxer's error concealment.
    void on_discontinuity(TransportDiscontinuity discontinuity);
    void select_service(const TransportService &service);
    void clear_service();

    void set_volume(float volume);
    void set_muted(bool muted);
    [[nodiscard]] float volume() const;
    [[nodiscard]] bool muted() const;

    void poll_events();
    [[nodiscard]] bool ready() const;
    [[nodiscard]] std::string status() const;
    [[nodiscard]] PlaybackTelemetry telemetry() const;

    // Must be called with the OpenGL context used by initialize() current.
    // Returns the OpenGL texture containing the most recent video frame.
    [[nodiscard]] std::uint32_t render(int width, int height);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace airspy_tv
