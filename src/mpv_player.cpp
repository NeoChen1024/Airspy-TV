#include "airspy_tv/mpv_player.hpp"

#define GL_GLEXT_PROTOTYPES 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_opengl.h>
#include <mpv/client.h>
#include <mpv/render.h>
#include <mpv/render_gl.h>
#include <mpv/stream_cb.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <format>
#include <limits>
#include <memory>
#include <mutex>
#include <new>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace airspy_tv {
namespace {

constexpr std::size_t transport_packet_size = 188;
constexpr std::size_t playback_queue_capacity = 8U << 20U;
constexpr std::size_t playback_queue_low_watermark = 1U << 20U;
constexpr std::size_t playback_queue_resume_watermark = 2U << 20U;

[[nodiscard]] std::string mpv_error(const std::string_view operation,
                                    const int code) {
    return std::format("{}: {} ({})", operation, mpv_error_string(code), code);
}

[[nodiscard]] void *get_proc_address(void *unused, const char *name) {
    static_cast<void>(unused);
    return reinterpret_cast<void *>(SDL_GL_GetProcAddress(name));
}

} // namespace

struct MpvPlayer::Impl {
    struct StreamCookie {
        Impl *owner{};
        std::uint64_t generation{};
        std::atomic<bool> canceled;
    };

    mpv_handle *handle{};
    mpv_render_context *render_context{};
    GLuint texture{};
    GLuint framebuffer{};
    int texture_width{};
    int texture_height{};

    mutable std::mutex mutex;
    std::condition_variable data_ready;
    std::deque<std::vector<std::uint8_t>> queue;
    std::size_t front_offset{};
    std::size_t queued_bytes{};
    std::uint64_t blocks_accepted{};
    std::uint64_t bytes_accepted{};
    std::uint64_t blocks_processed{};
    std::uint64_t bytes_processed{};
    std::uint64_t dropped_blocks{};
    std::uint64_t dropped_bytes{};
    // Do not let libmpv consume a short tail immediately after startup or a
    // dropout.  The hysteresis avoids repeatedly entering/leaving buffering
    // at the same threshold while preserving a bounded live latency.
    bool buffering{true};
    std::uint64_t generation{};
    bool source_active{};
    std::optional<TransportService> selected_service;
    // Stream-level seams reported by the decoder (fec_region_reset,
    // stream_end, retune). Monotonic; read by telemetry().
    std::uint64_t discontinuity_count{};

    bool file_loaded{};
    float current_volume{100.0F};
    bool current_muted{};
    std::string message{"Player idle"};

    // Signature is fixed by libmpv; the URI buffer is not modified.
    // NOLINTNEXTLINE(readability-non-const-parameter)
    static int open_stream(void *user_data, char *uri,
                           mpv_stream_cb_info *info) {
        static_cast<void>(uri);
        auto *self = static_cast<Impl *>(user_data);
        // libmpv owns this opaque cookie until close_stream().
        // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
        auto *cookie = new (std::nothrow) StreamCookie;
        if (cookie == nullptr) {
            return MPV_ERROR_NOMEM;
        }
        {
            const std::scoped_lock lock(self->mutex);
            cookie->owner = self;
            cookie->generation = self->generation;
        }
        info->cookie = cookie;
        info->read_fn = &read_stream;
        info->seek_fn = nullptr;
        info->size_fn = nullptr;
        info->close_fn = &close_stream;
        info->cancel_fn = &cancel_stream;
        return 0;
    }

    static std::int64_t read_stream(void *opaque, char *output,
                                    const std::uint64_t requested) {
        auto *cookie = static_cast<StreamCookie *>(opaque);
        Impl &self = *cookie->owner;
        std::unique_lock lock(self.mutex);
        self.data_ready.wait(lock, [&] {
            return cookie->canceled.load() || !self.source_active ||
                   cookie->generation != self.generation ||
                   (!self.buffering && !self.queue.empty());
        });
        if (cookie->canceled.load() || !self.source_active ||
            cookie->generation != self.generation) {
            return 0;
        }

        const auto maximum = static_cast<std::size_t>(std::min<std::uint64_t>(
            requested, static_cast<std::uint64_t>(
                           std::numeric_limits<std::size_t>::max())));
        std::size_t copied = 0;
        while (copied < maximum && !self.queue.empty()) {
            const auto &block = self.queue.front();
            const std::size_t count =
                std::min(maximum - copied, block.size() - self.front_offset);
            std::memcpy(output + copied, block.data() + self.front_offset,
                        count);
            copied += count;
            self.front_offset += count;
            self.queued_bytes -= count;
            if (self.front_offset == block.size()) {
                self.queue.pop_front();
                self.front_offset = 0;
                ++self.blocks_processed;
            }
        }
        self.bytes_processed += copied;
        if (self.queued_bytes <= playback_queue_low_watermark) {
            self.buffering = true;
        }
        return static_cast<std::int64_t>(copied);
    }

    static void cancel_stream(void *opaque) {
        auto *cookie = static_cast<StreamCookie *>(opaque);
        cookie->canceled = true;
        cookie->owner->data_ready.notify_all();
    }

    static void close_stream(void *opaque) {
        // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
        delete static_cast<StreamCookie *>(opaque);
    }

    void clear_queue_locked() {
        dropped_blocks += queue.size();
        dropped_bytes += queued_bytes;
        queue.clear();
        front_offset = 0;
        queued_bytes = 0;
        buffering = true;
    }

    void restart_playback() {
        if (handle == nullptr) {
            return;
        }
        bool active = false;
        {
            const std::scoped_lock lock(mutex);
            ++generation;
            clear_queue_locked();
            file_loaded = false;
            active = source_active;
            message = active ? "Waiting for MPEG-TS" : "Player idle";
        }
        data_ready.notify_all();
        if (active) {
            std::array command{"loadfile", "airspytv://live", "replace",
                               static_cast<const char *>(nullptr)};
            const int result = mpv_command_async(handle, 0, command.data());
            if (result < 0) {
                const std::scoped_lock lock(mutex);
                message = mpv_error("mpv loadfile", result);
            }
        } else {
            std::array command{"stop", static_cast<const char *>(nullptr)};
            static_cast<void>(mpv_command_async(handle, 0, command.data()));
        }
    }

    [[nodiscard]] bool
    packet_selected(const std::span<const std::uint8_t> packet) const {
        if (!selected_service.has_value()) {
            return true;
        }
        const auto pid =
            static_cast<std::uint16_t>(((packet[1] & 0x1FU) << 8U) | packet[2]);
        if (pid == 0 || pid == 1 || pid == 0x11 || pid == 0x12 || pid == 0x14 ||
            pid == selected_service->pmt_pid ||
            pid == selected_service->pcr_pid) {
            return true;
        }
        return std::ranges::any_of(
            selected_service->components,
            [pid](const TransportStreamComponent &component) {
                return component.pid == pid;
            });
    }

    void destroy_gl_objects() {
        if (framebuffer != 0) {
            glDeleteFramebuffers(1, &framebuffer);
            framebuffer = 0;
        }
        if (texture != 0) {
            glDeleteTextures(1, &texture);
            texture = 0;
        }
        texture_width = 0;
        texture_height = 0;
    }

    bool ensure_target(const int width, const int height) {
        if (texture != 0 && texture_width == width &&
            texture_height == height) {
            return true;
        }
        destroy_gl_objects();
        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA,
                     GL_UNSIGNED_BYTE, nullptr);
        glGenFramebuffers(1, &framebuffer);
        glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                               GL_TEXTURE_2D, texture, 0);
        const bool complete =
            glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glBindTexture(GL_TEXTURE_2D, 0);
        if (!complete) {
            destroy_gl_objects();
            return false;
        }
        texture_width = width;
        texture_height = height;
        return true;
    }
};

MpvPlayer::MpvPlayer() : impl_(std::make_unique<Impl>()) {}

MpvPlayer::~MpvPlayer() noexcept { shutdown(); }

bool MpvPlayer::initialize(std::string &error) {
    if (impl_->handle != nullptr) {
        return true;
    }
    impl_->handle = mpv_create();
    if (impl_->handle == nullptr) {
        error = "mpv_create failed";
        return false;
    }
    const auto set_option = [this, &error](const char *name,
                                           const char *value) {
        const int result = mpv_set_option_string(impl_->handle, name, value);
        if (result >= 0) {
            return true;
        }
        error = mpv_error(std::format("mpv option {}", name), result);
        return false;
    };
    if (!set_option("config", "no") || !set_option("terminal", "no") ||
        !set_option("vo", "libmpv") || !set_option("cache", "no") ||
        !set_option("keep-open", "yes") ||
        !set_option("demuxer-lavf-format", "mpegts")) {
        shutdown();
        return false;
    }
    int result = mpv_initialize(impl_->handle);
    if (result < 0) {
        error = mpv_error("mpv_initialize", result);
        shutdown();
        return false;
    }
    result = mpv_stream_cb_add_ro(impl_->handle, "airspytv", impl_.get(),
                                  &Impl::open_stream);
    if (result < 0) {
        error = mpv_error("mpv_stream_cb_add_ro", result);
        shutdown();
        return false;
    }

    mpv_opengl_init_params gl_init{.get_proc_address = &get_proc_address,
                                   .get_proc_address_ctx = nullptr};
    const char *api = MPV_RENDER_API_TYPE_OPENGL;
    std::array<mpv_render_param, 3> parameters{{
        {MPV_RENDER_PARAM_API_TYPE, const_cast<char *>(api)},
        {MPV_RENDER_PARAM_OPENGL_INIT_PARAMS, &gl_init},
        {MPV_RENDER_PARAM_INVALID, nullptr},
    }};
    result = mpv_render_context_create(&impl_->render_context, impl_->handle,
                                       parameters.data());
    if (result < 0) {
        error = mpv_error("mpv_render_context_create", result);
        shutdown();
        return false;
    }
    set_volume(impl_->current_volume);
    set_muted(impl_->current_muted);
    return true;
}

void MpvPlayer::shutdown() {
    if (impl_ == nullptr) {
        return;
    }
    {
        const std::scoped_lock lock(impl_->mutex);
        impl_->source_active = false;
        ++impl_->generation;
        impl_->clear_queue_locked();
    }
    impl_->data_ready.notify_all();
    if (impl_->render_context != nullptr) {
        mpv_render_context_set_update_callback(impl_->render_context, nullptr,
                                               nullptr);
        mpv_render_context_free(impl_->render_context);
        impl_->render_context = nullptr;
    }
    impl_->destroy_gl_objects();
    if (impl_->handle != nullptr) {
        mpv_terminate_destroy(impl_->handle);
        impl_->handle = nullptr;
    }
}

void MpvPlayer::set_source_active(const bool active) {
    {
        const std::scoped_lock lock(impl_->mutex);
        if (impl_->source_active == active) {
            return;
        }
        impl_->source_active = active;
    }
    impl_->restart_playback();
}

void MpvPlayer::submit(const std::span<const std::uint8_t> transport_stream) {
    if (transport_stream.size() < transport_packet_size) {
        return;
    }
    std::vector<std::uint8_t> filtered;
    filtered.reserve(transport_stream.size());
    {
        const std::scoped_lock lock(impl_->mutex);
        if (!impl_->source_active) {
            return;
        }
        for (std::size_t offset = 0;
             offset + transport_packet_size <= transport_stream.size();
             offset += transport_packet_size) {
            const auto packet =
                transport_stream.subspan(offset, transport_packet_size);
            if (packet.front() == 0x47U && impl_->packet_selected(packet)) {
                filtered.insert(filtered.end(), packet.begin(), packet.end());
            }
        }
        if (filtered.empty()) {
            return;
        }
        while (!impl_->queue.empty() && impl_->queued_bytes + filtered.size() >
                                            playback_queue_capacity) {
            const std::size_t dropped =
                impl_->queue.front().size() - impl_->front_offset;
            ++impl_->dropped_blocks;
            impl_->dropped_bytes += dropped;
            impl_->queued_bytes -= dropped;
            impl_->queue.pop_front();
            impl_->front_offset = 0;
        }
        if (filtered.size() > playback_queue_capacity) {
            const std::size_t keep =
                playback_queue_capacity -
                (playback_queue_capacity % transport_packet_size);
            ++impl_->dropped_blocks;
            impl_->dropped_bytes += filtered.size() - keep;
            filtered.erase(filtered.begin(),
                           filtered.end() - static_cast<std::ptrdiff_t>(keep));
        }
        ++impl_->blocks_accepted;
        impl_->bytes_accepted += filtered.size();
        impl_->queued_bytes += filtered.size();
        impl_->queue.push_back(std::move(filtered));
        if (impl_->buffering &&
            impl_->queued_bytes >= playback_queue_resume_watermark) {
            impl_->buffering = false;
        }
    }
    impl_->data_ready.notify_all();
}

void MpvPlayer::on_discontinuity(const TransportDiscontinuity discontinuity) {
    switch (discontinuity) {
    case TransportDiscontinuity::fec_region_reset:
        // A gated region was dropped and the FEC re-seeded: the demuxer sees
        // a continuity jump and error-conceals it. The stream continues — no
        // restart, no queue drop (the pre-seam bytes are still valid).
        {
            const std::scoped_lock lock(impl_->mutex);
            ++impl_->discontinuity_count;
        }
        break;
    case TransportDiscontinuity::stream_end:
        // The input ended: the queued tail is still valid and plays out; the
        // next read returns EOF and the demuxer ends cleanly instead of
        // grinding through data that no longer has a source.
        {
            const std::scoped_lock lock(impl_->mutex);
            ++impl_->discontinuity_count;
        }
        break;
    case TransportDiscontinuity::retune:
        // The content may have changed entirely (retune, source switch, or
        // dropped-block recovery): restart so the demuxer re-parses the new
        // channel's PAT/PMT instead of concatenating two unrelated streams
        // (which the per-frame source-active toggle alone misses — a live
        // retune keeps streaming).
        {
            const std::scoped_lock lock(impl_->mutex);
            ++impl_->discontinuity_count;
        }
        impl_->restart_playback();
        break;
    }
}

void MpvPlayer::select_service(const TransportService &service) {
    {
        const std::scoped_lock lock(impl_->mutex);
        if (impl_->selected_service.has_value() &&
            impl_->selected_service->service_id == service.service_id &&
            impl_->selected_service->pmt_pid == service.pmt_pid &&
            impl_->selected_service->pcr_pid == service.pcr_pid &&
            impl_->selected_service->components == service.components) {
            return;
        }
        impl_->selected_service = service;
    }
    impl_->restart_playback();
}

void MpvPlayer::clear_service() {
    {
        const std::scoped_lock lock(impl_->mutex);
        if (!impl_->selected_service.has_value()) {
            return;
        }
        impl_->selected_service.reset();
    }
    impl_->restart_playback();
}

void MpvPlayer::set_volume(const float volume) {
    impl_->current_volume = std::clamp(volume, 0.0F, 100.0F);
    if (impl_->handle != nullptr) {
        double value = impl_->current_volume;
        static_cast<void>(mpv_set_property(impl_->handle, "volume",
                                           MPV_FORMAT_DOUBLE, &value));
    }
}

void MpvPlayer::set_muted(const bool muted) {
    impl_->current_muted = muted;
    if (impl_->handle != nullptr) {
        int value = muted ? 1 : 0;
        static_cast<void>(
            mpv_set_property(impl_->handle, "mute", MPV_FORMAT_FLAG, &value));
    }
}

float MpvPlayer::volume() const { return impl_->current_volume; }

bool MpvPlayer::muted() const { return impl_->current_muted; }

void MpvPlayer::poll_events() {
    if (impl_->handle == nullptr) {
        return;
    }
    while (true) {
        const mpv_event *event = mpv_wait_event(impl_->handle, 0.0);
        if (event == nullptr || event->event_id == MPV_EVENT_NONE) {
            break;
        }
        const std::scoped_lock lock(impl_->mutex);
        if (event->event_id == MPV_EVENT_FILE_LOADED) {
            impl_->file_loaded = true;
            impl_->message = "Playing decoded MPEG-TS";
        } else if (event->event_id == MPV_EVENT_END_FILE) {
            impl_->file_loaded = false;
            const auto *end =
                static_cast<const mpv_event_end_file *>(event->data);
            if (end != nullptr && end->error < 0) {
                impl_->message = mpv_error("Playback", end->error);
            }
        }
    }
}

bool MpvPlayer::ready() const {
    const std::scoped_lock lock(impl_->mutex);
    return impl_->file_loaded;
}

PlaybackTelemetry MpvPlayer::telemetry() const {
    PlaybackTelemetry result;
    {
        const std::scoped_lock lock(impl_->mutex);
        result.queued_bytes = impl_->queued_bytes;
        result.queue_capacity = playback_queue_capacity;
        result.active = impl_->source_active;
        result.blocks_accepted = impl_->blocks_accepted;
        result.bytes_accepted = impl_->bytes_accepted;
        result.blocks_processed = impl_->blocks_processed;
        result.bytes_processed = impl_->bytes_processed;
        result.dropped_blocks = impl_->dropped_blocks;
        result.dropped_bytes = impl_->dropped_bytes;
        result.buffering = impl_->source_active && impl_->buffering;
        result.discontinuities = impl_->discontinuity_count;
    }
    if (impl_->handle == nullptr) {
        return result;
    }
    double value = 0.0;
    if (mpv_get_property(impl_->handle, "playback-time", MPV_FORMAT_DOUBLE,
                         &value) >= 0) {
        result.playback_time_s = value;
    }
    value = 0.0;
    if (mpv_get_property(impl_->handle, "avsync", MPV_FORMAT_DOUBLE, &value) >=
        0) {
        // mpv reports the A/V difference in seconds; positive = audio ahead.
        result.avsync_ms = value * 1e3;
    }
    std::int64_t count = 0;
    if (mpv_get_property(impl_->handle, "frame-drop-count", MPV_FORMAT_INT64,
                         &count) >= 0) {
        result.dropped_frames = count;
    }
    count = 0;
    if (mpv_get_property(impl_->handle, "vo-drop-frame-count", MPV_FORMAT_INT64,
                         &count) >= 0) {
        result.vo_dropped_frames = count;
    }
    int flag = 0;
    if (mpv_get_property(impl_->handle, "pause", MPV_FORMAT_FLAG, &flag) >= 0) {
        result.paused = flag != 0;
    }
    return result;
}

std::string MpvPlayer::status() const {
    const std::scoped_lock lock(impl_->mutex);
    return impl_->message;
}

std::uint32_t MpvPlayer::render(const int width, const int height) {
    if (impl_->render_context == nullptr || width <= 0 || height <= 0 ||
        !impl_->ensure_target(width, height)) {
        return 0;
    }
    static_cast<void>(mpv_render_context_update(impl_->render_context));
    GLint previous_framebuffer = 0;
    std::array<GLint, 4> previous_viewport{};
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &previous_framebuffer);
    glGetIntegerv(GL_VIEWPORT, previous_viewport.data());
    mpv_opengl_fbo target{.fbo = static_cast<int>(impl_->framebuffer),
                          .w = width,
                          .h = height,
                          .internal_format = GL_RGBA8};
    // The video target is an FBO-backed texture, not OpenGL's default
    // framebuffer. libmpv therefore renders in normal texture coordinates;
    // ImGui samples that texture directly.
    int flip_y = 0;
    int block_for_target_time = 0;
    std::array<mpv_render_param, 4> parameters{{
        {MPV_RENDER_PARAM_OPENGL_FBO, &target},
        {MPV_RENDER_PARAM_FLIP_Y, &flip_y},
        {MPV_RENDER_PARAM_BLOCK_FOR_TARGET_TIME, &block_for_target_time},
        {MPV_RENDER_PARAM_INVALID, nullptr},
    }};
    static_cast<void>(
        mpv_render_context_render(impl_->render_context, parameters.data()));
    glBindFramebuffer(GL_FRAMEBUFFER,
                      static_cast<GLuint>(previous_framebuffer));
    glViewport(previous_viewport[0], previous_viewport[1], previous_viewport[2],
               previous_viewport[3]);
    return impl_->texture;
}

} // namespace airspy_tv
