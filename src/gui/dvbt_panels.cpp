#include "../decode_report.hpp"
#include "../decoder_diagnostics.hpp"
#include "airspy_tv/debug.hpp"
#include "app_state.hpp"
#include "panels.hpp"
#include "widgets.hpp"

#include <imgui.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <format>
#include <iostream>
#include <optional>
#include <span>
#include <string>

namespace airspy_tv::gui {

using dvbt::CodeRate;
using dvbt::Constellation;
using dvbt::GuardInterval;
using dvbt::TransmissionMode;

void initialize_standard_state(AppState &state) {
    if (state.session.standard() == ReceiveStandard::DvbT) {
        state.session.set_dvbt_telemetry_enabled(
            is_debug_enabled(), std::chrono::steady_clock::now());
        state.session.set_dvbt_parameters(state.dvbt.parameters);
    }
}

void update_standard_state(AppState &state) {
    if (state.session.standard() != ReceiveStandard::DvbT) {
        return;
    }
    const auto snapshot = state.session.dvbt_snapshot();
    state.dvbt.signal = snapshot.signal;
    state.dvbt.decoder = snapshot.decoder;
    if (is_debug_enabled()) {
        for (const auto &record : state.session.drain_dvbt_telemetry()) {
            format_debug_telemetry(std::cerr, record);
        }
    }

    static auto last_diagnostics = std::chrono::steady_clock::now();
    const auto now = std::chrono::steady_clock::now();
    if (is_debug_enabled() &&
        now - last_diagnostics >= std::chrono::seconds(10)) {
        last_diagnostics = now;
        dump_decoder_diagnostics(state.dvbt.decoder);
    }
}

void draw_standard_settings_panel(AppState &state) {
    if (state.session.standard() != ReceiveStandard::DvbT) {
        draw_disabled_wrapped(
            "Settings for the selected demodulator will appear here once "
            "that standard is implemented.");
        return;
    }

    ImGui::SeparatorText("DVB-T settings");
    auto decoder_threads =
        static_cast<std::uint32_t>(state.dvbt.parameters.worker_threads);
    ImGui::BeginDisabled(state.session.is_open());
    ImGui::TextUnformatted("Decoder worker budget");
    ImGui::SetNextItemWidth(-1.0F);
    if (ImGui::InputScalar("##decoder-worker-budget", ImGuiDataType_U32,
                           &decoder_threads)) {
        decoder_threads = std::min(decoder_threads, std::uint32_t{256});
        state.dvbt.parameters.worker_threads = decoder_threads;
        state.session.set_dvbt_parameters(state.dvbt.parameters);
    }
    ImGui::EndDisabled();
    ImGui::TextDisabled("0 = Auto (%zu logical CPUs)",
                        airspy_tv::dvbt::default_viterbi_worker_count());
    const std::size_t active_workers =
        state.dvbt.decoder.resample_workers +
        state.dvbt.decoder.symbol_workers +
        state.dvbt.decoder.transport.viterbi_workers;
    if (active_workers != 0) {
        ImGui::TextDisabled("Workers: %zu resample, %zu symbol, %zu FEC",
                            state.dvbt.decoder.resample_workers,
                            state.dvbt.decoder.symbol_workers,
                            state.dvbt.decoder.transport.viterbi_workers);
    }

    bool parameters_changed = false;
    const auto draw_optional_combo =
        [&parameters_changed](const char *label,
                              const std::span<const char *const> names,
                              int &selected) {
            ImGui::TextUnformatted(label);
            ImGui::PushID(label);
            ImGui::SetNextItemWidth(-1.0F);
            if (ImGui::BeginCombo("##value",
                                  names[static_cast<std::size_t>(selected)])) {
                for (std::size_t index = 0; index < names.size(); ++index) {
                    if (ImGui::Selectable(names[index],
                                          selected ==
                                              static_cast<int>(index))) {
                        selected = static_cast<int>(index);
                        parameters_changed = true;
                    }
                }
                ImGui::EndCombo();
            }
            ImGui::PopID();
        };

    int bandwidth = 1;
    switch (state.dvbt.parameters.channel_bandwidth_hz) {
    case 5'000'000:
        bandwidth = 0;
        break;
    case 7'000'000:
        bandwidth = 2;
        break;
    case 8'000'000:
        bandwidth = 3;
        break;
    default:
        bandwidth = 1;
        break;
    }
    constexpr std::array bandwidth_names{"5 MHz", "6 MHz", "7 MHz", "8 MHz"};
    constexpr std::array<std::uint32_t, 4> bandwidth_values{
        5'000'000, 6'000'000, 7'000'000, 8'000'000};
    draw_optional_combo("Channel bandwidth", bandwidth_names, bandwidth);
    state.dvbt.parameters.channel_bandwidth_hz =
        bandwidth_values[static_cast<std::size_t>(bandwidth)];

    int selected_mode =
        !state.dvbt.parameters.mode.has_value()
            ? 0
            : (*state.dvbt.parameters.mode == TransmissionMode::k2 ? 1 : 2);
    constexpr std::array mode_names{"Auto", "2K", "8K"};
    draw_optional_combo("Transmission mode", mode_names, selected_mode);
    state.dvbt.parameters.mode =
        selected_mode == 0
            ? std::nullopt
            : std::optional{selected_mode == 1 ? TransmissionMode::k2
                                               : TransmissionMode::k8};

    int guard = 0;
    if (state.dvbt.parameters.guard_interval.has_value()) {
        switch (*state.dvbt.parameters.guard_interval) {
        case GuardInterval::gi_1_32:
            guard = 1;
            break;
        case GuardInterval::gi_1_16:
            guard = 2;
            break;
        case GuardInterval::gi_1_8:
            guard = 3;
            break;
        case GuardInterval::gi_1_4:
            guard = 4;
            break;
        }
    }
    constexpr std::array guard_names{"Auto", "1/32", "1/16", "1/8", "1/4"};
    draw_optional_combo("Guard interval", guard_names, guard);
    constexpr std::array guard_values{
        GuardInterval::gi_1_32, GuardInterval::gi_1_16, GuardInterval::gi_1_8,
        GuardInterval::gi_1_4};
    state.dvbt.parameters.guard_interval =
        guard == 0
            ? std::nullopt
            : std::optional{guard_values[static_cast<std::size_t>(guard - 1)]};

    int modulation = 0;
    if (state.dvbt.parameters.constellation.has_value()) {
        switch (*state.dvbt.parameters.constellation) {
        case Constellation::qpsk:
            modulation = 1;
            break;
        case Constellation::qam16:
            modulation = 2;
            break;
        case Constellation::qam64:
            modulation = 3;
            break;
        }
    }
    constexpr std::array modulation_names{"Auto (TPS)", "QPSK", "16-QAM",
                                          "64-QAM"};
    draw_optional_combo("Modulation", modulation_names, modulation);
    constexpr std::array modulation_values{
        Constellation::qpsk, Constellation::qam16, Constellation::qam64};
    state.dvbt.parameters.constellation =
        modulation == 0
            ? std::nullopt
            : std::optional{
                  modulation_values[static_cast<std::size_t>(modulation - 1)]};

    int code_rate = 0;
    if (state.dvbt.parameters.code_rate.has_value()) {
        switch (*state.dvbt.parameters.code_rate) {
        case CodeRate::rate_1_2:
            code_rate = 1;
            break;
        case CodeRate::rate_2_3:
            code_rate = 2;
            break;
        case CodeRate::rate_3_4:
            code_rate = 3;
            break;
        case CodeRate::rate_5_6:
            code_rate = 4;
            break;
        case CodeRate::rate_7_8:
            code_rate = 5;
            break;
        }
    }
    constexpr std::array code_rate_names{"Auto (TPS)", "1/2", "2/3",
                                         "3/4",        "5/6", "7/8"};
    draw_optional_combo("Code rate", code_rate_names, code_rate);
    constexpr std::array code_rate_values{
        CodeRate::rate_1_2, CodeRate::rate_2_3, CodeRate::rate_3_4,
        CodeRate::rate_5_6, CodeRate::rate_7_8};
    state.dvbt.parameters.code_rate =
        code_rate == 0
            ? std::nullopt
            : std::optional{
                  code_rate_values[static_cast<std::size_t>(code_rate - 1)]};

    if (parameters_changed) {
        // Publish the new input bandwidth before resetting the demodulator so
        // blocks arriving during the reset are tagged with the same rate as
        // the new decoder configuration. set_parameters() is synchronous,
        // but live input must remain free to continue feeding the source.
        state.session.set_dvbt_parameters(state.dvbt.parameters);
        state.status = "DVB-T parameters updated; receiver reacquiring";
    }
}

static void draw_dvbt_diagnostics_panel(AppState &state) {
    if (ImGui::CollapsingHeader("DVB-T FEC & timing",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushID("dvbt-diagnostics-panel");
        const bool locked = state.dvbt.signal.locked;
        const ImVec4 lock_colour = locked ? ImVec4(0.35F, 0.88F, 0.55F, 1.0F)
                                          : ImVec4(1.0F, 0.38F, 0.25F, 1.0F);
        draw_status_indicator(locked ? "OFDM MONITOR LOCKED"
                                     : "OFDM MONITOR UNLOCKED",
                              lock_colour);

        const bool transport_locked =
            state.dvbt.decoder.transport.rs_synchronized &&
            state.dvbt.decoder.transport.ts_packets != 0;
        const auto &transport = state.dvbt.decoder.transport;
        const bool pre_viterbi_available =
            transport.pre_viterbi_compared_bits != 0;
        const double pre_viterbi_ber =
            pre_viterbi_available
                ? static_cast<double>(transport.pre_viterbi_error_bits) /
                      static_cast<double>(transport.pre_viterbi_compared_bits)
                : 0.0;
        const bool decoder_active = state.dvbt.decoder.processing ||
                                    state.dvbt.decoder.ofdm_locked ||
                                    state.dvbt.decoder.input_blocks != 0;
        const ImVec4 decoder_colour =
            state.dvbt.decoder.failed ? ImVec4(1.0F, 0.38F, 0.25F, 1.0F)
            : transport_locked
                ? ImVec4(0.35F, 0.88F, 0.55F, 1.0F)
                : (decoder_active ? ImVec4(1.0F, 0.72F, 0.22F, 1.0F)
                                  : ImVec4(0.55F, 0.62F, 0.70F, 1.0F));
        draw_status_indicator(
            state.dvbt.decoder.failed ? "TS DECODER FAILED"
            : transport_locked
                ? "TS DECODER LOCKED"
                : (decoder_active ? "TS DECODER ACQUIRING" : "TS DECODER IDLE"),
            decoder_colour);
        if (state.dvbt.decoder.failed) {
            ImGui::PushStyleColor(ImGuiCol_Text,
                                  ImVec4(1.0F, 0.45F, 0.35F, 1.0F));
            ImGui::TextWrapped("%s", state.dvbt.decoder.error.c_str());
            ImGui::PopStyleColor();
        }

        // Continual-carrier fade indicator: the GUI diag dump already logs
        // it every 10 s, but a real-time bar helps correlate a drift/stall
        // with a marginal channel before consulting logs.
        {
            const float fi = state.dvbt.decoder.fade_indicator;
            const std::string fi_text = std::format("{:.3f}", fi);
            draw_metric("Fade ind", fi_text.c_str(), std::clamp(fi, 0.0F, 1.0F),
                        ImVec4(0.62F, 0.92F, 0.45F, 1.0F));
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
                ImGui::SetTooltip(
                    "Continual-carrier normalized correlation; ~1 = healthy, "
                    "<= 0.25 freeze, 0.25-0.5 marginal grid.\n"
                    "If it stays high while the grid drifts, the alias was "
                    "latched before the correlation collapsed.");
            }
        }
        {
            const bool timing_valid =
                state.dvbt.decoder.ofdm_locked &&
                state.dvbt.decoder.timing_confidence > 0.0F;
            const bool sro_resampler_ready =
                state.dvbt.decoder.sro_resampler_ready;
            const std::string actuator_text =
                sro_resampler_ready
                    ? std::format(
                          "{:+.3f} / {:+.3f} ppm / {}",
                          state.dvbt.decoder.sro_resampler_command_ppm,
                          state.dvbt.decoder.sro_resampler_applied_ppm,
                          timing_valid
                              ? std::format(
                                    "{:.1f} smp",
                                    state.dvbt.decoder.timing_offset_samples)
                              : "-- smp")
                    : std::format(
                          "-- / -- ppm / {}",
                          timing_valid
                              ? std::format(
                                    "{:.1f} smp",
                                    state.dvbt.decoder.timing_offset_samples)
                              : "-- smp");
            draw_bipolar_metric(
                "SRO cmd / applied", actuator_text.c_str(),
                sro_resampler_ready
                    ? std::clamp(
                          0.5F + state.dvbt.decoder.sro_resampler_applied_ppm /
                                     10.0F,
                          0.0F, 1.0F)
                    : 0.5F,
                ImVec4(0.52F, 0.82F, 1.0F, 1.0F));
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
                const double scheduled_delay_ms =
                    state.dvbt.decoder.sro_input_sample_rate_hz != 0
                        ? 1000.0 *
                              static_cast<double>(
                                  state.dvbt.decoder.sro_fixed_delay_samples) /
                              state.dvbt.decoder.sro_input_sample_rate_hz
                        : 0.0;
                ImGui::SetTooltip(
                    "DVB-T timing-loop command, correction currently applied "
                    "by the common variable-rate resampler, and filtered "
                    "pilot-slope timing offset. The bar uses applied ppm "
                    "only.\n"
                    "Measured SRO %+.4f ppm, scheduled delay %.3f ms, late "
                    "%llu samples, pending %zu.\n"
                    "Command input %llu, target %llu, applied %llu.\n"
                    "Requested ratio %.12f, effective ratio %.12f.",
                    state.dvbt.decoder.sample_clock_offset_ppm,
                    scheduled_delay_ms,
                    static_cast<unsigned long long>(
                        state.dvbt.decoder.sro_schedule_late_samples),
                    state.dvbt.decoder.sro_pending_commands,
                    static_cast<unsigned long long>(
                        state.dvbt.decoder.sro_command_input_sample),
                    static_cast<unsigned long long>(
                        state.dvbt.decoder.sro_effective_input_sample),
                    static_cast<unsigned long long>(
                        state.dvbt.decoder.sro_applied_input_sample),
                    state.dvbt.decoder.resampler_requested_ratio,
                    state.dvbt.decoder.resampler_effective_ratio);
            }
            const std::string confidence_text = std::format(
                "{:.1f}%%", state.dvbt.decoder.timing_confidence * 100.0F);
            draw_metric("Timing conf", confidence_text.c_str(),
                        state.dvbt.decoder.timing_confidence,
                        ImVec4(0.35F, 0.88F, 0.55F, 1.0F));
            const bool cfo_resampler_ready =
                state.dvbt.decoder.cfo_resampler_ready;
            const std::string cfo_text =
                cfo_resampler_ready
                    ? std::format("{:+.1f} / {:+.1f} / {:+.2f} Hz",
                                  state.dvbt.decoder.acquisition_cfo_hz,
                                  state.dvbt.decoder.cfo_resampler_applied_hz,
                                  state.dvbt.decoder.residual_carrier_offset_hz)
                    : "acquiring";
            const float cfo_position =
                cfo_resampler_ready &&
                        state.signal.carrier_offset_limit_hz > 0.0F
                    ? std::clamp(
                          0.5F +
                              state.dvbt.decoder.cfo_resampler_applied_hz /
                                  (2.0F * state.signal.carrier_offset_limit_hz),
                          0.0F, 1.0F)
                    : 0.5F;
            draw_bipolar_metric("CFO acq / applied", cfo_text.c_str(),
                                cfo_position, ImVec4(0.62F, 0.78F, 1.0F, 1.0F));
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
                const double scheduled_delay_ms =
                    state.dvbt.decoder.cfo_input_sample_rate_hz != 0
                        ? 1000.0 *
                              static_cast<double>(
                                  state.dvbt.decoder.cfo_fixed_delay_samples) /
                              state.dvbt.decoder.cfo_input_sample_rate_hz
                        : 0.0;
                ImGui::SetTooltip(
                    "Initial CFO is acquired before production samples enter "
                    "the ring; steady-state residual CFO is fed back to the "
                    "same common resampler independently of SRO. The third "
                    "displayed value is residual CFO; the bar uses applied "
                    "CFO only.\n"
                    "Acquisition fractional %+.2f Hz, integer bins %+d.\n"
                    "Estimated %+.2f Hz, residual %+.2f Hz, command %+.2f Hz.\n"
                    "Bootstrap attempts %llu, replayed %llu samples, retained "
                    "peak %llu samples.\n"
                    "CFO rebootstrap %llu/%llu, last trigger %+.2f Hz at "
                    "source/output %llu/%llu.\n"
                    "Scheduled delay %.3f ms, late %llu samples, pending %zu.\n"
                    "Latest command input %llu, target %llu. Last applied "
                    "target %llu, actual %llu.",
                    state.dvbt.decoder.acquisition_fractional_cfo_hz,
                    state.dvbt.decoder.acquisition_carrier_bin_offset,
                    state.dvbt.decoder.tracked_carrier_offset_hz,
                    state.dvbt.decoder.residual_carrier_offset_hz,
                    state.dvbt.decoder.cfo_resampler_command_hz,
                    static_cast<unsigned long long>(
                        state.dvbt.decoder.bootstrap_attempts),
                    static_cast<unsigned long long>(
                        state.dvbt.decoder.bootstrap_replayed_input_samples),
                    static_cast<unsigned long long>(
                        state.dvbt.decoder.bootstrap_retained_peak_samples),
                    static_cast<unsigned long long>(
                        state.dvbt.decoder.cfo_rebootstrap_count),
                    static_cast<unsigned long long>(
                        state.dvbt.decoder.cfo_rebootstrap_requests),
                    state.dvbt.decoder.cfo_rebootstrap_last_residual_hz,
                    static_cast<unsigned long long>(
                        state.dvbt.decoder.cfo_rebootstrap_source_sample),
                    static_cast<unsigned long long>(
                        state.dvbt.decoder.cfo_rebootstrap_output_sample),
                    scheduled_delay_ms,
                    static_cast<unsigned long long>(
                        state.dvbt.decoder.cfo_schedule_late_samples),
                    state.dvbt.decoder.cfo_pending_commands,
                    static_cast<unsigned long long>(
                        state.dvbt.decoder.cfo_command_input_sample),
                    static_cast<unsigned long long>(
                        state.dvbt.decoder.cfo_effective_input_sample),
                    static_cast<unsigned long long>(
                        state.dvbt.decoder.cfo_applied_effective_input_sample),
                    static_cast<unsigned long long>(
                        state.dvbt.decoder.cfo_applied_input_sample));
            }
        }
        const auto ber_quality = [](const double ber) {
            return ber == 0.0
                       ? 1.0F
                       : std::clamp(static_cast<float>(-std::log10(ber) / 7.0),
                                    0.0F, 1.0F);
        };
        const std::string pre_viterbi_text =
            pre_viterbi_available ? std::format("{:.2e}", pre_viterbi_ber)
                                  : "--";
        draw_metric("Inner BER", pre_viterbi_text.c_str(),
                    pre_viterbi_available ? ber_quality(pre_viterbi_ber) : 0.0F,
                    ImVec4(0.75F, 0.72F, 0.30F, 1.0F));
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
            ImGui::SetTooltip(
                "Bit errors before the outer FEC, measured against the "
                "received mother-code metrics. For DVB-T this is the "
                "pre-Viterbi BER (re-encoded survivor path).\n%llu errors / "
                "%llu compared bits",
                static_cast<unsigned long long>(
                    transport.pre_viterbi_error_bits),
                static_cast<unsigned long long>(
                    transport.pre_viterbi_compared_bits));
        }

        const bool post_viterbi_available =
            transport.post_viterbi_compared_bits != 0;
        const double post_viterbi_ber =
            post_viterbi_available
                ? static_cast<double>(transport.post_viterbi_error_bits) /
                      static_cast<double>(transport.post_viterbi_compared_bits)
                : 0.0;
        const std::string post_viterbi_text =
            post_viterbi_available ? std::format("{:.2e}", post_viterbi_ber)
                                   : "--";
        draw_metric("Outer BER", post_viterbi_text.c_str(),
                    post_viterbi_available ? ber_quality(post_viterbi_ber)
                                           : 0.0F,
                    ImVec4(0.35F, 0.88F, 0.55F, 1.0F));
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
            ImGui::SetTooltip(
                "RS-derived outer error estimate: corrected payload bits "
                "from valid codewords plus a full 188-byte penalty for each "
                "uncorrectable codeword. This keeps the metric live during "
                "outer-lock loss.\n%llu error bits / %llu attempted bits; "
                "%llu RS failures",
                static_cast<unsigned long long>(
                    transport.post_viterbi_error_bits),
                static_cast<unsigned long long>(
                    transport.post_viterbi_compared_bits),
                static_cast<unsigned long long>(
                    transport.rs_uncorrectable_packets));
        }
        ImGui::TextColored(
            transport.rs_uncorrectable_packets == 0
                ? ImVec4(0.45F, 0.82F, 0.62F, 1.0F)
                : ImVec4(0.95F, 0.55F, 0.28F, 1.0F),
            "RS failures %llu    TEI output %llu",
            static_cast<unsigned long long>(transport.rs_uncorrectable_packets),
            static_cast<unsigned long long>(transport.tei_packets));
        ImGui::Separator();
        const char *mode = "--";
        const char *guard = "--";
        const char *modulation = "--";
        const char *code_rate = "--";
        if (state.dvbt.parameters.code_rate.has_value()) {
            switch (*state.dvbt.parameters.code_rate) {
            case airspy_tv::dvbt::CodeRate::rate_1_2:
                code_rate = "1/2 (manual)";
                break;
            case airspy_tv::dvbt::CodeRate::rate_2_3:
                code_rate = "2/3 (manual)";
                break;
            case airspy_tv::dvbt::CodeRate::rate_3_4:
                code_rate = "3/4 (manual)";
                break;
            case airspy_tv::dvbt::CodeRate::rate_5_6:
                code_rate = "5/6 (manual)";
                break;
            case airspy_tv::dvbt::CodeRate::rate_7_8:
                code_rate = "7/8 (manual)";
                break;
            }
        }
        if (state.dvbt.signal.locked) {
            mode =
                state.dvbt.signal.mode == airspy_tv::dvbt::TransmissionMode::k8
                    ? "8K"
                    : "2K";
            switch (state.dvbt.signal.guard_interval) {
            case airspy_tv::dvbt::GuardInterval::gi_1_32:
                guard = "1/32";
                break;
            case airspy_tv::dvbt::GuardInterval::gi_1_16:
                guard = "1/16";
                break;
            case airspy_tv::dvbt::GuardInterval::gi_1_8:
                guard = "1/8";
                break;
            case airspy_tv::dvbt::GuardInterval::gi_1_4:
                guard = "1/4";
                break;
            }
            switch (state.dvbt.signal.constellation) {
            case airspy_tv::dvbt::Constellation::qpsk:
                modulation = "QPSK";
                break;
            case airspy_tv::dvbt::Constellation::qam16:
                modulation = "16-QAM";
                break;
            case airspy_tv::dvbt::Constellation::qam64:
                modulation = "64-QAM";
                break;
            }
        }
        if (state.dvbt.decoder.tps_locked) {
            mode = state.dvbt.decoder.tps_mode == TransmissionMode::k8
                       ? "8K (TPS)"
                       : "2K (TPS)";
            switch (state.dvbt.decoder.tps_guard_interval) {
            case GuardInterval::gi_1_32:
                guard = "1/32 (TPS)";
                break;
            case GuardInterval::gi_1_16:
                guard = "1/16 (TPS)";
                break;
            case GuardInterval::gi_1_8:
                guard = "1/8 (TPS)";
                break;
            case GuardInterval::gi_1_4:
                guard = "1/4 (TPS)";
                break;
            }
            switch (state.dvbt.decoder.tps_constellation) {
            case Constellation::qpsk:
                modulation = "QPSK (TPS)";
                break;
            case Constellation::qam16:
                modulation = "16-QAM (TPS)";
                break;
            case Constellation::qam64:
                modulation = "64-QAM (TPS)";
                break;
            }
            if (!state.dvbt.parameters.code_rate.has_value()) {
                switch (state.dvbt.decoder.tps_code_rate) {
                case CodeRate::rate_1_2:
                    code_rate = "1/2 (TPS)";
                    break;
                case CodeRate::rate_2_3:
                    code_rate = "2/3 (TPS)";
                    break;
                case CodeRate::rate_3_4:
                    code_rate = "3/4 (TPS)";
                    break;
                case CodeRate::rate_5_6:
                    code_rate = "5/6 (TPS)";
                    break;
                case CodeRate::rate_7_8:
                    code_rate = "7/8 (TPS)";
                    break;
                }
            }
        }
        ImGui::Text("Bandwidth      %u MHz",
                    state.dvbt.parameters.channel_bandwidth_hz / 1'000'000U);
        ImGui::Text("Mode           %s", mode);
        ImGui::Text("Guard          %s", guard);
        ImGui::Text("Modulation     %s", modulation);
        ImGui::Text("Code rate      %s", code_rate);
        draw_disabled_wrapped(
            state.dvbt.decoder.tps_locked
                ? "TPS synchronization and BCH are valid; auto decoder "
                  "parameters come from this TPS frame."
            : state.dvbt.signal.locked
                ? "OFDM monitor is locked; waiting for a valid TPS frame."
                : "RF estimates use the selected channel bandwidth and "
                  "out-of-channel noise; MER and constellation require OFDM "
                  "lock.");
        ImGui::PopID();
    }
}

void draw_standard_diagnostics_panel(AppState &state) {
    if (state.session.standard() == ReceiveStandard::DvbT) {
        draw_dvbt_diagnostics_panel(state);
    }
}

} // namespace airspy_tv::gui
