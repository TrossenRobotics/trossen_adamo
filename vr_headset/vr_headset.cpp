// VR headset receiver binary.
//
// Receives VR controller data from Meta Quest headset via UDP (port 9000),
// publishes VR frames (poses + buttons + triggers) to Adamo pubsub.
// Acts as the "leader" in VR teleoperation, broadcasting controller state
// for vr_follower to consume and drive robot arms.

#include "adamo/adamo.hpp"
#include "trossen_vr/trossen_vr.hpp"

#include "trossen_adamo/args.hpp"
#include "trossen_adamo/handshake.hpp"
#include "trossen_adamo/loop.hpp"
#include "trossen_adamo/publisher.hpp"
#include "trossen_adamo/signal.hpp"
#include "trossen_adamo/topics.hpp"
#include "trossen_adamo/wire.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

// Built-in defaults. Override order: CLI flag > environment variable > default.
constexpr const char* kDefaultRobot = "vr_teleop";

struct Options {
    std::string api_key;
    std::string robot;        // resolved to kDefaultRobot if neither CLI nor env set
    std::string protocol_str = "quic";
    double teleoperation_time = 20.0;
    double ready_timeout = 60.0;
    double rate_hz = 100.0;
    double stall_log_ms = 50.0;
};

void usage(const char* prog) {
    std::fprintf(stderr,
        "Usage: %s [options]\n"
        "\n"
        "Required:\n"
        "  --api-key KEY               (or ADAMO_API_KEY)\n"
        "\n"
        "Defaults (override via env or flag):\n"
        "  --robot NAME                (default: vr_teleop;     env ADAMO_ROBOT_NAME)\n"
        "\n"
        "Options:\n"
        "  --protocol quic|udp|tcp     (default: quic)\n"
        "  --teleoperation-time SEC    (default: 20)\n"
        "  --ready-timeout SEC         (default: 60)\n"
        "  --rate-hz HZ                (default: 100)\n"
        "  --stall-log-ms MS           (default: 50)\n",
    prog);
}

Options parse(int argc, char** argv) {
    namespace ta = trossen_adamo::args;
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if      (a == "--api-key")             o.api_key = ta::require_value(a, argc, argv, i);
        else if (a == "--robot")               o.robot = ta::require_value(a, argc, argv, i);
        else if (a == "--protocol")            o.protocol_str = ta::require_value(a, argc, argv, i);
        else if (a == "--teleoperation-time")  o.teleoperation_time = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--ready-timeout")       o.ready_timeout = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--rate-hz")             o.rate_hz = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--stall-log-ms")        o.stall_log_ms = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--help" || a == "-h")   { usage(argv[0]); std::exit(0); }
        else throw std::runtime_error("unknown option: " + a);    
    }
    o.api_key   = ta::require_cli_or_env("--api-key",   "ADAMO_API_KEY", o.api_key);
    o.robot     = ta::cli_env_or_default(o.robot,     "ADAMO_ROBOT_NAME",        kDefaultRobot);
    if (o.rate_hz <= 0.0) throw std::runtime_error("--rate-hz must be positive");
    return o;
}

}  // namespace

int main(int argc, char** argv) try {
    namespace ta = trossen_adamo;
    ta::install_signal_handlers();

    const Options opt = parse(argc, argv);
    const adamo::Protocol protocol = ta::args::parse_protocol(opt.protocol_str);

    // trossen_vr NetworkManager receives UDP packets from Meta Quest on port 9000.
    std::cout << "vr_headset: configuring network manager\n";
    trossen_vr::ReceiverConfig net_config;
    net_config.port = 9000;
    trossen_vr::NetworkManager receiver(net_config);
    receiver.start();

    // Open Adamo session and set up pubsub topics.
    std::cout << "vr_headset: opening Adamo session (" << opt.protocol_str << ")\n";
    auto session = adamo::Session::open(opt.api_key, protocol);
    const auto state_topic            = ta::topics::vr_state_of(opt.robot);
    const auto vr_headset_ready_topic = ta::topics::vr_headset_ready_of(opt.robot);
    const auto follower_ready_topic   = ta::topics::follower_ready_of(opt.robot);

    // Follower readiness subscription (vr_follower will publish when ready).
    auto ready_sub = session.subscribe(follower_ready_topic);

    // Headset publishes VR frames to state_topic and readiness signal for handshake.
    auto ready_pub = session.publisher(vr_headset_ready_topic, /*priority=*/250,
                                       /*express=*/true, /*reliable=*/false);
    auto state_pub = session.publisher(state_topic, /*priority=*/250,
                                       /*express=*/true, /*reliable=*/false);

    ta::LatestPublisher state_latest(std::move(state_pub));

    // Wait for vr_follower to be ready before starting teleop.
    ta::handshake::wait_for_peer_ready(ready_pub, ready_sub, opt.ready_timeout, "follower");

    std::cout << "vr_headset: starting teleop\n";
    std::cout << "vr_headset: broadcasting VR controller data to vr_follower\n";
    std::this_thread::sleep_for(std::chrono::seconds(1));

    const auto loop_end = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(opt.teleoperation_time);

    // Track connection status changes for logging.
    trossen_vr::ConnectionStatus last_status = trossen_vr::ConnectionStatus::Disconnected;

    while (!ta::stop_requested() && std::chrono::steady_clock::now() < loop_end) {
        const auto loop_start = std::chrono::steady_clock::now();

        // Log connection status changes (Connecting/Connected/Degraded/Disconnected).
        auto current_status = receiver.get_connection_status();
        if (current_status != last_status) {
            switch (current_status) {
                case trossen_vr::ConnectionStatus::Connecting:
                    std::cout << "Connecting..." << std::endl;
                    break;
                case trossen_vr::ConnectionStatus::Connected:
                    std::cout << "Connection established (";
                    std::cout << receiver.get_message_frequency() << " Hz)" << std::endl;
                    break;
                case trossen_vr::ConnectionStatus::Degraded:
                    std::cout << "Connection degraded (low frequency: ";
                    std::cout << receiver.get_message_frequency() << " Hz)" << std::endl;
                    break;
                case trossen_vr::ConnectionStatus::Disconnected:
                    std::cout << "Connection lost (timeout)" << std::endl;
                    break;
            }
            last_status = current_status;
        }

        // Fetch latest VR frame from NetworkManager.
        auto frame_opt = receiver.latest_frame();
        
        if (!frame_opt) {
            ta::sleep_until_next_tick(loop_start, opt.rate_hz);
            continue;
        }
        const auto& frame = *frame_opt;

        // Encode VR frame to wire format and publish to Adamo.
        const auto payload = ta::wire::encode_vr_frame(ta::wire::now_seconds(), frame);
        state_latest.put(payload.data(), payload.size());

        const auto elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - loop_start).count();
        if (elapsed_ms > opt.stall_log_ms) {
            std::fprintf(stderr, "vr_headset loop stall: %.1fms\n", elapsed_ms);
        }
        ta::sleep_until_next_tick(loop_start, opt.rate_hz);
    }

    state_latest.close();

    std::cout << "vr_headset: done\n";
    return 0;
} 
catch (const std::exception& e) {
    std::fprintf(stderr, "vr_headset error: %s\n", e.what());
    return 1;
}
