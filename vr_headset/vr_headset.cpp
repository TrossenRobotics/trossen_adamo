#include "adamo/adamo.hpp"
#include "trossen_vr/trossen_vr.hpp"

#include "trossen_adamo/args.hpp"
#include "trossen_adamo/handshake.hpp"
#include "trossen_adamo/loop.hpp"
#include "trossen_adamo/publisher.hpp"
#include "trossen_adamo/signal.hpp"
#include "trossen_adamo/topics.hpp"
#include "trossen_adamo/wire.hpp"

#include <iostream>
#include <chrono>
#include <stdexcept>


namespace {

// Built-in defaults. Override order: CLI flag > environment variable > default.
constexpr const char* kDefaultRobot     = "vr_headset";

struct Options {
    std::string api_key;
    std::string robot;        // resolved to kDefaultRobot if neither CLI nor env set
    std::string protocol_str = "quic";
    double teleoperation_time = 20.0;
    double connection_timeout = 20.0;
    double ready_timeout = 60.0;
    double rate_hz = 100.0;
    double velocity_limit = 5.0;
    double stall_log_ms = 50.0;
    bool clear_error = false;
};

void usage(const char* prog) {
    std::fprintf(stderr,
        "Usage: %s [options]\n"
        "\n"
        "Required:\n"
        "  --api-key KEY               (or ADAMO_API_KEY)\n"
        "\n"
        "Defaults (override via env or flag):\n"
        "  --robot NAME                (default: vr_headset;          env ADAMO_ROBOT_NAME)\n"
        "\n"
        "Options:\n"
        "  --protocol quic|udp|tcp     (default: quic)\n"
        "  --teleoperation-time SEC    (default: 20)\n"
        "  --connect-timeout SEC       (default: 20)\n"
        "  --ready-timeout SEC         (default: 60)\n"
        "  --rate-hz HZ                (default: 100)\n"
        "  --velocity-limit RAD/S      (default: 5.0)\n"
        "  --stall-log-ms MS           (default: 50)\n"
        "  --clear-error               clear arm fault on connect\n",
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
        else if (a == "--connect-timeout")     o.connection_timeout = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--ready-timeout")       o.ready_timeout = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--rate-hz")             o.rate_hz = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--velocity-limit")      o.velocity_limit = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--stall-log-ms")        o.stall_log_ms = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--clear-error")         o.clear_error = true;
        else if (a == "--help" || a == "-h")   { usage(argv[0]); std::exit(0); }
        else throw std::runtime_error("unknown option: " + a);    
    }
    o.api_key   = ta::require_cli_or_env("--api-key",   "ADAMO_API_KEY", o.api_key);
    o.robot     = ta::cli_env_or_default(o.robot,     "ADAMO_ROBOT_NAME",        kDefaultRobot);
    if (o.rate_hz <= 0.0) throw std::runtime_error("--rate-hz must be positive");
    if (o.velocity_limit <= 0.0) throw std::runtime_error("--velocity-limit must be positive");
    return o;
}

} // namespace

int main(int argc, char** argv) try {
    namespace ta = trossen_adamo;
    ta::install_signal_handlers();

    const Options opt = parse(argc, argv);
    const adamo::Protocol protocol = ta::args::parse_protocol(opt.protocol_str);

    // trossen_vr network manager setup
    std::cout << "vr_headset: configuring network manager" << "\n";
    trossen_vr::ReceiverConfig net_config;
    net_config.port = 9000;
    trossen_vr::NetworkManager receiver(net_config);
    receiver.start();

    // Adamo Session
    std::cout << "vr_headset: opening Adamo session (" << opt.protocol_str << ")\n";
    auto session = adamo::Session::open(opt.api_key, protocol);
    const auto state_topic         = ta::topics::state_of(opt.robot);
    const auto vr_headset_ready_topic = ta::topics::vr_headset_ready_of(opt.robot);
    const auto follower_ready_topic= ta::topics::follower_ready_of(opt.robot);

    // follower = subscriber
    // subscribe will create subscriber and return subscriber object
    auto ready_sub = session.subscribe(follower_ready_topic);

    //trossen_vr_state might needed not sure
    // vr_headset = publisher
    // state_pub = pose publisher main
    auto ready_pub = session.publisher(vr_headset_ready_topic, /*priority=*/250,
                                       /*express=*/true, /*reliable=*/false);
    auto state_pub = session.publisher(state_topic, /*priority=*/250,
                                       /*express=*/true, /*reliable=*/false);

    ta::LatestPublisher state_latest(std::move(state_pub));

    // make last follower string logically 
    ta::handshake::wait_for_peer_ready(ready_pub, ready_sub, opt.ready_timeout, "follower");

    std::cout << "vr_headset: starting teleop\n";
    std::this_thread::sleep_for(std::chrono::seconds(1));

    const double teleop_started_at = ta::wire::now_seconds();
    const auto loop_end = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(opt.teleoperation_time);

    // need to init package that we will be sending

    trossen_vr::ConnectionStatus last_status = trossen_vr::ConnectionStatus::Disconnected;

    while (!ta::stop_requested() && std::chrono::steady_clock::now() < loop_end) {
        const auto loop_start = std::chrono::steady_clock::now();

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

        // get the frame from the vr_headset
        auto frame_opt = receiver.latest_frame();
        
        if (!frame_opt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        const auto& frame = *frame_opt;

        // convert frame to payload
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

    std::cout << "vr_headset: returning home + sleep\n";
    return 0;
} 
catch (const std::exception& e) {
    std::fprintf(stderr, "vr_headset error: %s\n", e.what());
    return 1;
}