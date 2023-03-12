#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <spdlog/spdlog.h>


#include <iostream>
#include <chrono>
#include <future>
#include <memory>
#include <thread>

using namespace mavsdk;
using std::chrono::seconds;

void usage(const std::string& bin_name)
{
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect to the simulator use URL: udp://:14540\n";
}

std::shared_ptr<System> get_system(Mavsdk& mavsdk)
{
    std::cout << "Waiting to discover system...\n";
    auto prom = std::promise<std::shared_ptr<System>>{};
    auto fut = prom.get_future();

    // We wait for new systems to be discovered, once we find one that has an
    // autopilot, we decide to use it.
    Mavsdk::NewSystemHandle handle = mavsdk.subscribe_on_new_system([&mavsdk, &prom, &handle]() {
        auto system = mavsdk.systems().back();

        if (system->has_autopilot()) {
            std::cout << "Discovered autopilot\n";

            // Unsubscribe again as we only want to find one system.
            mavsdk.unsubscribe_on_new_system(handle);
            prom.set_value(system);
        }
    });

    // We usually receive heartbeats at 1Hz, therefore we should find a
    // system after around 3 seconds max, surely.
    if (fut.wait_for(seconds(3)) == std::future_status::timeout) {
        std::cerr << "No autopilot found.\n";
        return {};
    }

    // Get discovered system now.
    return fut.get();
}

int main(int argc, char** argv)
{
    if (argc != 2) {
        usage(argv[0]);
        return 1;
    }

    Mavsdk mavsdk;
    ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    auto system = get_system(mavsdk);
    if (!system) {
        return 1;
    }

    system->enable_timesync();

    // Instantiate plugins.
    auto telemetry = Telemetry{system};

    telemetry.subscribe_position_velocity_ned(
        [] (Telemetry::PositionVelocityNed local_position) {

            spdlog::info("local position ned: {}, {}, {}", local_position.position.north_m, 
            local_position.position.east_m, local_position.position.down_m);

        }
    );

    telemetry.subscribe_scaled_imu(
        [system] (Telemetry::Imu imu_data) {
            uint64_t timestamp_us =  imu_data.timestamp_us - system->get_timesync_offset_ns() / 1e3;
            spdlog::info("imu scaled raw_ts = {} ms,  host_ts = {} ms", imu_data.timestamp_us / 1e3, timestamp_us / 1e3);
    });

    telemetry.subscribe_imu(
        [system] (Telemetry::Imu imu_data) {
            uint64_t timestamp_us =  imu_data.timestamp_us - system->get_timesync_offset_ns() / 1e3;
            spdlog::info("imu highres raw_ts = {} ms,  host_ts = {} ms", imu_data.timestamp_us / 1e3, timestamp_us / 1e3);
        }
    );

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        spdlog::info("system steady time now {} ms", std::chrono::steady_clock::now().time_since_epoch().count() / 1e6);
        spdlog::info("current time offset estimated: {} ms", system->get_timesync_offset_ns() / 1e6);
    }

    

    return 0;
}