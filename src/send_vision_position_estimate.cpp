#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mocap/mocap.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/shell/shell.h>

#include <spdlog/spdlog.h>


#include <iostream>
#include <chrono>
#include <future>
#include <memory>
#include <thread>

using namespace mavsdk;
using std::chrono::seconds;
using std::chrono::milliseconds;

void usage(const std::string& bin_name)
{
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect to the simulator use URL: udp://:14540\n";
}

void run_interactive_shell(std::shared_ptr<System> system);

std::shared_ptr<System> get_system(Mavsdk& mavsdk)
{
    std::cout << "Waiting to discover system...\n";
    auto prom = std::promise<std::shared_ptr<System>>{};
    auto fut = prom.get_future();

    // We wait for new systems to be discovered, once we find one that has an
    // autopilot, we decide to use it.
    mavsdk.subscribe_on_new_system([&mavsdk, &prom]() {
        auto system = mavsdk.systems().back();

        if (system->has_autopilot()) {
            std::cout << "Discovered autopilot\n";

            // Unsubscribe again as we only want to find one system.
            mavsdk.subscribe_on_new_system(nullptr);
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

void run_fake_odometry_send(std::shared_ptr<System> system)
{
    Mocap mocap{system}; 
    spdlog::info("Start fake odometry sending");
    uint64_t count = 0;
    while (true) {
        Mocap::VisionPositionEstimate zero{};
        zero.time_usec = std::chrono::steady_clock::now().time_since_epoch().count();
        auto ret = mocap.set_vision_position_estimate(zero);
        
        if (ret == Mocap::Result::NoSystem)
            spdlog::warn("no system connected");
        else if (ret == Mocap::Result::Success)
            spdlog::info("mocap sent success");
        else
            spdlog::warn("mocap send other error {}", ret);
        count++;
        if (count % 100 == 0)
            std::cout << "vision position estimate: " << zero << std::endl;
        std::this_thread::sleep_for(milliseconds(100));
    }
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

    telemetry.subscribe_odometry(
        [system] (Telemetry::Odometry odometry_data) {
            uint64_t time_usec = odometry_data.time_usec - system->get_timesync_offset_ns() / 1e3;
            spdlog::info("{} odometry received at host: {} {} {} ", time_usec, 
                odometry_data.position_body.x_m, odometry_data.position_body.y_m, odometry_data.position_body.z_m);
    });

    std::thread t_odometry_send(run_fake_odometry_send, system);
    std::thread t_shell(run_interactive_shell, system);

    while (true) {
        std::this_thread::sleep_for(seconds(10));
        spdlog::info("system steady time now {} ms", std::chrono::steady_clock::now().time_since_epoch().count() / 1e6);
        spdlog::info("current time offset estimated: {} ms", system->get_timesync_offset_ns() / 1e6);
    }

    return 0;
}

void run_interactive_shell(std::shared_ptr<System> system)
{
    Shell shell{system};

    shell.subscribe_receive([](const std::string output) { std::cout << output; });

    while (true) {
        std::string command;
        getline(std::cin, command);

        if (command == "exit") {
            break;
        }

        shell.send(command);
    }
    std::cout << '\n';
}
