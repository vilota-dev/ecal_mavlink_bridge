#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mocap/mocap.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <spdlog/spdlog.h>

#include <ecal/ecal.h>
// #include <ecal/msg/capnproto/helper.h>
#include <ecal/msg/capnproto/subscriber.h>
#include <ecal/msg/capnproto/publisher.h>


#include <capnp/odometry3d.capnp.h>
#include <capnp/mavstate.capnp.h>

#include <iostream>
#include <chrono>
#include <future>
#include <memory>
#include <thread>

using namespace mavsdk;
using std::chrono::seconds;
using std::chrono::milliseconds;

constexpr int AUTOPILOT_HEARTBEAT_TIMEOUT_S = 7;

#define UNUSED(x) (void)(x)

void usage(const std::string& bin_name)
{
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect to the simulator use URL: udp://:14540\n";
}

// void send_heartbeat(MavlinkPassthrough& mavlink_passthrough, uint8_t system_id, bool active)
// {
//     mavlink_message_t message;
//     mavlink_msg_heartbeat_pack(
//         system_id,
//         MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY,
//         &message,
//         MAV_TYPE_ONBOARD_CONTROLLER,
//         MAV_AUTOPILOT_INVALID,
//         0,
//         0,
//         active ? MAV_STATE_ACTIVE : MAV_STATE_UNINIT); //
//     mavlink_passthrough.send_message(message);
// }

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
            spdlog::info ("Discovered autopilot, system id = {}", system->get_system_id());

            auto component_ids = system->component_ids();
            for (size_t i = 0; i < component_ids.size(); i++)
                spdlog::info("component {}: id = {}", i, component_ids[i]);

            // Unsubscribe again as we only want to find one system.
            mavsdk.unsubscribe_on_new_system(handle);
            prom.set_value(system);
        }
    });

    // We usually receive heartbeats at 1Hz, therefore we should find a
    // system after around 3 seconds max, surely.
    if (fut.wait_for(seconds(AUTOPILOT_HEARTBEAT_TIMEOUT_S)) == std::future_status::timeout) {
        std::cerr << "No autopilot found after seconds: "<< AUTOPILOT_HEARTBEAT_TIMEOUT_S << std::endl;
        return {};
    }

    // Get discovered system now.
    return fut.get();
}

class MavlinkOdometrySender {

  public:
    MavlinkOdometrySender(std::shared_ptr<System> system)
    {
        m_mocap = std::make_shared<Mocap>(system);

        // not implementing pose covariance yet
        m_odom_msg.pose_covariance.covariance_matrix.resize(1);
        m_odom_msg.pose_covariance.covariance_matrix[0] = NAN;

        // not implementing velocity covriance yet
        m_odom_msg.velocity_covariance.covariance_matrix.resize(1);
        m_odom_msg.velocity_covariance.covariance_matrix[0] = NAN;

        // hardcode as vision now
        m_odom_msg.mav_estimator = Mocap::Odometry::MavEstimator::Vision;

        count = 0;
    }

    bool Send(uint64_t tns, Mocap::PositionBody& p, Mocap::Quaternion& q)
    {
        m_odom_msg.time_usec = tns / 1e3;
        m_odom_msg.position_body = p;
        m_odom_msg.q = q;

        auto ret = m_mocap->set_odometry(m_odom_msg);

        if (ret == Mocap::Result::NoSystem)
            spdlog::warn("no system connected");
        else if (ret == Mocap::Result::Success)
            spdlog::debug("mocap sent success");
        else
            spdlog::warn("mocap send other error {}", ret);
        
        if (count % 100 == 0)
            std::cout << "mavlink odometry message sent to px4: " << m_odom_msg << std::endl;
        count++;

        return (ret == Mocap::Result::Success);
    }

    void odometry_callback(const char* ecal_topic_name, ecal::Odometry3d::Reader ecal_msg, const long long ecal_ts) {
    
        UNUSED(ecal_topic_name);
        UNUSED(ecal_ts);

        const auto& header = ecal_msg.getHeader();
        auto seq = header.getSeq();
        auto tns = header.getStamp();

        Mocap::PositionBody p;
        auto position = ecal_msg.getPose().getPosition();
        p.x_m = position.getX();
        p.y_m = position.getY();
        p.z_m = position.getZ();

        Mocap::Quaternion q;
        auto orientation = ecal_msg.getPose().getOrientation();
        q.w = orientation.getW();
        q.x = orientation.getX();
        q.y = orientation.getY();
        q.z = orientation.getZ();

        if (Send(tns, p, q)) {
            std::uint64_t nowTns = std::chrono::steady_clock::now().time_since_epoch().count();
            
            spdlog::debug("odometry of seq = {}, ts = {} sent at host ts = {}, latency = {} ms", seq, tns, nowTns, (nowTns - tns) / 1e6);
        }else
            spdlog::warn("failed to send odometry over mavlinke to px4");


    }

  private:
    std::shared_ptr<Mocap> m_mocap;
    Mocap::Odometry m_odom_msg;
    uint64_t count;
};

class MavStateSender {
  public:
    MavStateSender(std::string tf_prefix, int sendIntervalSec = 1)
    {
        m_pubMavState = std::make_shared<eCAL::capnproto::CPublisher<ecal::MavState>>();
        m_pubMavState->Create(tf_prefix + "mav_state");
        m_initialised = false;
        m_seq = 0;

        m_senderThread = std::thread(&MavStateSender::senderThread, this, sendIntervalSec);

        // initial value
        ecal::MavState::Builder msg = m_pubMavState->GetBuilder();
        msg.setModePX4(ecal::MavState::FlightModePX4::UNKNOWN);
    }

    void updateArmStatus(bool armed)
    {
        std::lock_guard<std::mutex> lock(m_mutexMavState);
        ecal::MavState::Builder msg = m_pubMavState->GetBuilder();

        std::uint64_t tns = std::chrono::steady_clock::now().time_since_epoch().count();

        if (msg.getArmed() != armed)
        {
            spdlog::warn("arm status update to {}", armed);
        }

        msg.setArmed(armed);

        if (msg.getHeader().getStamp() < tns)
            msg.getHeader().setStamp(tns);
        else
            spdlog::warn("tns regression on flight mode update, from {} to {}", msg.getHeader().getStamp(), tns);

        m_initialised = true;
    }

    void updateFlightMode(Telemetry::FlightMode mode)
    {
        std::lock_guard<std::mutex> lock(m_mutexMavState);
        ecal::MavState::Builder msg = m_pubMavState->GetBuilder();

        std::uint64_t tns = std::chrono::steady_clock::now().time_since_epoch().count();

        const auto lastMode = msg.getModePX4();

        if (mode == Telemetry::FlightMode::Manual)
            msg.setModePX4(ecal::MavState::FlightModePX4::MANUAL);
        else if (mode == Telemetry::FlightMode::Altctl)
            msg.setModePX4(ecal::MavState::FlightModePX4::ALTITUDE);
        else if (mode == Telemetry::FlightMode::Posctl)
            msg.setModePX4(ecal::MavState::FlightModePX4::POSITION);
        else if (mode == Telemetry::FlightMode::Land)
            msg.setModePX4(ecal::MavState::FlightModePX4::LAND);
        else if (mode == Telemetry::FlightMode::Offboard)
            msg.setModePX4(ecal::MavState::FlightModePX4::OFFBOARD);
        else {
            spdlog::warn("flight mode not recognised {}", mode);
            std::cout << mode << std::endl;
        }

        if (lastMode != msg.getModePX4()) {
            spdlog::warn("flight mode changes to {}", msg.getModePX4());
            std::cout << mode << std::endl;
        }
            

        if (msg.getHeader().getStamp() < tns)
            msg.getHeader().setStamp(tns);
        else
            spdlog::warn("tns regression on flight mode update, from {} to {}", msg.getHeader().getStamp(), tns);

        m_initialised = true;
    }
    

  private:
    
    bool m_initialised;
    std::mutex m_mutexMavState;
    std::shared_ptr<eCAL::capnproto::CPublisher<ecal::MavState>> m_pubMavState;
    std::thread m_senderThread;
    std::uint64_t m_seq;

    void senderThread(int16_t interval) {
        while (eCAL::Ok()) {
            if (m_initialised)
            {
                std::lock_guard<std::mutex> lock(m_mutexMavState);

                ecal::MavState::Builder msg = m_pubMavState->GetBuilder();
                msg.getHeader().setSeq(m_seq);

                m_pubMavState->Send();

                m_seq++;
            }
            std::this_thread::sleep_for(seconds(interval));
        }
    }
};



int main(int argc, char** argv)
{
    if (argc != 2) {
        usage(argv[0]);
        return 1;
    }

    Mavsdk mavsdk;
    Mavsdk::Configuration configuration{Mavsdk::Configuration::UsageType::CompanionComputer}; // default system id to 1, and component id to 195
    configuration.set_component_id(MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY);
    mavsdk.set_configuration(configuration); 
    ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

    std::string tf_prefix = "S0/";

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

    // send heartbeat
    // MavlinkPassthrough mavlink_passthrough{system};

    // std::thread(
    //     [&mavlink_passthrough] (uint8_t system_id) {

    //         while(eCAL::Ok()) {
    //             send_heartbeat(mavlink_passthrough, system_id, true);
    //             std::this_thread::sleep_for(std::chrono::seconds(1));
    //         }
            
    //     },
    //     system->get_system_id()
    // );

    spdlog::info("wait for timesync to complete...");
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (system->is_timesync_converged())
            break;
    }

    std::uint64_t last_local_pos = 0;

    telemetry.subscribe_position_velocity_ned(
        [&] (Telemetry::PositionVelocityNed local_position) {

            std::uint64_t tns = std::chrono::steady_clock::now().time_since_epoch().count();

            if (tns - last_local_pos > 5e9) {
                spdlog::info("local position ned received: {}, {}, {}", local_position.position.north_m, 
                    local_position.position.east_m, local_position.position.down_m);

                last_local_pos =  tns;
            }

        }
    );

    std::uint64_t last_odometry = 0;
    telemetry.subscribe_odometry(
        [&] (Telemetry::Odometry odometry_data) {

            std::uint64_t tns = std::chrono::steady_clock::now().time_since_epoch().count();

            if (tns - last_odometry > 5e9) {
                uint64_t time_usec = odometry_data.time_usec - system->get_timesync_offset_ns() / 1e3;
                spdlog::info("{} odometry received at host: {} {} {} ", time_usec, 
                    odometry_data.position_body.x_m, odometry_data.position_body.y_m, odometry_data.position_body.z_m);

                last_odometry =  tns;
            }

            
    });

    // Create eCAL publisher of mav status

    MavStateSender mavStateSender(tf_prefix, 1);

    telemetry.subscribe_armed(
        [&mavStateSender, system] (bool armed) {
            mavStateSender.updateArmStatus(armed);
        }
    );

    telemetry.subscribe_flight_mode(
        [&mavStateSender, system] (Telemetry::FlightMode mode) {
            mavStateSender.updateFlightMode(mode);
        }
    );

    telemetry.subscribe_battery(
        [] (Telemetry::Battery battery_data) {
            spdlog::info("battery voltage = {}, {}%", battery_data.voltage_v, battery_data.remaining_percent);
        }
    );


    MavlinkOdometrySender mavOdometrySender{system};

    // std::thread t_odometry_send(run_fake_odometry_send, system);    
    eCAL::Initialize(0, nullptr, "ecal odometry mavlink bridge");
    eCAL::Process::SetState(proc_sev_healthy, proc_sev_level1, "I feel good !");

    spdlog::info("eCAL Version: {}", eCAL::GetVersionString());

    std::shared_ptr<eCAL::capnproto::CSubscriber<ecal::Odometry3d>> subOdometry;

    subOdometry = std::make_shared<eCAL::capnproto::CSubscriber<ecal::Odometry3d>>(tf_prefix + "vio_odom_ned");
    subOdometry->AddReceiveCallback(std::bind(&MavlinkOdometrySender::odometry_callback, &mavOdometrySender, 
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    while (eCAL::Ok()) {
        std::this_thread::sleep_for(seconds(10));
        spdlog::info("system steady time now {} ms", std::chrono::steady_clock::now().time_since_epoch().count() / 1e6);
        // spdlog::info("current time offset estimated: {} ms", system->get_timesync_offset_ns() / 1e6);
    }

    return 0;
}