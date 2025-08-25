#include "ecal_odometry_mavlink_bridge.hpp"
#include "waypoint_navigator/waypoint_navigator.hpp"

using namespace mavsdk;
using namespace ecal_mavlink;
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
              << "Followed by 0 for VK180 or 1 for VK180P\n"
              << "Followed by path to waypoint mission yaml file\n"
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

//VkcOdomReceiver
VkcOdomReceiver::VkcOdomReceiver(std::shared_ptr<System> system)
    : m_system(system),
      m_mocap(std::make_shared<Mocap>(system)) {

    m_odom_msg.pose_covariance.covariance_matrix.resize(1);
    m_odom_msg.pose_covariance.covariance_matrix[0] = NAN;
    m_odom_msg.velocity_covariance.covariance_matrix.resize(1);
    m_odom_msg.velocity_covariance.covariance_matrix[0] = NAN;
    m_odom_msg.mav_estimator = Mocap::Odometry::MavEstimator::Vision;
    count = 0;
}

vkc::ReceiverStatus VkcOdomReceiver::handle(const vkc::Message<vkc::Shared<vkc::Odometry3d>>& message) {
    auto reader = message.payload.reader();
    const auto& header = reader.getHeader();
    auto seq = header.getSeq();

    // Using PX4-MAVSDK timesync offset
    uint64_t header_stamp_ns = header.getStampMonotonic();
    uint64_t tns = header_stamp_ns;

    Mocap::PositionBody p;
    auto position = reader.getPose().getPosition();
    p.x_m = position.getX();
    p.y_m = position.getY();
    p.z_m = position.getZ();

    Mocap::Quaternion q;
    auto orientation = reader.getPose().getOrientation();
    q.w = orientation.getW();
    q.x = orientation.getX();
    q.y = orientation.getY();
    q.z = orientation.getZ();

    if (Send(tns, p, q)) {
        spdlog::debug("odometry of seq={} sent successfully", seq);
    } else {
        spdlog::warn("failed to send odometry over mavlink to px4");
    }
    
    return vkc::ReceiverStatus::Open;
}

bool VkcOdomReceiver::Send(uint64_t tns, Mocap::PositionBody& p, Mocap::Quaternion& q) {
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
    
    if (count % 100 == 0) {
        std::cout << "mavlink odometry message sent to px4: " << m_odom_msg << std::endl;
    }
    count++;

    return (ret == Mocap::Result::Success);
}


//EcalMavStateSender
EcalMavStateSender::EcalMavStateSender(std::unique_ptr<vkc::Receiver<vkc::MavState>> recv, int sendIntervalSec)
    : m_pubMavState(std::move(recv)) {
    m_initialised = false;
    m_seq = 0;
    m_armed = false;
    m_mode = vkc::MavState::FlightModePX4::UNKNOWN;
    m_tns = 0;
    m_senderThread = std::thread(&EcalMavStateSender::senderThread, this, sendIntervalSec);
}

void EcalMavStateSender::updateArmStatus(bool armed) {
    std::lock_guard<std::mutex> lock(m_mutexMavState);
    std::uint64_t tns = std::chrono::steady_clock::now().time_since_epoch().count();

    if (m_armed != armed) {
        spdlog::warn("arm status update to {}", armed);
    }

    m_armed = armed;

    if (m_tns < tns)
        m_tns = tns;
    else
        spdlog::warn("tns regression on flight mode update, from {} to {}", tns, m_tns);

    m_initialised = true;
}

void EcalMavStateSender::updateFlightMode(Telemetry::FlightMode mode) {
    std::lock_guard<std::mutex> lock(m_mutexMavState);
    unsigned long tns = std::chrono::steady_clock::now().time_since_epoch().count();

    const auto lastMode = m_mode;

    if (mode == Telemetry::FlightMode::Manual)
        m_mode = vkc::MavState::FlightModePX4::MANUAL;
    else if (mode == Telemetry::FlightMode::Altctl)
        m_mode = vkc::MavState::FlightModePX4::ALTITUDE;
    else if (mode == Telemetry::FlightMode::Posctl)
        m_mode = vkc::MavState::FlightModePX4::POSITION;
    else if (mode == Telemetry::FlightMode::Land)
        m_mode = vkc::MavState::FlightModePX4::LAND;
    else if (mode == Telemetry::FlightMode::Offboard)
        m_mode = vkc::MavState::FlightModePX4::OFFBOARD;
    else {
        spdlog::warn("flight mode not recognised {}", mode);
    }

    if (lastMode != m_mode) {
        spdlog::warn("flight mode changes to {}", mode);
    }

    if (m_tns < tns)
        m_tns = tns;
    else
        spdlog::warn("tns regression on flight mode update, from {} to {}", m_tns, tns);

    m_initialised = true;
}

void EcalMavStateSender::senderThread(int16_t interval) {
    while (eCAL::Ok()) {
        if (m_initialised) {
            std::lock_guard<std::mutex> lock(m_mutexMavState);

            auto builder = std::make_unique<capnp::MallocMessageBuilder>();
            vkc::MavState::Builder msg = builder->initRoot<vkc::MavState>();
            msg.setArmed(m_armed);
            msg.setModePX4(m_mode);
            msg.getHeader().setStampMonotonic(m_tns);
            msg.getHeader().setSeq(m_seq);
            m_pubMavState->handle(vkc::Shared<vkc::MavState>(std::move(builder)));

            m_seq++;
        }
        std::this_thread::sleep_for(seconds(interval));
    }
}

//EcalLocalPositionSender
EcalLocalPositionSender::EcalLocalPositionSender(std::unique_ptr<vkc::Receiver<vkc::Odometry3d>> ned_receiver,
                                               std::unique_ptr<vkc::Receiver<vkc::Odometry3d>> nwu_receiver)
    : m_pubLocalPositionNED(std::move(ned_receiver)), 
      m_pubLocalPositionNWU(std::move(nwu_receiver)) {
    std::cout << "publisher ecal for px4 local position created" << std::endl;
}

void EcalLocalPositionSender::callback(Telemetry::PositionVelocityNed local_position, Telemetry::Quaternion attitude_quat) {
    std::uint64_t tns = std::chrono::steady_clock::now().time_since_epoch().count();

    // ned publisher
    {
        auto builder = std::make_unique<capnp::MallocMessageBuilder>();
        vkc::Odometry3d::Builder odomBuilder = builder->initRoot<vkc::Odometry3d>();
        auto header = odomBuilder.getHeader();
        header.setStampMonotonic(tns);
        header.setSeq(header.getSeq() + 1);
        header.setClockDomain(vkc::Header::ClockDomain::MONOTONIC);
        odomBuilder.setBodyFrame(vkc::Odometry3d::BodyFrame::NED);
        odomBuilder.setReferenceFrame(vkc::Odometry3d::ReferenceFrame::NED);
        odomBuilder.setVelocityFrame(vkc::Odometry3d::VelocityFrame::NONE);

        auto orientation = odomBuilder.getPose().getOrientation();
        orientation.setW(attitude_quat.w);
        orientation.setX(attitude_quat.x);
        orientation.setY(attitude_quat.y);
        orientation.setZ(attitude_quat.z);

        auto position = odomBuilder.getPose().getPosition();
        position.setX(local_position.position.north_m);
        position.setY(local_position.position.east_m);
        position.setZ(local_position.position.down_m);
        auto shared = vkc::Shared<vkc::Odometry3d>(std::move(builder));
        m_pubLocalPositionNED->handle(shared);
    }

    // nwu publisher
    {
        Eigen::Vector3d position_ned = {
            local_position.position.north_m,
            local_position.position.east_m,
            local_position.position.down_m
        };

        Eigen::Quaterniond orientation_ned = {
            attitude_quat.w,
            attitude_quat.x,
            attitude_quat.y,
            attitude_quat.z
        };

        Sophus::SE3d T_ned;

        T_ned.translation() = position_ned;
        T_ned.setQuaternion(orientation_ned);

        // transform ned to nwu
        Sophus::Matrix3d R_ned_nwu;
        // change of coordinates from NWU to NED
        Sophus::SE3d T_ned_nwu;
        R_ned_nwu << 1, 0, 0, 0, -1, 0, 0, 0, -1;
        T_ned_nwu.setRotationMatrix(R_ned_nwu);
        T_ned_nwu.translation().setZero();

        Sophus::SE3d T_nwu_nwu;
        T_nwu_nwu = T_ned_nwu.inverse() * T_ned * T_ned_nwu;

        {
            auto builder = std::make_unique<capnp::MallocMessageBuilder>();
            vkc::Odometry3d::Builder odomBuilder = builder->initRoot<vkc::Odometry3d>();
            auto header = odomBuilder.getHeader();
            header.setClockDomain(vkc::Header::ClockDomain::MONOTONIC);
            header.setStampMonotonic(tns);
            header.setSeq(header.getSeq() + 1);
                
            odomBuilder.setBodyFrame(vkc::Odometry3d::BodyFrame::NWU);
            odomBuilder.setReferenceFrame(vkc::Odometry3d::ReferenceFrame::NWU);
            odomBuilder.setVelocityFrame(vkc::Odometry3d::VelocityFrame::NONE);
            auto quat = T_nwu_nwu.unit_quaternion();
            auto orientation = odomBuilder.getPose().getOrientation();
            orientation.setW(quat.w());
            orientation.setX(quat.x());
            orientation.setY(quat.y());
            orientation.setZ(quat.z());

            auto position = odomBuilder.getPose().getPosition();
            position.setX(T_nwu_nwu.translation().x());
            position.setY(T_nwu_nwu.translation().y());
            position.setZ(T_nwu_nwu.translation().z());
            auto shared = vkc::Shared<vkc::Odometry3d>(std::move(builder));

            m_pubLocalPositionNWU->handle(shared);
        }
    }        
}

int main(int argc, char** argv)
{
    if (argc < 4 || argc > 5) {
        usage(argv[0]);
        return 1;
    }
    
    const std::string tf_prefix = "S" + std::string(argv[2]) +"/";

    mavsdk::Mavsdk::Configuration configuration{mavsdk::Mavsdk::ComponentType::GroundStation};
    mavsdk::Mavsdk mavsdk(configuration);

    ConnectionResult connection_result = mavsdk.add_any_connection(argv[1], argc == 4 ? ForwardingOption::ForwardingOn : ForwardingOption::ForwardingOff);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed to autopilot: " << connection_result << '\n';
        return 1;
    }

    // waypoint mission file
    const std::string yaml_path = argv[3];

    // we will also add the connection to gcs

    if (argc == 5) {

        std::cout << "connecting to gcs at " << argv[4] << std::endl;
        connection_result = mavsdk.add_any_connection(argv[4], argc == 5 ? ForwardingOption::ForwardingOn : ForwardingOption::ForwardingOff);

        if (connection_result != ConnectionResult::Success) {
            std::cerr << "Connection failed to gcs: " << connection_result << '\n';
            return 1;
        }
    }

    auto visualkit = vkc::VisualKit::create(std::nullopt);
    auto system = get_system(mavsdk);
    if (!system) {
        return 1;
    }

    system->enable_timesync();

    // Instantiate plugins.
    auto telemetry = Telemetry{system};


    spdlog::info("wait for timesync to complete...");
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (system->is_timesync_converged())
            break;
    }

    waypoint_navigator::WaypointNavigator navigator(system);
    if (!navigator.loadWaypointsFromYaml(yaml_path))
    {
        spdlog::error("Failed to load waypoints from YAML: {}", yaml_path);
        return 1;
    }

    /*
    telemetry.set_rate_position_velocity_ned(10.0); // 10 Hz update rate for position and velocity
    telemetry.set_rate_attitude_euler(10.0);        // also throttle euler attitude updates
    */

    auto ned_receiver = visualkit->sink().obtain(tf_prefix + "local_position_ned", vkc::Type<vkc::Odometry3d>());
    auto nwu_receiver = visualkit->sink().obtain(tf_prefix + "local_position", vkc::Type<vkc::Odometry3d>());
    EcalLocalPositionSender ecalLocalPositionSender(std::move(ned_receiver), std::move(nwu_receiver));
    telemetry.subscribe_position_velocity_ned(
        [&] (Telemetry::PositionVelocityNed local_position) {

            auto tele_quat = telemetry.attitude_quaternion();
            if (std::isnan(tele_quat.w) || std::isnan(tele_quat.x) || std::isnan(tele_quat.y) || std::isnan(tele_quat.z)) {
                spdlog::warn("nan quaternion");
                return;
            }
            ecalLocalPositionSender.callback(local_position, tele_quat);

            auto telem_euler = telemetry.attitude_euler();
            navigator.updateLocalPose(local_position, telem_euler);
        }
    );

    telemetry.subscribe_flight_mode([&navigator](Telemetry::FlightMode flight_mode)
                                    {
        navigator.updateFlightMode(flight_mode);
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
    auto mav_state_recv = visualkit->sink().obtain(tf_prefix + "mav_state", vkc::Type<vkc::MavState>());
    EcalMavStateSender EcalMavStateSender(std::move(mav_state_recv), 1);

    telemetry.subscribe_armed(
        [&EcalMavStateSender, system] (bool armed) {
            EcalMavStateSender.updateArmStatus(armed);
        }
    );

    telemetry.subscribe_flight_mode(
        [&EcalMavStateSender, system] (Telemetry::FlightMode mode) {
            EcalMavStateSender.updateFlightMode(mode);
        }
    );

    telemetry.subscribe_battery(
        [] (Telemetry::Battery battery_data) {
            spdlog::info("battery voltage = {}, {}%", battery_data.voltage_v, battery_data.remaining_percent);
        }
    );

   
    eCAL::Initialize(0, nullptr, "ecal odometry mavlink bridge");
    eCAL::Process::SetState(proc_sev_healthy, proc_sev_level1, "I feel good !");

    spdlog::info("eCAL Version: {}", eCAL::GetVersionString());

    auto odomReceiver = std::make_unique<VkcOdomReceiver>(system);
    visualkit->source().install(tf_prefix + "vio_odom_ned", std::move(odomReceiver));

    visualkit->source().start();
    visualkit->sink().start();

    static int64_t last_offset = 0;
    std::atomic_bool running = true;

    mavsdk::MavlinkPassthrough mavlink_passthrough{system};

    std::atomic<waypoint_navigator::TaskState> last_task_state = waypoint_navigator::TaskState::IDLE;
    uint16_t last_ch8 = 0;

    mavlink_passthrough.subscribe_message(MAVLINK_MSG_ID_RC_CHANNELS,
        [&](const mavlink_message_t& message) {
            mavlink_rc_channels_t rc;
            mavlink_msg_rc_channels_decode(&message, &rc);

            uint16_t ch7 = rc.chan7_raw;
            uint16_t ch8 = rc.chan8_raw;

            // LAND: Always takes priority
            if (ch8 >= 1800 && ch8 <= 2000 && last_task_state != waypoint_navigator::TaskState::LAND) {
                navigator.setTaskState(waypoint_navigator::TaskState::LAND);
                last_task_state = waypoint_navigator::TaskState::LAND;
                spdlog::info("RC Command: LAND (CH8={})", ch8);
                last_ch8 = ch8;
                return;
            }

            // IDLE: Edge-triggered only when entering 1000-1200 zone
            if (ch8 >= 1000 && ch8 <= 1200 &&
                !(last_ch8 >= 1000 && last_ch8 <= 1200) &&
                last_task_state != waypoint_navigator::TaskState::IDLE) {
                navigator.setTaskState(waypoint_navigator::TaskState::IDLE);
                last_task_state = waypoint_navigator::TaskState::IDLE;
                spdlog::info("RC Command: IDLE (CH8={} -> CH8={})", last_ch8, ch8);
                last_ch8 = ch8;
                return;
            }

            // Update CH8 for next call
            last_ch8 = ch8;

            // TAKEOFF: only from IDLE
            if (last_task_state == waypoint_navigator::TaskState::IDLE &&
                ch7 >= 1400 && ch7 <= 1600) {
                navigator.setTaskState(waypoint_navigator::TaskState::TAKEOFF);
                last_task_state = waypoint_navigator::TaskState::TAKEOFF;
                spdlog::info("RC Command: TAKEOFF (CH7={})", ch7);
                return;
            }

            // MISSION: only from TAKEOFF
            if (last_task_state == waypoint_navigator::TaskState::TAKEOFF &&
                ch7 >= 1800 && ch7 <= 2000) {
                navigator.setTaskState(waypoint_navigator::TaskState::MISSION);
                last_task_state = waypoint_navigator::TaskState::MISSION;
                spdlog::info("RC Command: MISSION (CH7={})", ch7);
                return;
            }
        });

    // Command thread
    // std::thread input_thread([&]() {
    //     while (running) {
    //         std::cout << "\nA: Arm\nD: Disarm\nT: Takeoff\nM: Mission\nL: Land\nQ: Quit\nEnter command: ";
    //         char cmd;
    //         std::cin >> cmd;
    //         cmd = std::toupper(cmd);

    //         switch (cmd) {
    //             case 'A':
    //                 navigator.doArm();
    //                 break;
    //             case 'D':
    //                 navigator.doDisarm();
    //                 break;
    //             case 'T':
    //                 navigator.setTaskState(waypoint_navigator::TaskState::TAKEOFF);
    //                 break;
    //             case 'M':
    //                 navigator.setTaskState(waypoint_navigator::TaskState::MISSION);
    //                 break;
    //             case 'L':
    //                 navigator.setTaskState(waypoint_navigator::TaskState::LAND);
    //                 break;
    //             case 'Q':
    //                 running = false;
    //                 break;
    //             default:
    //                 std::cout << "Unknown command.\n";
    //         }
    //     }
    // });

    // Main timesync monitor loop
    while (eCAL::Ok() && running) {
        std::this_thread::sleep_for(std::chrono::seconds(10));
        int64_t offset_ns = system->get_timesync_offset_ns();
        double offset_ms = offset_ns / 1e6;

        spdlog::info("system steady time now {} ms, current timesync offset {} ms", 
                    std::chrono::steady_clock::now().time_since_epoch().count() / 1e6,
                    offset_ms);

        if (last_offset != 0 && std::abs(offset_ns - last_offset) > 5e6) {
            spdlog::warn("timesync offset jump detected: {} -> {} ms", 
                        last_offset / 1e6, offset_ms);
        }
        last_offset = offset_ns;
    }

    // input_thread.join();  // Wait for the input thread to exit
    // while (eCAL::Ok()) {
    //     std::this_thread::sleep_for(seconds(10));
    //     int64_t offset_ns = system->get_timesync_offset_ns();
    //     double offset_ms = offset_ns / 1e6;

    //     spdlog::info("system steady time now {} ms, current timesync offset {} ms", 
    //                 std::chrono::steady_clock::now().time_since_epoch().count() / 1e6,
    //                 offset_ms);

    //     if (last_offset != 0 && std::abs(offset_ns - last_offset) > 5e6) {
    //         spdlog::warn("timesync offset jump detected: {} -> {} ms", 
    //                     last_offset / 1e6, offset_ms);
    //     }
    //     last_offset = offset_ns;
    // }
    visualkit->sink().stop(false);
    visualkit->source().stop(false);

    return 0;
}