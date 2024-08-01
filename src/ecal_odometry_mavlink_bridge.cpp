#include <vk_sdk/Sdk.hpp>
#include "vk_sdk/capnp/Shared.hpp"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mocap/mocap.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>

#include <ecal/ecal.h>
// #include <ecal/msg/capnproto/helper.h>
// #include <ecal/msg/capnproto/subscriber.h>
// #include <ecal/msg/capnproto/publisher.h>


// #include <vk_sdk/odometry3d.capnp.h>
// #include <vkc/mavstate.capnp.h>

#include <iostream>
#include <chrono>
#include <future>
#include <memory>
#include <thread>

#include <sophus/se3.hpp>

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
              << "Followed by 0 for VK180 or 1 for VK180P\n"
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

class VkcOdomReceiver: public vkc::Receiver<vkc::Odometry3d> {
  public:
    VkcOdomReceiver(std::shared_ptr<System> system)
        : m_mocap(std::make_shared<Mocap>(system)) {
        m_odom_msg.pose_covariance.covariance_matrix.resize(1);
        m_odom_msg.pose_covariance.covariance_matrix[0] = NAN;
        m_odom_msg.velocity_covariance.covariance_matrix.resize(1);
        m_odom_msg.velocity_covariance.covariance_matrix[0] = NAN;
        m_odom_msg.mav_estimator = Mocap::Odometry::MavEstimator::Vision;
        count = 0;
    }
    vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Odometry3d>>& message) override {
        auto reader = message.payload.reader();
        const auto& header =  reader.getHeader();
        auto seq = header.getSeq();
        auto tns = header.getStampMonotonic();

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
            std::uint64_t nowTns = std::chrono::steady_clock::now().time_since_epoch().count();
            
            spdlog::debug("odometry of seq = {}, ts = {} sent at host ts = {}, latency = {} ms", seq, tns, nowTns, (nowTns - tns) / 1e6);
        }else
            spdlog::warn("failed to send odometry over mavlink to px4");
        
        return vkc::ReceiverStatus::Open;
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
  private:
    std::shared_ptr<Mocap> m_mocap;
    Mocap::Odometry m_odom_msg;
    uint64_t count;
};

// class MavlinkOdometrySender {

//   public:
//     MavlinkOdometrySender(std::shared_ptr<System> system)
//     {
//         m_mocap = std::make_shared<Mocap>(system);

//         // not implementing pose covariance yet
//         m_odom_msg.pose_covariance.covariance_matrix.resize(1);
//         m_odom_msg.pose_covariance.covariance_matrix[0] = NAN;

//         // not implementing velocity covriance yet
//         m_odom_msg.velocity_covariance.covariance_matrix.resize(1);
//         m_odom_msg.velocity_covariance.covariance_matrix[0] = NAN;

//         // hardcode as vision now
//         m_odom_msg.mav_estimator = Mocap::Odometry::MavEstimator::Vision;

//         count = 0;
//     }

//     bool Send(uint64_t tns, Mocap::PositionBody& p, Mocap::Quaternion& q)
//     {
//         m_odom_msg.time_usec = tns / 1e3;
//         m_odom_msg.position_body = p;
//         m_odom_msg.q = q;

//         auto ret = m_mocap->set_odometry(m_odom_msg);

//         if (ret == Mocap::Result::NoSystem)
//             spdlog::warn("no system connected");
//         else if (ret == Mocap::Result::Success)
//             spdlog::debug("mocap sent success");
//         else
//             spdlog::warn("mocap send other error {}", ret);
        
//         if (count % 100 == 0)
//             std::cout << "mavlink odometry message sent to px4: " << m_odom_msg << std::endl;
//         count++;

//         return (ret == Mocap::Result::Success);
//     }

//     // void odometry_callback(const char* ecal_topic_name, vkc::Odometry3d::Reader ecal_msg, const long long ecal_ts) {
//     void odometry_callback(const char* ecal_topic_name, const eCAL::SReceiveCallbackData* data) {
    
//         UNUSED(ecal_topic_name);
//         // UNUSED(ecal_ts);

//         kj::ArrayPtr<const kj::byte> bytes(reinterpret_cast<const kj::byte*>(data->buf), data->size);
//         kj::ArrayInputStream stream(bytes);
//         std::shared_ptr<capnp::MessageBuilder> mBuffer = stream;
//         vkc::Odometry3d::Reader ecal_msg = mBuffer->getRoot<vkc::Odometry3d>().asReader();
//         const auto& header = ecal_msg.getHeader();
//         auto seq = header.getSeq();
//         auto tns = header.getStampMonotonic();

//         Mocap::PositionBody p;
//         auto position = ecal_msg.getPose().getPosition();
//         p.x_m = position.getX();
//         p.y_m = position.getY();
//         p.z_m = position.getZ();

//         Mocap::Quaternion q;
//         auto orientation = ecal_msg.getPose().getOrientation();
//         q.w = orientation.getW();
//         q.x = orientation.getX();
//         q.y = orientation.getY();
//         q.z = orientation.getZ();

//         if (Send(tns, p, q)) {
//             std::uint64_t nowTns = std::chrono::steady_clock::now().time_since_epoch().count();
            
//             spdlog::debug("odometry of seq = {}, ts = {} sent at host ts = {}, latency = {} ms", seq, tns, nowTns, (nowTns - tns) / 1e6);
//         }else
//             spdlog::warn("failed to send odometry over mavlink to px4");


//     }

//   private:
//     std::shared_ptr<Mocap> m_mocap;
//     Mocap::Odometry m_odom_msg;
//     uint64_t count;
// };

class EcalMavStateSender {
  public:
    EcalMavStateSender(std::unique_ptr<vkc::Receiver<vkc::MavState>> recv, int sendIntervalSec = 1)
        : m_pubMavState(std::move(recv))
    {
        // m_pubMavState = std::make_shared<eCAL::capnproto::CPublisher<vkc::MavState>>();
        // m_pubMavState->Create(tf_prefix + "mav_state");
        m_initialised = false;
        m_seq = 0;

        m_armed = false;
        m_mode = vkc::MavState::FlightModePX4::UNKNOWN;
        m_tns = 0;
        m_senderThread = std::thread(&EcalMavStateSender::senderThread, this, sendIntervalSec);
    }

    void updateArmStatus(bool armed)
    {
        std::lock_guard<std::mutex> lock(m_mutexMavState);
        // vkc::MavState::Builder msg = m_pubMavState->GetBuilder();

        std::uint64_t tns = std::chrono::steady_clock::now().time_since_epoch().count();

        if (m_armed != armed)
        {
            spdlog::warn("arm status update to {}", armed);
        }

        m_armed = armed;

        if (m_tns < tns)
            m_tns = tns;
        else
            spdlog::warn("tns regression on flight mode update, from {} to {}", tns, m_tns);

        m_initialised = true;
    }

    void updateFlightMode(Telemetry::FlightMode mode)
    {
        std::lock_guard<std::mutex> lock(m_mutexMavState);
        // vkc::MavState::Builder msg = m_pubMavState->GetBuilder();

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
    

  private:
    
    bool m_initialised;
    std::mutex m_mutexMavState;
    // std::shared_ptr<eCAL::capnproto::CPublisher<vkc::MavState>> m_pubMavState;
    std::unique_ptr<vkc::Receiver<vkc::MavState>> m_pubMavState;
    bool m_armed;
    vkc::MavState::FlightModePX4 m_mode;
    unsigned long m_tns;

    std::thread m_senderThread;
    std::uint64_t m_seq;

    void senderThread(int16_t interval) {
        while (eCAL::Ok()) {
            if (m_initialised)
            {
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
};

class EcalLocalPositionSender {

  public:
    EcalLocalPositionSender(std::unique_ptr<vkc::Receiver<vkc::Odometry3d>> ned_receiver,
                            std::unique_ptr<vkc::Receiver<vkc::Odometry3d>> nwu_receiver)
        : m_pubLocalPositionNED(std::move(ned_receiver)), m_pubLocalPositionNWU(std::move(nwu_receiver)) {
        std::cout << "publisher ecal for px4 local position created" << std::endl;

        // // ned publisher
        // {
        //     m_pubLocalPositionNED = std::make_shared<eCAL::capnproto::CPublisher<vkc::Odometry3d>>();
        //     m_pubLocalPositionNED->Create(tf_prefix + "local_position_ned");

        // }

        // // nwu publisher
        // {
        //     m_pubLocalPositionNWU = std::make_shared<eCAL::capnproto::CPublisher<vkc::Odometry3d>>();
        //     m_pubLocalPositionNWU->Create(tf_prefix + "local_position");
        // }

    }

    void callback(Telemetry::PositionVelocityNed local_position, Telemetry::Quaternion attitude_quat) {
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

  private:
    std::unique_ptr<vkc::Receiver<vkc::Odometry3d>> m_pubLocalPositionNED, m_pubLocalPositionNWU;
    // std::shared_ptr<eCAL::capnproto::CPublisher<vkc::Odometry3d>> m_pubLocalPositionNED, m_pubLocalPositionNWU;
};


int main(int argc, char** argv)
{
    if (argc < 3 || argc > 4) {
        usage(argv[0]);
        return 1;
    }
    
    const std::string tf_prefix = "S" + argv[2] +"/";

    Mavsdk::Configuration configuration{Mavsdk::ComponentType::GroundStation}; // default system id to 1, we need GCS mode so we can receive mavlink logs
    // configuration.set_component_id(MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY); // This should be avoided, as it will prevent PX4 sending by info, warning, debug etc
    Mavsdk mavsdk{configuration};
    ConnectionResult connection_result = mavsdk.add_any_connection(argv[1], argc == 4 ? ForwardingOption::ForwardingOn : ForwardingOption::ForwardingOff);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed to autopilot: " << connection_result << '\n';
        return 1;
    }

    // we will also add the connection to gcs

    if (argc == 4) {

        std::cout << "connecting to gcs at " << argv[2] << std::endl;
        connection_result = mavsdk.add_any_connection(argv[2], argc == 3 ? ForwardingOption::ForwardingOn : ForwardingOption::ForwardingOff);

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

    // std::shared_ptr<eCAL::capnproto::CPublisher<vkc::Odometry3d>> pubOdometry;

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
// m_pubMavState->Create(tf_prefix + "mav_state");
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


    // MavlinkOdometrySender mavOdometrySender{system};

    // std::thread t_odometry_send(run_fake_odometry_send, system);    
    eCAL::Initialize(0, nullptr, "ecal odometry mavlink bridge");
    eCAL::Process::SetState(proc_sev_healthy, proc_sev_level1, "I feel good !");

    spdlog::info("eCAL Version: {}", eCAL::GetVersionString());

    auto odomReceiver = std::make_unique<VkcOdomReceiver>(system);
    visualkit->source().install(tf_prefix + "vio_odom_ned", std::move(odomReceiver));

    visualkit->source().start();
    visualkit->sink().start();
    // std::shared_ptr<eCAL::CSubscriber> subOdometry;

    // subOdometry = std::make_shared<eCAL::CSubscriber>(tf_prefix + "vio_odom_ned");
    // subOdometry->AddReceiveCallback(std::bind(&MavlinkOdometrySender::odometry_callback, &mavOdometrySender, 
    //     std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    while (eCAL::Ok()) {
        std::this_thread::sleep_for(seconds(10));
        spdlog::info("system steady time now {} ms", std::chrono::steady_clock::now().time_since_epoch().count() / 1e6);
        // spdlog::info("current time offset estimated: {} ms", system->get_timesync_offset_ns() / 1e6);
    }
    visualkit->sink().stop(false);
    visualkit->source().stop(false);

    return 0;
}