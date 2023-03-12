#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mocap/mocap.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <spdlog/spdlog.h>

#include <ecal/ecal.h>
// #include <ecal/msg/capnproto/helper.h>
#include <ecal/msg/capnproto/subscriber.h>
// #include <ecal/msg/capnproto/publisher.h>


#include <capnp/odometry3d.capnp.h>


#include <iostream>
#include <chrono>
#include <future>
#include <memory>
#include <thread>

using namespace mavsdk;
using std::chrono::seconds;
using std::chrono::milliseconds;

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
        m_odom_msg.time_usec = tns;
        m_odom_msg.position_body = p;
        m_odom_msg.q = q;

        auto ret = m_mocap->set_odometry(m_odom_msg);

        if (ret == Mocap::Result::NoSystem)
            spdlog::warn("no system connected");
        else if (ret == Mocap::Result::Success)
            spdlog::info("mocap sent success");
        else
            spdlog::warn("mocap send other error {}", ret);
        
        if (count % 100 == 0)
            std::cout << "vision position estimate: " << m_odom_msg << std::endl;
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

        Send(tns, p, q);

        spdlog::info("odometry seq = {}", seq);
    }

  private:
    std::shared_ptr<Mocap> m_mocap;
    Mocap::Odometry m_odom_msg;
    uint64_t count;
};



int main(int argc, char** argv)
{
    if (argc != 2) {
        usage(argv[0]);
        return 1;
    }

    Mavsdk mavsdk;
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