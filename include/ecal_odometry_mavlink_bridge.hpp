#pragma once
// 系统头文件
#include <iostream>
#include <chrono>
#include <future>
#include <memory>
#include <thread>

// 第三方库头文件
#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>
#include <ecal/ecal.h>

// MAVSDK头文件
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mocap/mocap.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

// VK SDK头文件
#include <vk_sdk/Sdk.hpp>
#include "vk_sdk/capnp/Shared.hpp"
#include "vk_sdk/capnp/mavstate.capnp.h"

// Sophus几何库头文件
#include <sophus/se3.hpp>

namespace ecal_mavlink {

class VkcOdomReceiver: public vkc::Receiver<vkc::Odometry3d> {
    public:
        VkcOdomReceiver(std::shared_ptr<mavsdk::System> system);
        vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Odometry3d>>& message) override;
        bool Send(uint64_t tns, mavsdk::Mocap::PositionBody& p, mavsdk::Mocap::Quaternion& q);

    private:
        std::shared_ptr<mavsdk::System> m_system;
        std::shared_ptr<mavsdk::Mocap> m_mocap;
        mavsdk::Mocap::Odometry m_odom_msg;
        uint64_t count;
    };


class EcalMavStateSender {
    public:
        EcalMavStateSender(std::unique_ptr<vkc::Receiver<vkc::MavState>> recv, int sendIntervalSec = 1);
        void updateArmStatus(bool armed);
        void updateFlightMode(mavsdk::Telemetry::FlightMode mode);
        
    private:
        void senderThread(int16_t interval);

        bool m_initialised;
        std::mutex m_mutexMavState;
        std::unique_ptr<vkc::Receiver<vkc::MavState>> m_pubMavState;
        bool m_armed;
        vkc::MavState::FlightModePX4 m_mode;
        unsigned long m_tns;
        std::thread m_senderThread;
        std::uint64_t m_seq;
    };

class EcalLocalPositionSender {
    public:
        EcalLocalPositionSender(std::unique_ptr<vkc::Receiver<vkc::Odometry3d>> ned_receiver,
                                std::unique_ptr<vkc::Receiver<vkc::Odometry3d>> nwu_receiver);
        void callback(mavsdk::Telemetry::PositionVelocityNed local_position, 
                        mavsdk::Telemetry::Quaternion attitude_quat);

    private:
        std::unique_ptr<vkc::Receiver<vkc::Odometry3d>> m_pubLocalPositionNED;
        std::unique_ptr<vkc::Receiver<vkc::Odometry3d>> m_pubLocalPositionNWU;
    };
} // namespace ecal_mavlink
