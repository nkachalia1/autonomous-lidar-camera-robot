// Stream the closest RPLIDAR return inside a configured front sector.
//
// This helper keeps the RPLIDAR motor running until the process receives
// SIGINT/SIGTERM.  It prints one JSON object per scan so a Python controller
// can watch the front obstacle distance without repeatedly starting and
// stopping the lidar motor.

#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

namespace {

std::atomic<bool> stop_requested{false};

void request_stop(int) {
    stop_requested.store(true);
}

double angle_delta_deg(double angle_deg, double center_deg) {
    return std::fmod(angle_deg - center_deg + 540.0, 360.0) - 180.0;
}

void print_usage(const char* program) {
    std::cerr << "Usage: " << program
              << " <serial-port> <baudrate> <front-center-deg>"
              << " <front-half-width-deg>\n";
}

}  // namespace

int main(int argc, char** argv) {
    using namespace sl;

    if (argc != 5) {
        print_usage(argv[0]);
        return 2;
    }

    const std::string serial_port = argv[1];
    const auto baudrate = static_cast<sl_u32>(std::strtoul(argv[2], nullptr, 10));
    const double front_center_deg = std::strtod(argv[3], nullptr);
    const double front_half_width_deg = std::strtod(argv[4], nullptr);

    if (baudrate == 0 || front_half_width_deg <= 0.0 ||
        front_half_width_deg > 180.0) {
        print_usage(argv[0]);
        return 2;
    }

    std::signal(SIGINT, request_stop);
    std::signal(SIGTERM, request_stop);

    ILidarDriver* driver = *createLidarDriver();
    if (driver == nullptr) {
        std::cerr << "Failed to allocate SLAMTEC lidar driver.\n";
        return 3;
    }

    IChannel* channel = *createSerialPortChannel(serial_port.c_str(), baudrate);
    if (channel == nullptr || SL_IS_FAIL(driver->connect(channel))) {
        std::cerr << "Failed to connect to lidar at " << serial_port << ".\n";
        delete driver;
        return 4;
    }

    sl_lidar_response_device_health_t health{};
    if (SL_IS_FAIL(driver->getHealth(health)) ||
        health.status == SL_LIDAR_STATUS_ERROR) {
        std::cerr << "Lidar health check failed; status="
                  << static_cast<int>(health.status)
                  << " error_code=" << health.error_code << ".\n";
        delete driver;
        return 5;
    }

    driver->setMotorSpeed();
    if (SL_IS_FAIL(driver->startScan(0, 1))) {
        std::cerr << "Failed to start lidar scanning.\n";
        driver->setMotorSpeed(0);
        delete driver;
        return 6;
    }

    std::size_t scan_index = 0;
    std::uint64_t previous_timestamp_us = 0;

    while (!stop_requested.load()) {
        sl_lidar_response_measurement_node_hq_t nodes[8192]{};
        std::size_t count = sizeof(nodes) / sizeof(nodes[0]);
        sl_u64 timestamp_us = 0;

        const sl_result result = driver->grabScanDataHqWithTimeStamp(
            nodes, count, timestamp_us, 2000);
        if (SL_IS_FAIL(result)) {
            continue;
        }
        if (timestamp_us <= previous_timestamp_us) {
            continue;
        }
        if (SL_IS_FAIL(driver->ascendScanData(nodes, count))) {
            continue;
        }

        double closest_m = std::numeric_limits<double>::infinity();
        std::size_t front_count = 0;
        std::size_t valid_count = 0;

        for (std::size_t index = 0; index < count; ++index) {
            const auto& node = nodes[index];
            if (node.dist_mm_q2 == 0) {
                continue;
            }

            ++valid_count;
            const double angle_deg = (node.angle_z_q14 * 90.0) / 16384.0;
            const double distance_m = (node.dist_mm_q2 / 4.0) / 1000.0;

            if (distance_m < 0.10 || distance_m > 8.0) {
                continue;
            }

            if (std::abs(angle_delta_deg(angle_deg, front_center_deg)) <=
                front_half_width_deg) {
                ++front_count;
                if (distance_m < closest_m) {
                    closest_m = distance_m;
                }
            }
        }

        std::cout << std::fixed << std::setprecision(3);
        std::cout << "{\"type\":\"front\",\"scan_index\":" << scan_index
                  << ",\"timestamp_us\":" << timestamp_us
                  << ",\"valid_count\":" << valid_count
                  << ",\"front_count\":" << front_count
                  << ",\"closest_m\":";
        if (std::isfinite(closest_m)) {
            std::cout << closest_m;
        } else {
            std::cout << "null";
        }
        std::cout << "}\n" << std::flush;

        previous_timestamp_us = timestamp_us;
        ++scan_index;
    }

    driver->stop();
    driver->setMotorSpeed(0);
    delete driver;
    return 0;
}
