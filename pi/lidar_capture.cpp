// Record complete RPLIDAR scans as newline-delimited JSON.
//
// This program uses SLAMTEC's scan timestamp API. On Linux, the SDK maps the
// timestamp into CLOCK_MONOTONIC microseconds, which can be compared with the
// camera SensorTimestamp values recorded by rpicam-vid after unit conversion.

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <time.h>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

namespace {

std::atomic<bool> stop_requested{false};

void request_stop(int) {
    stop_requested.store(true);
}

std::uint64_t monotonic_ns() {
    timespec value{};
    if (clock_gettime(CLOCK_MONOTONIC, &value) != 0) {
        return 0;
    }
    return static_cast<std::uint64_t>(value.tv_sec) * 1'000'000'000ULL +
           static_cast<std::uint64_t>(value.tv_nsec);
}

void print_usage(const char* program) {
    std::cerr
        << "Usage: " << program
        << " <serial-port> <baudrate> <duration-seconds> <output.jsonl>\n";
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
    const double duration_seconds = std::strtod(argv[3], nullptr);
    const std::string output_path = argv[4];

    if (baudrate == 0 || duration_seconds <= 0.0) {
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

    sl_lidar_response_device_info_t device_info{};
    if (SL_IS_FAIL(driver->getDeviceInfo(device_info))) {
        std::cerr << "Failed to read lidar device information.\n";
        delete driver;
        return 5;
    }

    sl_lidar_response_device_health_t health{};
    if (SL_IS_FAIL(driver->getHealth(health)) ||
        health.status == SL_LIDAR_STATUS_ERROR) {
        std::cerr << "Lidar health check failed; status="
                  << static_cast<int>(health.status)
                  << " error_code=" << health.error_code << ".\n";
        delete driver;
        return 6;
    }

    std::ofstream output(output_path, std::ios::out | std::ios::trunc);
    if (!output) {
        std::cerr << "Failed to open output file " << output_path << ".\n";
        delete driver;
        return 7;
    }

    output << "{\"type\":\"header\",\"schema_version\":1,"
              "\"timestamp_domain\":\"CLOCK_MONOTONIC\","
              "\"timestamp_unit\":\"microseconds\","
              "\"serial_port\":\""
           << serial_port << "\",\"baudrate\":" << baudrate
           << ",\"firmware_version\":\"" << (device_info.firmware_version >> 8)
           << "." << std::setfill('0') << std::setw(2)
           << (device_info.firmware_version & 0xFF) << std::setfill(' ')
           << "\",\"hardware_revision\":"
           << static_cast<int>(device_info.hardware_version) << "}\n";

    driver->setMotorSpeed();
    if (SL_IS_FAIL(driver->startScan(0, 1))) {
        std::cerr << "Failed to start lidar scanning.\n";
        driver->setMotorSpeed(0);
        delete driver;
        return 8;
    }

    const auto started = std::chrono::steady_clock::now();
    const auto deadline =
        started + std::chrono::duration<double>(duration_seconds);

    std::size_t scan_index = 0;
    std::size_t timeout_count = 0;
    std::size_t rejected_scan_count = 0;
    std::uint64_t previous_timestamp_us = 0;

    while (!stop_requested.load() &&
           std::chrono::steady_clock::now() < deadline) {
        sl_lidar_response_measurement_node_hq_t nodes[8192]{};
        std::size_t count = sizeof(nodes) / sizeof(nodes[0]);
        sl_u64 scan_timestamp_us = 0;

        const sl_result result = driver->grabScanDataHqWithTimeStamp(
            nodes, count, scan_timestamp_us, 2000);
        if (result == SL_RESULT_OPERATION_TIMEOUT) {
            ++timeout_count;
            continue;
        }
        if (SL_IS_FAIL(result)) {
            ++rejected_scan_count;
            std::cerr << "Rejected scan; SDK result=0x" << std::hex << result
                      << std::dec << ".\n";
            continue;
        }
        if (scan_timestamp_us <= previous_timestamp_us) {
            ++rejected_scan_count;
            std::cerr << "Rejected non-monotonic scan timestamp "
                      << scan_timestamp_us << ".\n";
            continue;
        }
        if (SL_IS_FAIL(driver->ascendScanData(nodes, count))) {
            ++rejected_scan_count;
            continue;
        }

        std::size_t valid_count = 0;
        for (std::size_t index = 0; index < count; ++index) {
            if (nodes[index].dist_mm_q2 != 0) {
                ++valid_count;
            }
        }

        output << "{\"type\":\"scan\",\"scan_index\":" << scan_index
               << ",\"timestamp_us\":" << scan_timestamp_us
               << ",\"host_received_monotonic_ns\":" << monotonic_ns()
               << ",\"raw_point_count\":" << count
               << ",\"valid_point_count\":" << valid_count
               << ",\"points\":[";

        bool first_point = true;
        output << std::fixed << std::setprecision(3);
        for (std::size_t index = 0; index < count; ++index) {
            const auto& node = nodes[index];
            if (node.dist_mm_q2 == 0) {
                continue;
            }
            if (!first_point) {
                output << ',';
            }
            first_point = false;
            const double angle_deg =
                (node.angle_z_q14 * 90.0) / 16384.0;
            const double distance_m = (node.dist_mm_q2 / 4.0) / 1000.0;
            const int quality =
                node.quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
            output << '[' << angle_deg << ',' << distance_m << ',' << quality
                   << ']';
        }
        output << "]}\n";

        previous_timestamp_us = scan_timestamp_us;
        ++scan_index;
        if (scan_index % 20 == 0) {
            output.flush();
        }
    }

    driver->stop();
    driver->setMotorSpeed(0);

    output << "{\"type\":\"summary\",\"scan_count\":" << scan_index
           << ",\"timeout_count\":" << timeout_count
           << ",\"rejected_scan_count\":" << rejected_scan_count
           << ",\"end_monotonic_ns\":" << monotonic_ns() << "}\n";
    output.flush();

    delete driver;

    std::cout << "Recorded " << scan_index << " scans to " << output_path
              << "; timeouts=" << timeout_count
              << "; rejected=" << rejected_scan_count << ".\n";
    return scan_index == 0 ? 9 : 0;
}

