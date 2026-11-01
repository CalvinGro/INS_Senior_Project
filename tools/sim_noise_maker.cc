/*
Author      - Calvin Gross
Start Date  - 10/30/26
Title       - Ins Simulation Noise Maker
Project     - Integrated Navigation System (GNSS + IMU) -- Senior Project --
Description - This noise maker takes measurements that perfectly follow a 
            trajectory from a csv and adds noise to them.
*/

#include "ins_types.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <random>
#include <cmath>

#define DEFAULT_DT 1.0/208.0

// struct to hold current bias and timestamps for processing
struct csv_processing_state {
    float accel_bias_x, accel_bias_y, accel_bias_z;
    float gyro_bias_x, gyro_bias_y, gyro_bias_z;
    float baro_bias;
    uint64_t prev_timestamp;
    uint64_t prev_imu_timestamp;
    bool processed_timestamps = false;
    bool processed_imu_timestamps = false;
};


// default vgr configurations (chatgpt gathered example values from datasheets)
ins::SimV1Configuration config{
    // Accelerometer white-noise intensity: m^2/s^3
    .accel_vgr_x = 3.08e-7f,
    .accel_vgr_y = 3.08e-7f,
    .accel_vgr_z = 3.08e-7f,

    // Gyroscope white-noise intensity: rad^2/s
    .gyro_vgr_x = 2.44e-9f,
    .gyro_vgr_y = 2.44e-9f,
    .gyro_vgr_z = 2.44e-9f,

    // Magnetometer sample variance: uT^2
    .mag_var_x = 0.1024f,
    .mag_var_y = 0.1024f,
    .mag_var_z = 0.1681f,

    // Barometer altitude sample variance: m^2
    .baro_var_z = 0.0367f,

    // Assumed accelerometer bias random walk: m^2/s^5
    .accel_bias_vgr_x = 1.0e-6f,
    .accel_bias_vgr_y = 1.0e-6f,
    .accel_bias_vgr_z = 1.0e-6f,

    // Assumed gyroscope bias random walk: rad^2/s^3
    .gyro_bias_vgr_x = 1.0e-8f,
    .gyro_bias_vgr_y = 1.0e-8f,
    .gyro_bias_vgr_z = 1.0e-8f,

    // Assumed barometer bias random walk: m^2/s
    .baro_bias_vgr_z = 1.0e-4f,

    .gnss_default_var_x = 4.5084,
    .gnss_default_var_y = 4.5084,
    .gnss_default_var_z = 18.0337
};

// seed for random numbers
std::mt19937 generator(37);

// fixed distributions
std::normal_distribution<float> baro_z_noise(0.0f, std::sqrt(config.baro_var_z));
std::normal_distribution<float> mag_x_noise(0.0f, std::sqrt(config.mag_var_x));
std::normal_distribution<float> mag_y_noise(0.0f, std::sqrt(config.mag_var_y));
std::normal_distribution<float> mag_z_noise(0.0f, std::sqrt(config.mag_var_z));


int process_next_line(
    std::ofstream* noisy, 
    std::ifstream* perfect, 
    csv_processing_state* cur_state
) {
    std::string next_line;
    if (!std::getline(*perfect, next_line)) return 1;

    // read line into vector
    std::string mea_type;
    std::istringstream row(next_line);
    std::string field;

    std::vector<float> pm;              // perfect measurements

    std::getline(row, mea_type, ',');
    std::getline(row, field, ',');
    uint64_t timestamp = std::stoull(field);

    while (std::getline(row, field, ',')) {
        if (!field.empty()) pm.push_back(std::stof(field));
        else pm.push_back(0.0f);
    }

    // calc dt
    float dt;
    if (!cur_state->processed_timestamps) {
        dt = 0.0f;
        cur_state->processed_timestamps = true;
    }
    else dt = (timestamp - cur_state->prev_timestamp) * 0.000001;
    cur_state->prev_timestamp = timestamp;

    if (dt > 0) {
        // calc normal distributions for bias using dt
        std::normal_distribution<float> accel_bias_x_noise(0.0f, std::sqrt(config.accel_bias_vgr_x * dt));
        std::normal_distribution<float> accel_bias_y_noise(0.0f, std::sqrt(config.accel_bias_vgr_y * dt));
        std::normal_distribution<float> accel_bias_z_noise(0.0f, std::sqrt(config.accel_bias_vgr_z * dt));
        std::normal_distribution<float> gyro_bias_x_noise(0.0f, std::sqrt(config.gyro_bias_vgr_x * dt));
        std::normal_distribution<float> gyro_bias_y_noise(0.0f, std::sqrt(config.gyro_bias_vgr_y * dt));
        std::normal_distribution<float> gyro_bias_z_noise(0.0f, std::sqrt(config.gyro_bias_vgr_z * dt));
        std::normal_distribution<float> baro_bias_noise(0.0f, std::sqrt(config.baro_bias_vgr_z * dt));

        // add noise to all bias based on dt
        cur_state->accel_bias_x += accel_bias_x_noise(generator);
        cur_state->accel_bias_y += accel_bias_y_noise(generator);
        cur_state->accel_bias_z += accel_bias_z_noise(generator);
        cur_state->gyro_bias_x += gyro_bias_x_noise(generator);
        cur_state->gyro_bias_y += gyro_bias_y_noise(generator);
        cur_state->gyro_bias_z += gyro_bias_z_noise(generator);
        cur_state->baro_bias += baro_bias_noise(generator);
    }

    // define processing for each type of measurement and output new line to noisy
    *noisy << mea_type << "," << std::to_string(timestamp) << ",";

    // accelerometer and gyroscope measurement
    if (mea_type == "AG") {
        float dt_from_last_imu;
        if (!cur_state->processed_imu_timestamps) {
            dt_from_last_imu = DEFAULT_DT;
            cur_state->processed_imu_timestamps = true;
        }
        else dt_from_last_imu = (timestamp - cur_state->prev_imu_timestamp) * 0.000001;
        cur_state->prev_imu_timestamp = timestamp;

        // calc normal dist for accel and gyro measurements using dt_from_last_imu
        std::normal_distribution<float> accel_x_noise(0.0f, std::sqrt(config.accel_vgr_x / dt_from_last_imu));
        std::normal_distribution<float> accel_y_noise(0.0f, std::sqrt(config.accel_vgr_y / dt_from_last_imu));
        std::normal_distribution<float> accel_z_noise(0.0f, std::sqrt(config.accel_vgr_z / dt_from_last_imu));
        std::normal_distribution<float> gyro_x_noise(0.0f, std::sqrt(config.gyro_vgr_x / dt_from_last_imu));
        std::normal_distribution<float> gyro_y_noise(0.0f, std::sqrt(config.gyro_vgr_y / dt_from_last_imu));
        std::normal_distribution<float> gyro_z_noise(0.0f, std::sqrt(config.gyro_vgr_z / dt_from_last_imu));

        // add noisy measurements in proper order
        *noisy << pm[0] + accel_x_noise(generator) + cur_state->accel_bias_x << ",";
        *noisy << pm[1] + accel_y_noise(generator) + cur_state->accel_bias_y << ",";
        *noisy << pm[2] + accel_z_noise(generator) + cur_state->accel_bias_z << ",";
        *noisy << pm[3] + gyro_x_noise(generator) + cur_state->gyro_bias_x << ",";
        *noisy << pm[4] + gyro_y_noise(generator) + cur_state->gyro_bias_y << ",";
        *noisy << pm[5] + gyro_z_noise(generator) + cur_state->gyro_bias_z << ",,,,,,,,,,\n";

    // barometer measurement
    } else if (mea_type == "BARO") {
        *noisy << ",,,,,,,,,,,,,,," << pm[15] + baro_z_noise(generator) + cur_state->baro_bias << "\n";

    // magnetometer measurement
    } else if (mea_type == "MAG") {
        *noisy << ",,,,,," << pm[6] + mag_x_noise(generator) << ",";
        *noisy << pm[7] + mag_y_noise(generator) << ",";
        *noisy << pm[8] + mag_z_noise(generator) << ",,,,,,,\n";

    // gps measurement
    } else if (mea_type == "GNSS") {
        const float var_x = (pm[12] == 0.0f) ? config.gnss_default_var_x : pm[12];
        const float var_y = (pm[13] == 0.0f) ? config.gnss_default_var_y : pm[13];
        const float var_z = (pm[14] == 0.0f) ? config.gnss_default_var_z : pm[14];

        std::normal_distribution<float> gps_x_noise(0.0f, std::sqrt(var_x));
        std::normal_distribution<float> gps_y_noise(0.0f, std::sqrt(var_y));
        std::normal_distribution<float> gps_z_noise(0.0f, std::sqrt(var_z));

        *noisy << ",,,,,,,,," << pm[9] + gps_x_noise(generator) << ",";
        *noisy << pm[10]+gps_y_noise(generator) << ",";
        *noisy << pm[11]+gps_z_noise(generator) << ",";
        *noisy << pm[12] << "," << pm[13] << "," << pm[14] << ",\n";
    } else return 2;
    return 0;
}



int main(void) {
    std::string file_name;
    std::cout << "What trajectory file should I add noise too?: ";
    std::getline(std::cin, file_name);
    std::ifstream perf_mea(file_name);

    if(!perf_mea) {
        std::cerr << "No file named: " << file_name << "\n";
        return 1;
    }

    // start file to write noisy measurements to
    std::string new_file_name;
    if (file_name.ends_with(".csv")){
        new_file_name = file_name.substr(0, file_name.size() - 4) + "_noisy.csv";
    } else {
        new_file_name = file_name + "_noisy.csv";
    }
    std::ofstream noisy_mea(new_file_name);
    if(!noisy_mea) {
        std::cerr << "Could not create file: " << new_file_name << "\n";
        return 1;
    }

    // copy csv header from prev trajectory csv to new noisy trajectory csv
    std::string csv_header;
    std::getline(perf_mea, csv_header);
    noisy_mea << csv_header << "\n";

    csv_processing_state starting_state = {};

    // process every line and build the noisy measurement file.
    while (process_next_line(&noisy_mea, &perf_mea, &starting_state) == 0);
}
