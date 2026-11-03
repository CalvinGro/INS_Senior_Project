/*
Author      - Calvin Gross
Start Date  - 11/3/26
Title       - Simulation Version 1
Project     - Integrated Navigation System (GNSS + IMU) -- Senior Project --
Description - This file uses a provided csv of noisy measurements to estimate a system's actual states
            It then builds a csv of the estimated trajectory based off these readings.
*/

#include "ins_types.h"
#include "ins_sim_v1.h"

#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <string>

class Simulation_1 
{
    ins::SimV1Configuration config;
    ins::InsSimV1 sim_ins;

public:
    Simulation_1()
       : config(ins::default_config),
        sim_ins(ins::InsSimV1(&config))
    {};

    
    ins::SimStatus build_trajectory(void) {
        // receive measurement csv file path
        std::cout << "\nWhat is the path to the noisy measurement file?: ";

        std::string mea_file_path;
        std::getline(std::cin, mea_file_path);
        std::ifstream measurements(mea_file_path);

        // ignore header
        std::string header;
        std::getline(measurements, header);

        if(!measurements) {
            std::cerr << "No file named: " << mea_file_path << "\n";
            return ins::SimStatus::no_input_file_for_trajectory_building;
        }

        // start file to write trajectory to
        std::string new_file_name;
        if (mea_file_path.ends_with(".csv")){
            new_file_name = mea_file_path.substr(0, mea_file_path.size() - 4) + "_traj.csv";
        } else {
            new_file_name = mea_file_path + "_traj.csv";
        }

        std::ofstream new_traj(new_file_name);
        if(!new_traj) {
            std::cerr << "Could not create file: " << new_file_name << "\n";
            return ins::SimStatus::no_output_file_created_for_trajectory_building;
        }

        // write the trajectory header
        new_traj << "timestamp,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,orient_x,orient_y,orient_z,orient_w\n";

        // apply all the measurements to the eskf and record the state after each one
        uint64_t mea_timestamp;
        while (apply_next_measurement(&measurements, &mea_timestamp) != ins::SimStatus::no_more_measurements_to_apply) {
            ins::SimStatus record_status = record_cur_state(&new_traj, mea_timestamp);
            if (record_status != ins::SimStatus::ok) {
                return record_status;
            }
        }
        return ins::SimStatus::ok;
    };


    ins::SimStatus list_errors(void) {
        for (int i = 0; i < sim_ins.loe_i; i++) {
            ins::InsStatus err_status = sim_ins.list_of_errors[i];

            switch (err_status) {
            case ins::InsStatus::ok:
                std::cout << "ok\n";
                break;

            // accel + gyro prediction errors
            case ins::InsStatus::dt_of_0_for_ag:
                std::cout << "dt_of_0_for_ag\n";
                break;
            case ins::InsStatus::escm_uninit_or_not_finite_ag:
                std::cout << "escm_uninit_or_not_finite_ag\n";
                break;
            case ins::InsStatus::ns_uninit_or_not_finite_ag:
                std::cout << "ns_uninit_or_not_finite_ag\n";
                break;
            case ins::InsStatus::ns_quaternion_not_unit_legth_ag:
                std::cout << "ns_quaternion_not_unit_legth_ag\n";
                break;
            case ins::InsStatus::ns_overflow_ag:
                std::cout << "ns_overflow_ag\n";
                break;
            case ins::InsStatus::sample_not_finite_ag:
                std::cout << "sample_not_finite_ag\n";
                break;

            // gnss errors
            case ins::InsStatus::escm_uninit_or_not_finite_gnss:
                std::cout << "escm_uninit_or_not_finite_gnss\n";
                break;
            case ins::InsStatus::ns_uninit_or_not_finite_gnss:
                std::cout << "ns_uninit_or_not_finite_gnss\n";
                break;
            case ins::InsStatus::ns_quaternion_not_unit_legth_gnss:
                std::cout << "ns_quaternion_not_unit_legth_gnss\n";
                break;
            case ins::InsStatus::sample_not_finite_gnss:
                std::cout << "sample_not_finite_gnss\n";
                break;

            // barometer errors
            case ins::InsStatus::escm_uninit_or_not_finite_b:
                std::cout << "escm_uninit_or_not_finite_b\n";
                break;
            case ins::InsStatus::ns_uninit_or_not_finite_b:
                std::cout << "ns_uninit_or_not_finite_b\n";
                break;
            case ins::InsStatus::ns_quaternion_not_unit_legth_b:
                std::cout << "ns_quaternion_not_unit_legth_b\n";
                break;
            case ins::InsStatus::sample_not_finite_b:
                std::cout << "sample_not_finite_b\n";
                break;

            // magnetometer errors
            case ins::InsStatus::escm_uninit_or_not_finite_m:
                std::cout << "escm_uninit_or_not_finite_m\n";
                break;
            case ins::InsStatus::ns_uninit_or_not_finite_m:
                std::cout << "ns_uninit_or_not_finite_m\n";
                break;
            case ins::InsStatus::ns_quaternion_not_unit_legth_m:
                std::cout << "ns_quaternion_not_unit_legth_m\n";
                break;
            case ins::InsStatus::sample_not_finite_m:
                std::cout << "sample_not_finite_m\n";
                break;

            // apply measurement errors
            case ins::InsStatus::timestamps_out_of_order_applyMeasurement:
                std::cout << "timestamps_out_of_order_applyMeasurement\n";
                break;

            // free-list errors
            case ins::InsStatus::all_nodes_full:
                std::cout << "all_nodes_full\n";
                break;
            case ins::InsStatus::attempted_to_acquire_invalid_index:
                std::cout << "attempted_to_acquire_invalid_index\n";
                break;
            case ins::InsStatus::attempted_to_release_invalid_index:
                std::cout << "attempted_to_release_invalid_index\n";
                break;

            // delayed state FR errors
            case ins::InsStatus::recorded_states_uninit:
                std::cout << "recorded_states_uninit\n";
                break;
            case ins::InsStatus::bi_search_failed_in_findRecalcCheckpoint:
                std::cout << "bi_search_failed_in_findRecalcCheckpoint\n";
                break;
            case ins::InsStatus::indexed_past_last_measurement:
                std::cout << "indexed_past_last_measurement\n";
                break;
            case ins::InsStatus::invalid_state_index_removeLastState:
                std::cout << "invalid_state_index_removeLastState\n";
                break;
            case ins::InsStatus::unlinked_state_removeLastState:
                std::cout << "unlinked_state_removeLastState\n";
                break;
            case ins::InsStatus::bi_search_found_wrong_state:
                std::cout << "bi_search_found_wrong_state\n";
                break;
            case ins::InsStatus::failed_to_add_first_state:
                std::cout << "failed_to_add_first_state\n";
                break;
            case ins::InsStatus::measurement_out_of_fr_range:
                std::cout << "measurement_out_of_fr_range\n";
                break;

            // error overflow
            case ins::InsStatus::loe_overflow:
                std::cout << "loe_overflow\n";
                break;

            default:
                std::cout << "unknown InsStatus\n";
                break;
        }
        }
        return ins::SimStatus::ok;
    };

private:
    ins::SimStatus apply_next_measurement(std::ifstream* measurements, uint64_t* mea_timestamp) {
        // check if there is a next measurement
        std::string next_line;
        if(!std::getline(*measurements, next_line)) {
            return ins::SimStatus::no_more_measurements_to_apply;
        }

        // read line into vector
        std::string mea_type;
        std::istringstream row(next_line);
        std::string field;

        std::vector<float> mea_vect;

        // determine the measurement type
        std::getline(row, mea_type, ',');

        std::getline(row, field, ',');
        uint64_t timestamp = std::stoull(field);
        *mea_timestamp = timestamp;

        while (std::getline(row, field, ',')) {
            if (!field.empty()) mea_vect.push_back(std::stof(field));
            else mea_vect.push_back(0.0f);
        }

        // 15 value vector for all measurements except baro which has 16
        if (!(mea_vect.size() == 15 || mea_vect.size() == 16)) {
            return ins::SimStatus::cant_apply_malformed_measurement;
        }

        // apply the measurement based on its type
        if (mea_type == "AG") {
            ins::AGExternalV1Sample sample = {
                .accel_x = mea_vect[0],
                .accel_y = mea_vect[1], 
                .accel_z = mea_vect[2],
                .ang_vel_x = mea_vect[3],
                .ang_vel_y = mea_vect[4],
                .ang_vel_z = mea_vect[5],
                .timestamp = timestamp,
            };
            sim_ins.receiveV1AccelGyro(sample);

        } else if (mea_type == "BARO") {
            ins::BaroExternalV1Sample sample = {
                .z_pos = mea_vect[15],
                .timestamp = timestamp,
            };
            sim_ins.receiveV1Baro(sample);

        } else if (mea_type == "MAG") {
            ins::MagExternalV1Sample sample = {
                .ang_pos_x = mea_vect[6],
                .ang_pos_y = mea_vect[7],
                .ang_pos_z = mea_vect[8],
                .timestamp = timestamp,
            };
            sim_ins.receiveV1Mag(sample);

        } else if (mea_type == "GNSS") {
            ins::GnssExternalV1Sample sample = {
                .pos_x = mea_vect[9],
                .pos_y = mea_vect[10],
                .pos_z = mea_vect[11],
                .gnss_var_x = mea_vect[12],
                .gnss_var_y = mea_vect[13],
                .gnss_var_z = mea_vect[14],
                .timestamp = timestamp,
            };
            sim_ins.receiveV1Gnss(sample);
        } else {
            return ins::SimStatus::cant_apply_malformed_measurement;
        }
        return ins::SimStatus::ok;
    };

    // assumes that the new trajectory file already has a header and is on
    // a newline already.
    ins::SimStatus record_cur_state(std::ofstream* new_traj, uint64_t timestamp) {
        if (new_traj == nullptr || !new_traj->is_open()) {
            return ins::SimStatus::file_not_open_for_recording_states;
        }
        ins::NStateVec ns;
        ins::InsStatus get_status = sim_ins.getCurrentState(ns);
        if (get_status != ins::InsStatus::ok) return ins::SimStatus::get_failed;
        
        *new_traj << timestamp << ',';
        *new_traj << ns.position[0] << ',' << ns.position[1] << ',' << ns.position[2] << ',';
        *new_traj << ns.velocity[0] << ',' << ns.velocity[1] << ',' << ns.velocity[2] << ',';
        *new_traj << ns.angular_position.x() << ',' << ns.angular_position.y() << ',';
        *new_traj << ns.angular_position.z() << ',' << ns.angular_position.w() << '\n';
        return ins::SimStatus::ok;
    };
};


int main(void) {
    Simulation_1 sim = Simulation_1();

    sim.build_trajectory();
    sim.list_errors();
};