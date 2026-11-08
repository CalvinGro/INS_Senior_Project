"""
Author      - Calvin Gross
Start Date  - 11/1/26
Title       - Ins Simulation Measurement Creator
Project     - Integrated Navigation System (GNSS + IMU) -- Senior Project --
Description - This takes in a generated csv of a trajectory and creates v1 perfect measurements
            for each sensor based on it. Then this csv is fed to the sim_noise_maker to add bias and noise.
"""
from pathlib import Path
import pandas as pd
from typing import cast
import csv
from scipy.spatial.transform import Rotation
import numpy as np

class MeasurementGenerator:

    # sampling rates in microseconds
    _baro_sampling_rate = 1e6 / 50
    _mag_sampling_rate = 1e6 / 60
    _gps_sampling_rate = 1e6 / 20
    _ag_sampling_rate = 1e6 / 208


    def __init__(self, trajectory_csv):
        traj_path = Path(trajectory_csv)
        self.trajectory_csv = traj_path
        self.measurement_csv = traj_path.with_name(f"{traj_path.stem}_measurements{traj_path.suffix}")
        self.generator = np.random.default_rng(29)    # generator for randomizing gps variance

    
    def generate_measurements(self):
        # advance through true trajectory readings and call correct generate mea method on rows based
        # on the sampling rates of each sensor.
        traj_df = pd.read_csv(self.trajectory_csv)
        # last measurement of each given type in microseconds
        last_baro = 0
        last_mag = 0
        last_gps = 0
        last_ag = 0

        with self.measurement_csv.open("w", newline="", encoding="utf-8") as mea_file:
            writer = csv.writer(mea_file)
            writer.writerow(["mea_type","timestamp","ag_accel_x","ag_accel_y","ag_accel_z",
                            "ag_ang_vel_x","ag_ang_vel_y","ag_ang_vel_z","mag_field_x","mag_field_y","mag_field_z",
                            "gps_pos_x","gps_pos_y","gps_pos_z","gps_var_x","gps_var_y","gps_var_z","baro_z"])

            for i, row in enumerate(traj_df.itertuples(index=False)):
                time = cast(int, row.timestamp)

                # if Magnetometer
                if (self._mag_sampling_rate <= (time - last_mag)):
                    last_mag = time
                    self.add_mag_mea(row, writer)

                # if Barometer
                if (self._baro_sampling_rate <= (time - last_baro)):
                    last_baro = time
                    self.add_baro_mea(row, writer)

                # if Gps 
                if (self._gps_sampling_rate <= (time - last_gps)):
                    last_gps = time
                    self.add_gps_mea(row, writer)

                # if Accelerometer + Gyroscope
                if (self._ag_sampling_rate <= (time - last_ag)):
                    last_ag = time
                    self.add_ag_mea(i, traj_df, writer)
                



    def add_baro_mea(self, row, writer):
        # BARO,0,,,,,,,,,,,,,,,,0
        writer.writerow(["BARO",cast(int, row.timestamp),
                         "","","","","","","","","","","","","","","",
                         cast(float, row.pos_z)])
        

    def add_mag_mea(self, row, writer):
        enu_ref_to_bf = Rotation.from_quat([row.orient_x, row.orient_y, row.orient_z, row.orient_w])
        mag_body = enu_ref_to_bf.apply([0.0, 40.0, 0.0])
        # assume magnetic north is perfectly zero aligned ENU
        # MAG,12500,,,,,,,5,20,-45,,,,,,,
        writer.writerow(["MAG",cast(int, row.timestamp),"","","","","","",
             mag_body[0],mag_body[1],mag_body[2],"","","","","","",""])


    def add_gps_mea(self, row, writer):
        # generate random variances based on more normal dists for gps pos
        var_x = self.generator.normal(loc=4.5084, scale=0.45084)
        var_y = self.generator.normal(loc=4.5084, scale=0.45084)
        var_z = self.generator.normal(loc=18.0337, scale=1.80337)

        # no variances can be 0 or below
        while (var_x <= 0 or var_y <= 0 or var_z <= 0):
            var_x = self.generator.normal(loc=4.5084, scale=0.45084)
            var_y = self.generator.normal(loc=4.5084, scale=0.45084)
            var_z = self.generator.normal(loc=18.0337, scale=1.80337)

        # GNSS,0,,,,,,,,,,0,0,0,4.660779,4.731969,19.443865,
        writer.writerow(["GNSS",cast(int, row.timestamp),"","","","","","","","","",
                     cast(float, row.pos_x),cast(float, row.pos_y),cast(float, row.pos_z),
                     var_x,var_y,var_z,""])


    def add_ag_mea(self, index, traj_df, writer):
        prev = traj_df.iloc[index]
        cur = traj_df.iloc[index]
        
        if (index != 0 or index >= len(traj_df)-1):
            prev = traj_df.iloc[index-1]
        else:
            cur = traj_df.iloc[index+1]

        # calc acceleration from 2 timestamps
        
        # dt_sec can't be 0, trajectories are always 1 ms apart
        dt_sec = (cast(float, cur.timestamp) - cast(float, prev.timestamp)) * 1e-6
        accel_x = (cast(float, cur.vel_x) - cast(float, prev.vel_x)) / dt_sec
        accel_y = (cast(float, cur.vel_y) - cast(float, prev.vel_y)) / dt_sec
        accel_z = (cast(float, cur.vel_z) - cast(float, prev.vel_z)) / dt_sec

        # add gravity
        enu_ref_to_bf = Rotation.from_quat([cur.orient_x, cur.orient_y, cur.orient_z, cur.orient_w])
        
        bf_accel = enu_ref_to_bf.apply([accel_x, accel_y, accel_z + 9.80655])

        # calc angular velocity from 2 timestamps
        prev_q = Rotation.from_quat([prev.orient_x, prev.orient_y, prev.orient_z, prev.orient_w])
        cur_q = Rotation.from_quat([cur.orient_x, cur.orient_y, cur.orient_z, cur.orient_w])

        rotation_difference = prev_q * cur_q.inv()
        ang_vel = rotation_difference.as_rotvec() / dt_sec

        # AG,48077,0,0,9.80655,0,0,0,,,,,,,,,,
        writer.writerow(["AG",cast(int, cur.timestamp),
                     bf_accel[0], bf_accel[1], bf_accel[2], ang_vel[0], ang_vel[1], ang_vel[2], 
                     "","","","","","","","","",""])


if __name__ == '__main__':

    csv_path = input("Input the path to the trajectory csv: ")
    gen = MeasurementGenerator(csv_path)
    gen.generate_measurements()