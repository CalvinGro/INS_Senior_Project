"""
Author      - Calvin Gross
Start Date  - 11/3/26
Title       - Simulation Trajectory Comparision and Plotter
Project     - Integrated Navigation System (GNSS + IMU) -- Senior Project --
Description - This takes the true trajectory and estimated trajectory in as csv files. Then
            it compares the two by plotting them.
            Disclaimer: Most of these methods are generated with ChatGPT
"""

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.colors import Normalize
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from matplotlib.lines import Line2D
from typing import Any, cast
from mpl_toolkits.mplot3d import Axes3D

class trajectory_analysis:


    def __init__(self, true_traj_file, estimated_traj_file):
        self.true_df = pd.read_csv(true_traj_file)
        self.estimated_df = pd.read_csv(estimated_traj_file)


    def plot_line_graph(self, state):
        fig, ax = plt.subplots()

        ax.plot(self.true_df["timestamp"], self.true_df[state], label="True Trajectory")
        ax.plot(self.estimated_df["timestamp"], self.estimated_df[state], label="Estimated Trajectory")

        ax.set_xlabel("Timestamp")
        ax.set_ylabel(f"{state}")
        ax.set_title(f"{state} over time")
        ax.legend()
        ax.grid(True)

        plt.tight_layout()
        plt.show()

    def plot_trajectory(self):
            
            fig = plt.figure()
            ax = cast(Axes3D, fig.add_subplot(111, projection="3d"))

            for df, label in [
                (self.true_df, "True Trajectory"),
                (self.estimated_df, "Estimated Trajectory"),
            ]:
                ax.plot(df["pos_x"], df["pos_y"], df["pos_z"], label=label)

            ax.set_xlabel("X (m)")
            ax.set_ylabel("Y (m)")
            ax.set_zlabel("Z (m)")
            ax.legend()

            plt.tight_layout()
            plt.show()



    def plot_trajectory_with_velocity(self):
        fig = plt.figure()
        ax = cast(Axes3D, fig.add_subplot(111, projection="3d"))

        trajectories = [
            (self.true_df, "True Trajectory", "solid"),
            (self.estimated_df, "Estimated Trajectory", "dashed"),
        ]

        # Use one speed scale for both trajectories.
        speeds = [
            np.linalg.norm(df[["vel_x", "vel_y", "vel_z"]].to_numpy(), axis=1)
            for df, _, _ in trajectories
        ]
        max_speed = max(float(speed.max()) for speed in speeds)
        norm = Normalize(vmin=0, vmax=max_speed if max_speed > 0 else 1)
        cmap = plt.get_cmap("viridis")

        all_positions = []
        legend_handles = []

        for (df, label, style), speed in zip(trajectories, speeds):
            positions = df[["pos_x", "pos_y", "pos_z"]].to_numpy()
            all_positions.append(positions)

            # Connect each pair of consecutive positions.
            segments = np.stack([positions[:-1], positions[1:]], axis=1)

            line = Line3DCollection(
                segments,
                cmap=cmap,
                norm=norm,
                linewidths=2,
                linestyles=style,
            )

            # Color each segment using its average endpoint speed.
            line.set_array((speed[:-1] + speed[1:]) / 2)
            ax.add_collection3d(line)

            legend_handles.append(
                Line2D([0], [0], color="black", linestyle=style, label=label)
            )

        # Collections need explicit axis scaling.
        positions = np.vstack(all_positions)
        ax.auto_scale_xyz(positions[:, 0], positions[:, 1], positions[:, 2])

        color_scale = plt.cm.ScalarMappable(norm=norm, cmap=cmap)
        fig.colorbar(color_scale, ax=ax, label="Speed (m/s)", pad=0.1)

        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.legend(handles=legend_handles)

        plt.tight_layout()
        plt.show()



    def plot_trajectory_with_mea_type(self):
        fig = plt.figure()
        ax = cast(Axes3D, fig.add_subplot(111, projection="3d"))

        for df, label in [
            (self.true_df, "True Trajectory"),
            (self.estimated_df, "Estimated Trajectory"),
        ]:
            ax.plot(
                df["pos_x"], df["pos_y"], df["pos_z"],
                label=label, linewidth=1, alpha=0.5,
            )

        colors = {
            "AG": "tab:blue",
            "MAG": "tab:orange",
            "GNSS": "tab:green",
            "BARO": "tab:red",
        }

        for mea_type, color in colors.items():
            points = self.estimated_df[
                self.estimated_df["mea_type"] == mea_type
            ]

            if points.empty:
                continue

            ax.scatter(
                points["pos_x"], 
                points["pos_y"], 
                zs=cast(Any, points["pos_z"].to_numpy()),
                color=color,
                label=mea_type,
                s=12,
                depthshade=False,
            )

        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.legend()

        plt.tight_layout()
        plt.show()




if __name__ == '__main__':

    analysis = trajectory_analysis(
        "generated_data_sets/v1_data_sets/trajectory_data_2.csv",
        "generated_data_sets/v1_data_sets/trajectory_data_2_measurements_noisy_traj.csv"
        )
    #analysis.plot_line_graph("vel_x")
    analysis.plot_trajectory_with_mea_type()