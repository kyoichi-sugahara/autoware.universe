#!/usr/bin/env python3

# Copyright 2024 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import os
import re

import matplotlib.pyplot as plt


def find_latest_log_file(directory):
    log_files = [
        f
        for f in os.listdir(directory)
        if f.startswith("test_trajectory_follower_node_") and f.endswith(".log")
    ]
    if not log_files:
        raise FileNotFoundError("No log files found in the specified directory.")
    return max(log_files, key=lambda f: int(f.split("_")[-1].split(".")[0]))


def extract_calculation_times(log_file_path):
    with open(log_file_path, "r") as file:
        log_data = file.read()

    osqp_pattern = r"\[controller\.lateral_controller\]: executeOptimization time = (\d+) \[ns\]"
    cgmres_pattern = (
        r"\[controller\.lateral_controller\]: executeOptimization \(cgmres\) time = (\d+) \[ns\]"
    )
    # generate_mpc_matrix_pattern = (
    #     r"\[controller\.lateral_controller\]: generateMPCMatrix time = (\d+) \[ns\]"
    # )

    osqp_times = re.findall(osqp_pattern, log_data)
    cgmres_times = re.findall(cgmres_pattern, log_data)
    # generate_mpc_matrix_times = re.findall(generate_mpc_matrix_pattern, log_data)

    osqp_times = [int(time) / 1000000 for time in osqp_times]  # Convert nsec to msec
    cgmres_times = [int(time) / 1000000 for time in cgmres_times]  # Convert nsec to msec
    # generate_mpc_matrix_times = [
    #     int(time) / 1000000 for time in generate_mpc_matrix_times
    # ]  # Convert nsec to msec

    return osqp_times, cgmres_times
    # return osqp_times, cgmres_times, generate_mpc_matrix_times


# def plot_calculation_times(osqp_times, cgmres_times, generate_mpc_matrix_times, log_file_name):
def plot_calculation_times(osqp_times, cgmres_times, log_file_name):
    plt.figure(figsize=(10, 6))
    plt.plot(range(len(osqp_times)), osqp_times, label="OSQP")
    plt.plot(range(len(cgmres_times)), cgmres_times, label="CGMRES")
    # plt.plot(
    #     range(len(generate_mpc_matrix_times)), generate_mpc_matrix_times, label="generateMPCMatrix"
    # )
    plt.xlabel("Iteration")
    plt.ylabel("Calculation Time [ms]")  # Change the unit to msec
    plt.title(f"OSQP vs CGMRES vs generateMPCMatrix Calculation Time\nLog File: {log_file_name}")
    plt.legend()
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Plot OSQP, CGMRES, and generateMPCMatrix calculation times from a log file."
    )
    parser.add_argument(
        "log_path", help="Directory containing the log files or path to a specific log file."
    )
    args = parser.parse_args()

    if os.path.isdir(args.log_path):
        log_file_name = find_latest_log_file(args.log_path)
        log_file_path = os.path.join(args.log_path, log_file_name)
    elif os.path.isfile(args.log_path):
        log_file_path = args.log_path
        log_file_name = os.path.basename(args.log_path)
    else:
        raise ValueError("Invalid log path. Please provide a directory or a file path.")

    osqp_times, cgmres_times = extract_calculation_times(log_file_path)
    # osqp_times, cgmres_times, generate_mpc_matrix_times = extract_calculation_times(log_file_path)
    plot_calculation_times(osqp_times, cgmres_times, log_file_name)
    # plot_calculation_times(osqp_times, cgmres_times, generate_mpc_matrix_times, log_file_name)
