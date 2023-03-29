#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file Collect_evaluation_result.py
@author Yanwei Du (duyanwei0702@gmail.com)
@date 07-20-2022
@version 1.0
@license Copyright (c) 2022
@desc None
"""

import glob
import json
import numpy as np
import os
import zipfile


SeqNameList = [
    "MH_01_easy",
    "MH_02_easy",
    "MH_03_medium",
    "MH_04_difficult",
    "MH_05_difficult",
    "V1_01_easy",
    "V1_02_medium",
    "V1_03_difficult",
    "V2_01_easy",
    "V2_02_medium",
    "V2_03_difficult",
]

Result_root = os.path.join(os.environ["SLAM_RESULT"], "gf_orb_slam2/EuRoC/GFGG/")
Number_GF_List = [800]
NumRepeating = 5
SleepTime = 1  # 10 # 25 # second
SpeedPool = [1.0]  # , 2.0, 3.0] #, 4.0, 5.0]  # x
ResultFiles = [
    "AllFrameTrajectory",
    "KeyFrameTrajectory",
]

RMSE_THRESH = 10.0  # m
TRACKING_THRESH = 0.6
DataFreq = 20.0
SeqDuration = {
    "MH_01_easy": 182,
    "MH_02_easy": 150,
    "MH_03_medium": 132,
    "MH_04_difficult": 99,
    "MH_05_difficult": 111,
    "V1_01_easy": 143,
    "V1_02_medium": 84,
    "V1_03_difficult": 105,
    "V2_01_easy": 112,
    "V2_02_medium": 115,
    "V2_03_difficult": 115,
}


# ----------------------------------------------------------------------------------------------------------------------

class bcolors:
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    ALERT = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"

for ri, num_gf in enumerate(Number_GF_List):

    Experiment_prefix = "ObsNumber_" + str(int(num_gf))

    rmse_table = np.full((len(ResultFiles), len(SpeedPool), len(SeqNameList), NumRepeating), -1.0)

    mean_timing_table = np.full((len(SpeedPool), len(SeqNameList), NumRepeating), -1.0)
    median_timing_table = np.full((len(SpeedPool), len(SeqNameList), NumRepeating), -1.0)

    for speed_idx, speed in enumerate(SpeedPool):

        Result_root_speed = Result_root + "_Speedx" + str(speed)
        speed_str = str(speed)

        for iteration in range(0, NumRepeating):

            Experiment_dir = os.path.join(Result_root_speed, Experiment_prefix + "_Round" + str(iteration + 1))

            for sn, sname in enumerate(SeqNameList):

                SeqName = SeqNameList[sn]  # + '_blur_5'

                # print(
                #     bcolors.OKGREEN
                #     + f"Seq: {SeqName}; Feature: {num_gf}; Speed: {speed}; Round: {iteration + 1}"
                #     + bcolors().ENDC
                # )

                # collect rmse
                for i, result_name in enumerate(ResultFiles):
                    rmse_file = os.path.join(Experiment_dir, SeqName + "_" + result_name + ".zip")
                    if not os.path.exists(rmse_file):
                        print(f"cound not find rmse file: {rmse_file}")
                        continue
                    # read
                    with zipfile.ZipFile(rmse_file, "r") as z:
                        with z.open("stats.json") as f:
                            data = f.read()
                            rmse_table[i, speed_idx, sn, iteration] = json.loads(data)["rmse"]
                    # tracking failure
                    if "KeyFrame" in result_name:  # skip keyframe case
                        continue
                    file_est = rmse_file.replace(".zip", ".txt")
                    est_poses = np.loadtxt(file_est)
                    tracking_ratio = est_poses.shape[0] / (SeqDuration[SeqName] * DataFreq)
                    if tracking_ratio < TRACKING_THRESH:
                        rmse_table[i, speed_idx, sn, iteration] = -1.0
                        print(
                            f"tracking failed: Feature {num_gf}, Speed {speed}, Seq {SeqName}, Round {iteration+1}, TR {tracking_ratio}"
                        )
                    # record tracking latency
                    file_stats = os.path.join(Experiment_dir, SeqName + '_stats.txt')
                    if not os.path.exists(file_stats):
                        print(f"{file_stats} does NOT exist, take the current experiment as failure")
                        print(f"tracking failed: Feature {num_gf}, Speed {speed}, Seq {SeqName}, Round {iteration+1}")
                        rmse_table[i, speed_idx, sn, iteration] = -1.0
                        continue
                    stats = np.loadtxt(file_stats)
                    mean_timing_table[speed_idx, sn, iteration] = stats[2]
                    median_timing_table[speed_idx, sn, iteration] = stats[3]

    mean_timing_table = mean_timing_table.reshape(-1, NumRepeating)
    median_timing_table = median_timing_table.reshape(-1, NumRepeating)
    # save rmse
    for i, result_name in enumerate(ResultFiles):
        output = Result_root + "_" + str(num_gf) + "_" + result_name + ".txt"
        mn, nn, pn, qn = rmse_table.shape
        # the extra two column stores mean and median rmse
        cur_table = np.full((nn * pn, qn + 4), -1.0)
        cur_table[:, 0:qn] = rmse_table[i, :, :, :].reshape(nn * pn, qn)
        for row in range(cur_table.shape[0]):
            indices = cur_table[row, :] > 0.0
            if np.sum(indices) == qn:  # make sure every sequence succeeds
                cur_table[row, qn] = np.mean(cur_table[row, indices])
                cur_table[row, qn + 1] = np.median(cur_table[row, indices])
                cur_table[row, qn + 2] = np.mean(mean_timing_table[row, indices[:-4]])
                cur_table[row, qn + 3] = np.median(median_timing_table[row, indices[:-4]])
            # else:
            # cur_table[row, qn] = -1
            # cur_table[row, qn + 1] = -1
        # Round1, ..., RoundN, mean_rmse, median_rmse, mean_timing, median_timing
        np.savetxt(output, cur_table, fmt="%.6f", delimiter=",")

        # for visualization
        output = output.replace(".txt", "_vis.txt")
        final_table = np.full((pn + 4, nn), -1.0)
        for col in range(final_table.shape[1]):
            start_ind = 0 + pn * col
            end_ind = start_ind + pn
            final_table[0:pn, col] = cur_table[start_ind:end_ind, qn]
            temp_timing = cur_table[start_ind:end_ind, qn + 2 :]
            indices = final_table[:, col] > 0.0
            if np.sum(indices) == 0:
                continue
            final_table[pn, col] = np.mean(final_table[indices, col])
            final_table[pn + 1, col] = np.sum(indices) / pn
            final_table[pn + 2, col] = np.mean(temp_timing[indices[:-4], 0])
            final_table[pn + 3, col] = np.median(temp_timing[indices[:-4], 1])
        # MH01, MH02, ..., V203, mean_rmse, median_rmse, mean_timing, median_timing
        np.savetxt(output, np.transpose(final_table), fmt="%.6f", delimiter=",")
