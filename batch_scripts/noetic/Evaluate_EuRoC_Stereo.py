#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file Evaluate_EuRoC_Stereo.py
@author Yanwei Du (duyanwei0702@gmail.com)
@date 09-19-2022
@version 1.0
@license Copyright (c) 2022
@desc None
"""


import glob
import numpy as np
import os
import subprocess
import time


# This script is to run all the experiments in one program


DATA_ROOT = "/mnt/DATA/Datasets/EuRoC/"
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
SpeedPool = [1.0]  # , 2.0, 3.0] #, 4.0, 5.0]  # x
SleepTime = 1  # 10 # 25 # second
ConfigPath = os.path.join(os.environ["HOME"], "roboslam_ws/src/ORB_Data")
GT_ROOT = os.path.join(ConfigPath, "EuRoC_POSE_GT")
SENSOR = "cam0"
SaveResult = 1
ResultFiles = [
    "AllFrameTrajectory",
    "KeyFrameTrajectory",
]

# ----------------------------------------------------------------------------------------------------------------------


def call_evaluation(eval, gt, est, options, save):
    cmd_eval = eval + " " + gt + " " + est + " " + options
    if save:
        result = os.path.splitext(est)[0] + ".zip"
        cmd_eval = cmd_eval + " --save_results " + result
        if os.path.exists(result):
            cmd_rm_result = "rm " + result
            subprocess.call(cmd_rm_result, shell=True)

    print(bcolors.WARNING + "cmd_eval: \n" + cmd_eval + bcolors.ENDC)
    print(bcolors.HEADER + os.path.basename(est))
    subprocess.call(cmd_eval, shell=True)


class bcolors:
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    ALERT = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"


for speed in SpeedPool:

    Result_root_speed = Result_root + "_Speedx" + str(speed)

    for ri, num_gf in enumerate(Number_GF_List):

        Experiment_prefix = "ObsNumber_" + str(int(num_gf))

        for iteration in range(0, NumRepeating):

            Experiment_dir = os.path.join(Result_root_speed, Experiment_prefix + "_Round" + str(iteration + 1))

            for sn, sname in enumerate(SeqNameList):

                SeqName = SeqNameList[sn]  # + '_blur_5'

                print(
                    bcolors.OKGREEN
                    + f"Seq: {SeqName}; Feature: {num_gf}; Speed: {speed}; Round: {iteration + 1}"
                    + bcolors().ENDC
                )

                # create evaluation command
                file_eval = "evo_ape tum"
                options = "-va --align_origin"

                # gt file
                file_gt = os.path.join(GT_ROOT, SeqName + "_" + SENSOR + ".txt")
                if not os.path.exists(file_gt):
                    print(f"missing gt file: {file_gt}")
                    exit(-1)

                # evaluate each result file
                for result_filename in ResultFiles:
                    file_est = os.path.join(Experiment_dir, SeqName + "_" + result_filename + ".txt")
                    if not os.path.exists(file_est):
                        print(f"missing est file {file_est}")
                        continue
                    # evaluate
                    call_evaluation(file_eval, file_gt, file_est, options, SaveResult)

                print(bcolors.OKGREEN + "Finished" + bcolors.ENDC)
                time.sleep(SleepTime)
