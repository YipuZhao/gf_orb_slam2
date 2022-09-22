#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file Run_EuRoC_Stereo_nonROS.py
@author Yanwei Du (duyanwei0702@gmail.com)
@date 09-19-2022
@version 1.0
@license Copyright (c) 2022
@desc None
"""


# This script is to run all the experiments in one program

import os
import subprocess
import time
import signal

DATA_ROOT = "/mnt/DATA/Datasets/EuRoC"
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

Result_root = os.path.join(os.environ["SLAM_RESULT"], "gf_orb_slam2/EuRoC/GFGG")

Number_GF_List = [260]
# [400, 600, 800, 1000, 1500, 2000]; #
# Number_GF_List = [60, 80, 100, 150, 200]
NumRepeating = 1  # 10 # 20 #  5 #
SpeedPool = [1.0]  # , 2.0, 3.0, 4.0, 5.0]  # x
SleepTime = 1  # 10 # 25
EnableViewer = 0
EnableLogging = 1

GF_ORB_SLAM2_PATH = os.path.join(os.environ["HOME"], "roboslam_ws/src/gf_orb_slam2")
ConfigPath = os.path.join(os.environ["HOME"], "roboslam_ws/src/ORB_Data")

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


for speed in SpeedPool:

    Result_root_speed = Result_root + "_Speedx" + str(speed)

    for ri, num_gf in enumerate(Number_GF_List):

        Experiment_prefix = "ObsNumber_" + str(int(num_gf))

        for iteration in range(0, NumRepeating):

            Experiment_dir = os.path.join(Result_root_speed, Experiment_prefix + "_Round" + str(iteration + 1))

            # mkdir for pose traj
            cmd_mkdir = "mkdir -p " + Experiment_dir
            subprocess.call(cmd_mkdir, shell=True)

            for sn, sname in enumerate(SeqNameList):

                SeqName = SeqNameList[sn]  # + '_blur_5'

                print(
                    bcolors.OKGREEN
                    + f"Seq: {SeqName}; Feature: {num_gf}; Speed: {speed}; Round: {iteration + 1}"
                    + bcolors().ENDC
                )

                # hardcode, might try other feature number
                file_setting = os.path.join(ConfigPath, "EuRoC_yaml/EuRoC_stereo_lmk800.yaml")

                file_vocab = os.path.join(ConfigPath, "ORBvoc.bin")

                file_traj = os.path.join(Experiment_dir, SeqName)
                file_dummy_map = file_traj + "_dummy_map.txt"
                file_log = "> " + file_traj + "_logging.txt" if EnableLogging else ""
                file_data = os.path.join(DATA_ROOT, SeqName)
                file_timestamp = os.path.join(file_data, "times.txt")
                file_node = os.path.join(GF_ORB_SLAM2_PATH, "Examples/Stereo/stereo_euroc")

                # compose cmd
                cmd_slam = (
                    file_node
                    + " "
                    + file_vocab
                    + " "
                    + file_setting
                    + " "
                    + str(num_gf)
                    + " "
                    + "0"
                    + " "
                    + str(EnableViewer)
                    + " "
                    + file_data
                    + " "
                    + file_timestamp
                    + " "
                    + file_traj
                    + " "
                    + file_dummy_map
                    + " "
                    + str(speed)
                    + " "
                    + file_log
                )

                print(bcolors.WARNING + "cmd_slam: \n" + cmd_slam + bcolors.ENDC)

                print(bcolors.OKGREEN + "Launching SLAM" + bcolors.ENDC)
                subprocess.call(cmd_slam, shell=True)

                print(bcolors.OKGREEN + "Sleeping for a few secs to wait for voc loading" + bcolors.ENDC)
                time.sleep(SleepTime)

                print(bcolors.OKGREEN + "Finished" + bcolors.ENDC)
                time.sleep(SleepTime)
                subprocess.call("pkill stereo_euroc", shell=True)
