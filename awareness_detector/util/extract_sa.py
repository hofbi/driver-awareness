#!/usr/bin/env python3
"""Helper tool to run the situational awareness detection pipeline on a folder of bag files and extract sa messages only & create a csv file with the sa data"""

import time
import argparse
import os
import subprocess

DEFAULT_OUTPUT_DIR = os.getcwd()


def terminate_ros_node(process_string):
    """Terminates ros node, used for terinating rosbag because otherwise recording is not finished correctly"""
    list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    list_output = str(list_cmd.stdout.read(), "utf-8")
    print(list_output)
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode

    for string in list_output.split("\n"):
        if string.startswith(process_string):
            print(string)
            os.system("rosnode kill " + string)


def main():
    """Main function of the scenario auto evaluator"""
    args = parse_arguments()

    bagfiles_paths = []
    bagfile_names = []
    for file in os.listdir(str(args.input_folder)):
        if file.endswith(".bag"):
            bagfiles_paths.append(os.path.join(str(args.input_folder), file))
            bagfile_names.append(file)
    if len(bagfile_names) == 0:
        print("No bag file found in path: " + str(args.input_folder))

    output_path = args.output_folder
    ouput_prefix = str(args.output_prefix)
    if not os.path.exists(output_path):
        os.makedirs(output_path)

    for idx, bagfile in enumerate(bagfiles_paths):

        print(
            "\n==============================================================================\n"
        )
        print("Running extraction for file " + str(bagfile_names[idx]))

        awareness_proc = subprocess.Popen(
            [
                "roslaunch",
                "awareness_detector",
                "carla_awareness_model.launch",
                "ego_vehicle_name:=hero",
                "vehicle_filter:=vehicle.audi.etron",
            ]
        )
        time.sleep(2)
        record_proc = subprocess.Popen(
            [
                "rosbag",
                "record",
                "-O",
                os.path.join(output_path, ouput_prefix) + str(bagfile_names[idx]),
                "/awareness_detection/sa",
            ]
        )
        time.sleep(2)
        play_proc = subprocess.Popen(["rosbag", "play", bagfile])
        print("Waiting for runner to finish")

        play_proc.wait()
        terminate_ros_node("/record")

        awareness_proc.terminate()
        try:
            awareness_proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            print("Timeout while terminating")
            awareness_proc.kill()

        print(
            "\n==============================================================================\n"
        )
        time.sleep(2)

        csv_file = open(
            os.path.join(output_path, ouput_prefix) + str(bagfile_names[idx]) + ".csv",
            "w",
        )
        csv_proc = subprocess.Popen(
            [
                "rostopic",
                "echo",
                "-b",
                os.path.join(output_path, ouput_prefix) + str(bagfile_names[idx]),
                "-p",
                "/awareness_detection/sa",
            ],
            stdout=csv_file,
        )

        csv_proc.wait()


def parse_arguments():
    """Parser for commmand line arguments"""
    parser = argparse.ArgumentParser(
        description="Extract situational awareness from bag files. Ouput are bag files with sa topic only and csv file with sa data",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "-i",
        "--input_folder",
        type=str,
        help="Input folder of bag data",
    )
    parser.add_argument(
        "-o",
        "--output_folder",
        type=str,
        help="Output folder for bag and csv data",
        default=DEFAULT_OUTPUT_DIR,
    )
    parser.add_argument(
        "-p",
        "--output_prefix",
        type=str,
        help="Prefix for the output files",
        default="SA_",
    )

    return parser.parse_args()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
