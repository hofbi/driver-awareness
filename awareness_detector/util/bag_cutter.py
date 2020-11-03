#!/usr/bin/env python3
"""Helper tool to remove waiting time at beggining of bag if car did not drive yet"""

import rosbag
import time
import argparse
import os
import rospkg

DEFAULT_OUTPUT_DIR = os.path.join(
    rospkg.RosPack().get_path("awareness_detector"), "output_cutter"
)
DEFAULT_INPUT_DIR = os.getcwd()


def main():
    """Main function of bag cutter helper script"""

    args = parse_arguments()

    bagfiles_paths = []
    bagfile_names = []
    for file in os.listdir(args.input_folder):
        if file.endswith(".bag"):
            bagfiles_paths.append(os.path.join(args.input_folder, file))
            bagfile_names.append(file)

    if len(bagfiles_paths) == 0:
        print("No bag files found in " + args.input_folder)

    output_dir = args.output_folder
    os.makedirs(output_dir, exist_ok=True)

    for idx, bagfile in enumerate(bagfiles_paths):
        print("Opening " + bagfile + ".")
        outputfile_path = os.path.join(
            output_dir, "cutted_" + os.path.basename(bagfile)
        )
        if os.path.exists(outputfile_path):
            print("Output file already exists. Skipping bag..")
            continue
        bag = rosbag.Bag(bagfile, "r")
        print(
            "Start time: "
            + str(bag.get_start_time())
            + " End time: "
            + str(bag.get_end_time())
            + " Duration: "
            + str(bag.get_end_time() - bag.get_start_time())
        )
        out_bag = rosbag.Bag(outputfile_path, "w")
        start_writing = False
        num_messages = bag.get_message_count()

        i = 0
        num_removed = 0
        vehicle_info_msg = None
        vehicle_info_time = None
        write_vehicle_info_once = True
        vehicle_info_topic = "/carla/hero/vehicle_info"
        time_500_start = time.time()
        for topic, msg, timestamp in bag.read_messages():
            if i % 1000 == 0:
                time_500_end = time.time()
                duration = time_500_end - time_500_start
                duration_one = duration / 1000.0
                time_left = (num_messages - i) * duration_one
                print(
                    "Estimated time left: "
                    + str(int(time_left / 60))
                    + " minutes and "
                    + str(int(time_left % 60))
                    + " seconds."
                )
                print(
                    "File ["
                    + str(idx)
                    + "/"
                    + str(len(bagfiles_paths))
                    + "] Message ["
                    + str(i)
                    + "/"
                    + str(num_messages)
                    + "]"
                )
                time_500_start = time.time()
            i += 1
            if topic == vehicle_info_topic:
                vehicle_info_msg = msg
                vehicle_info_time = timestamp

            if start_writing:
                if write_vehicle_info_once:
                    print("Wrote vehicle_info message")
                    out_bag.write(
                        vehicle_info_topic, vehicle_info_msg, t=vehicle_info_time
                    )
                    write_vehicle_info_once = False
                out_bag.write(topic, msg, t=timestamp)
            else:
                if topic == "/carla/hero/vehicle_status":
                    if msg.control.throttle > 0.3:
                        start_writing = True
                        print(
                            "Start writing from "
                            + str(timestamp.to_sec())
                            + " seconds. Removing "
                            + str(timestamp.to_sec() - bag.get_start_time())
                            + " seconds."
                        )
                        num_removed = i

        bag.close()
        out_bag.close()
        print(bagfile + " done. Removed " + str(num_removed) + " messages.")


def parse_arguments():
    """Parser for commmand line arguments"""
    parser = argparse.ArgumentParser(
        description="Remove beginning of bag recording if car is not driving yet",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "-i",
        "--input_folder",
        type=str,
        help="Input folder of bag data",
        default=DEFAULT_INPUT_DIR,
    )

    parser.add_argument(
        "-o",
        "--output_folder",
        type=str,
        help="Output folder of bag data",
        default=DEFAULT_OUTPUT_DIR,
    )

    return parser.parse_args()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
