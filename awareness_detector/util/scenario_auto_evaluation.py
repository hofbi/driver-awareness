#!/usr/bin/env python3
"""Helper tool to start carla scenarios one after another using the carla scenario runner"""

import argparse
import os
import subprocess
import time
from datetime import datetime

DEFAULT_OUTPUT_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "output")
)
DEFAULT_LOG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "log"))
SCENARIOS_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "scenarios")
)

EXTRA_SCENARIO_CLASSES = [
    "LeftTurnTwoVehicles",
    "ManeuverOppositeDirectionPedestrian",
    "RightTurn",
]


def open_logfile(prefix):
    """Open new log file"""
    name = os.path.join(
        DEFAULT_LOG_DIR,
        prefix + "_" + datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + ".log",
    )
    return open(name, "w+")


def main():
    """Main function of the scenario auto evaluator"""
    args = parse_arguments()

    os.makedirs(DEFAULT_LOG_DIR, exist_ok=True)

    for index, scenario_class in enumerate(args.scenarioClasses):
        scenario = scenario_class + "_" + str(args.scenarioNumber)
        print(
            "\n==============================================================================\n"
        )
        print(
            "Running scenario %s (%d/%d)"
            % (scenario, index + 1, len(args.scenarioClasses))
        )

        runner_log = open_logfile("runner")
        client_log = open_logfile("client")

        runner_args = [
            "python3",
            os.environ["SCENARIO_RUNNER_PATH"],
            "--scenario",
            scenario,
            "--waitForEgo",
            "--junit",
            "--outputDir",
            args.outputDir,
            "--reloadWorld",
        ]
        if scenario_class in EXTRA_SCENARIO_CLASSES:
            scen_dir = os.path.join(SCENARIOS_DIR, scenario_class + ".py")
            conf_dir = os.path.join(SCENARIOS_DIR, scenario_class + ".xml")
            runner_args.extend(
                ["--additionalScenario", scen_dir, "--configFile", conf_dir]
            )

        runner_proc = subprocess.Popen(runner_args, stdout=runner_log)
        time.sleep(2)
        client_proc = subprocess.Popen(
            [
                "roslaunch",
                "awareness_detector",
                "evaluation.launch",
                "role_name:=hero",
                "vehicle_filter:=vehicle.audi.etron",
                "keyboard_control:=" + str(args.keyboard),
                "bag_prefix:=" + scenario,
            ],
            stdout=client_log,
        )
        print("Waiting for runner to finish")
        runner_proc.wait()

        client_proc.terminate()
        try:
            client_proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            print("Timeout while terminating")
            client_proc.kill()

        runner_log.close()
        client_log.close()

        input("\n-> %s finished. Press enter to continue..." % scenario)

        print(
            "\n==============================================================================\n"
        )
        time.sleep(3)


def parse_arguments():
    """Parser for commmand line arguments"""
    parser = argparse.ArgumentParser(
        description="Evaluate scenarios using the carla scenario runner",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "-s",
        "--scenarioClasses",
        type=str,
        nargs="+",
        help="Scenario classes that should be used for the evaluation",
        default=[
            "FollowLeadingVehicleWithObstacle",
            "StationaryObjectCrossing",
            "DynamicObjectCrossing",
            "SignalizedJunctionLeftTurn",
            "SignalizedJunctionRightTurn",
            "ManeuverOppositeDirection",
        ],
    )
    parser.add_argument(
        "-n",
        "--scenarioNumber",
        type=int,
        default=1,
        help="Scenario number that should be used for each scenario of a class",
    )
    parser.add_argument(
        "-o",
        "--outputDir",
        default=DEFAULT_OUTPUT_DIR,
        help="ScenarioRunner output dir",
    )
    parser.add_argument(
        "-k", "--keyboard", action="store_true", help="Use keyboard controls"
    )

    return parser.parse_args()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
