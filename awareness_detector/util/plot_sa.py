"""Plot SA evaluation"""

import matplotlib.pyplot as plt
import pandas as pd
from pathlib import Path
import os
import tikzplotlib
import argparse


def plot_data(input_dir, output_dir, main_scenario, show_plot):
    """plot SA evaluation data"""
    sa_df = pd.read_csv(input_dir.joinpath(main_scenario))
    bag_frequency = 0.05
    sa_df["time"] = sa_df.index * bag_frequency

    actual = sa_df["actual_sa_mean"]
    sigma = sa_df["actual_sa_sigma"]
    act_high = actual + sigma
    act_low = actual - sigma

    plt.figure("Average SA for a turn with 3 users")
    plt.plot(sa_df["time"], sa_df["optimal_sa"], label="$SA_{opt}$")
    plt.plot(sa_df["time"], actual, label="$SA_{act}$")
    plt.fill_between(sa_df["time"], act_high, act_low, facecolor="yellow", alpha=0.5)

    plt.legend()
    plt.xlabel("Time [s]", labelpad=-5)
    plt.ylabel("SA [1]")

    if not show_plot:
        tikzplotlib.save(output_dir.joinpath("sa_measure.tex"))
        clear_plot_data()

    actual_headers = sa_df.filter(regex=r"actual_sa_\d").columns

    plt.figure("Individual SAs for a turn with 3 users", figsize=[24, 3])
    for index, name in enumerate(actual_headers):
        plt.subplot(f"2{int(len(actual_headers)/2)}{index+1}")
        plt.plot(sa_df["time"], sa_df["optimal_sa"], label="$SA_{opt}$")
        plt.plot(sa_df["time"], sa_df[name], label="$SA_{act}$")
        plt.ylim(bottom=0)
        plt.legend()
        if index >= len(actual_headers) / 2:
            plt.xlabel("Time [s]")
        if index == 0 or index == len(actual_headers) / 2:
            plt.ylabel("SA [1]", labelpad=-2)
        plt.title(f"User {index+1}", y=0.75, x=0.1)

    if not show_plot:
        tikzplotlib.save(output_dir.joinpath("sa_measure_all.tex"))
        clear_plot_data()

    plt.figure("SA ratio")
    ratio = sa_df["sa_mean"]
    ratio_sigma = sa_df["sa_sigma"]
    ratio_high = ratio + ratio_sigma
    ratio_low = ratio - ratio_sigma

    plt.plot(sa_df["time"], ratio)
    plt.fill_between(
        sa_df["time"], ratio_high, ratio_low, facecolor="yellow", alpha=0.5
    )

    plt.xlabel("Time [s]")
    plt.ylabel("SA [1]", labelpad=-5)

    if not show_plot:
        tikzplotlib.save(output_dir.joinpath("sa_ratio.tex"))
        clear_plot_data()

    print(
        "Scenario & \
        $\overline{SA}_{opt}$ & $\overline{SA}_{act}$ & $\overline{SA}_{ratio}$ ($\pm\sigma$) \\\\ \hline"  # noqa: W605
    )
    for file_name in [
        input_dir.joinpath(file_name)
        for file_name in os.listdir(input_dir)
        if file_name.endswith(".csv")
    ]:
        sa_df = pd.read_csv(file_name)
        print(
            f"{file_name.stem} & {sa_df['optimal_sa'].mean():.2f} & \
            {sa_df['actual_sa_mean'].mean():.2f} & {sa_df['sa_mean'].mean():.2f} \
            $\pm$({sa_df['sa_sigma'].mean():.2f}) \\\\"  # noqa: W605
        )


def clear_plot_data():
    """clear plot data"""
    plt.clf()
    plt.close()


def main():
    """main"""
    parser = argparse.ArgumentParser(
        description="Plot SA evaluation.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--eval_dir",
        type=str,
        default=Path(__file__).parent.joinpath("data"),
        help="Path to the sa measurements",
    )
    parser.add_argument(
        "--main_scenario",
        type=str,
        default="sa_turn_with_three_users.csv",
        help="Name of the main scenario csv file",
    )
    parser.add_argument(
        "-o",
        "--out_dir",
        type=str,
        default=Path(__file__).parent,
        help="Path to the output directory",
    )
    parser.add_argument("-s", "--show", action="store_true", help="Show the plot")

    args = parser.parse_args()
    plot_data(Path(args.eval_dir), Path(args.out_dir), args.main_scenario, args.show)

    if args.show:
        plt.show()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
