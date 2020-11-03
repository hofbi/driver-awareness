# Driver Awareness

[![Actions Status](https://github.com/hofbi/driver-awareness/workflows/CI/badge.svg)](https://github.com/hofbi/driver-awareness)
[![Actions Status](https://github.com/hofbi/driver-awareness/workflows/CodeQL/badge.svg)](https://github.com/hofbi/driver-awareness)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)

Measure the driver situation awareness based on eye tracking data.

![Driver Awareness](doc/overview.png "Driver Awareness")

## Paper

If you use this code please cite our paper.

*Measuring Driver Situation Awareness Using Region-of-Interest Prediction and Eye Tracking, Markus Hofbauer, Christopher B. Kuhn, Lukas Püttner, Goran Petrovic, Eckehard Steinbach; ISM 2020* [[PDF](https://www.researchgate.net/publication/345241621_Measuring_Driver_Situation_Awareness_Using_Region-of-Interest_Prediction_and_Eye_Tracking)]

```tex
@inproceedings{hofbauer_2020,
    title = {Measuring Driver Situation Awareness Using Region-of-Interest Prediction and Eye Tracking},
    booktitle = {22nd IEEE International Symposium on Mulitmedia},
    publisher = {IEEE},
    address = {Naples, Italy},
    author = {Hofbauer, Markus and Kuhn, Christopher B. and Püttner, Lukas and Petrovic, Goran and Steinbach, Eckehard},
    month = {Dec},
    year = {2020},
    pages = {1--5},
}
```

## Setup

This is purely implemented in Python 3 and so far has been tested on

| OS  | ROS Version |
| --- | ----------- |
| Ubuntu 20.04 | Noetic |

1. Setup [TELECARLA](https://github.com/hofbi/telecarla/blob/master/README.md#setup)
1. Clone this repository into the workspace's `src` folder with `git clone https://github.com/hofbi/driver-awareness.git`
1. Run the install script: `./install.sh` **NOTE** You have to restart the computer after running this script.
1. Install the python requirements: `pip3 install -r requirements.txt`
1. Build the workspace: `catkin build`
1. Source your workspace `source ~/catkin_ws_teleop/devel/setup.<your_shell>`

## Run

* See the main module for running the entire application: [awareness_detector](awareness_detector).
* See the [gaze_detector](gaze_detector) for details on using the eye tracking device.

## Development

To install the additional tools required for the development, call

```shell
python3 -m pip install -r requirements-dev.txt
sudo snap install shfmt
```

from the source of this directory. Then, you can call

```shell
# Format the code
make format

# Check format
make check_format

# Pylint
make pylint
```
