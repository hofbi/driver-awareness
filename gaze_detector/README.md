# Gaze Detector

The gaze detector package is responsible for the communication with the eye tracking device. Details on the devices can be found [here](https://gitlab.lrz.de/teleop/env-setup/wikis/eye-tracking).

## Run

### Gaze Publisher

Acts as an interface to the eye tracking device and publishes the measurements as ROS message. There is currently a concrete implementation of the interface for [Pupil Core](https://pupil-labs.com).

* Connects to the Pupil Capture software over the network interface using ZMQ.
* Gaze and fixation measurements are received in normalized coordinates [0,1] on the defined surfaces.
* The surface is here the screen of the operator. The surface of the monitor can be tracked by the Pupil Capture application using markers placed around the screen. Refer to [Pupil Docs Surface Tracking](https://docs.pupil-labs.com/core/software/pupil-capture/#surface-tracking).
* The normalized coordinates are converted into pixel coordinates.

```shell
roslaunch gaze_detector gaze_publisher.launch pupil_ip:=<IP_PUPIL_CAPTURE>
```

### Gaze Visualizer

* Validate the accuracy of the current calibration (recalibrate in case of a bad calibration).
* The Gaze Publisher node has to be running.
* A calibration image with fixed points and a config file with the position of the circles in the image is shown to the user.
* Different markers are shown to the user which need to be fixated as long as the red dot is displayed on them.
* The node calculates the mean dist of the recorded data points to each circle in the given calibration image.
* The overall mean dist for a good calibration should be around 30 pixels.
* The points are plotted on the calibration image and the results are written into the `output` folder.

```shell
roslaunch gaze_detector gaze_visualizer.launch
```

### Remote Calibration

* Provides remote calibration of the Pupil Core if it is connected to a different workstation in the same network. This is required in case of running heavy applications such as the CARLA simulator to ensure a high gaze frequency.
* In the Pupil Capture software running on the second workstation *manual marker calibration* has to be selected.
* Calibration markers are displayed on the screen which is used for the actual task (e.g. the workstation running the simulator).
* Use the `remote_calibration_config.yaml` to configure the resolution of the used display, default is 1920x1080. Here also the size of the markers and their offset from the screen edges can be configured.

```shell
roslaunch gaze_detector remote_calibration.launch pupil_ip:=<IP_PUPIL_CAPTURE>
```

## Pupil Capture Software

* Start the software from a terminal with `pupil_capture`.
* On *General Settings* set detection & mapping mode to *3d*. Make sure both *detect eye 0* and *detect eye 1* is selected.
* Adjust the position of the eye-cameras in a way that both eyes are good visible in the camera image and are not too small or too big. Adjust the focus of the world camera so the used display is sharp.
* From the *Plugin Manager* activate the *Camera Intrinsics Estimation*, *Fixation Detector* if needed, *Pupil Remote* and *Surface Tracker*.
* Under *UVC Source* select a resolution of 1280x720 and a frame rate of 30 fps for the world camera.
* Under *Screen Marker Calibration* the *Manual Marker Calibration* should be selected to do remote calibration if the Pupil Capture software is not running on the computer that is connected to the screen and is used for measurements. Otherwise *Screen Marker Calibration* can be used.
* The provided calibration of the world camera can be used. With *show undistorted image* the calibration can be tested. If straight lines in real world are not straight in the image, the calibration should be runed using the camera intrinsics estimation plugin.
* Under Surface Tracker press *Add Surface* if the markers used for screen detection are in the fov of the camera. *Freeze Screen* can be used to adjust the defined surface to the display directly in the world camera view.
* If gaze measurements are done on the same display of the workstation the Pupil Capture is connected to use the `C` button to do screen marker calibration. If a separate workstation is used, run the [remote calibration](remote-calibration).

Find more details in the [docs](https://docs.pupil-labs.com/core/software/pupil-capture/)
