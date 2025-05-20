# Main Software

## Support

Tested on `Ubuntu 20.04 Focal Fosa` with `Python 3.8.10`

## How to use

1.  Install packages
    Follow [ROS installation guide](http://wiki.ros.org/Installation/Ubuntu)

    ```sh
    sudo apt install ros-noetic-ros-base ros-noetic-rtabmap-ros ros-noetic-stereo-image-proc
    ```

2.  Prepare Environment

    ```sh
    python -m venv .venv

    source /opt/ros/noetic/setup.bash
    source .venv/bin/activate

    pip install PyOpenGL PyOpenGL-accelerate pyopengltk opencv-contrib-python pillow reactivex pygame PyYAML platformdirs pyrr rospkg req requests websockets
    ```

3.  Run processes

    ```sh
    roscore& roslaunch ros.launch
    ```
