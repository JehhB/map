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

    pip install PyOpenGL PyOpenGL-accelerate pyopengltk opencv-contrib-python pillow pypubsub rospy cv_bridge sensor_msgs rospkg
    ```

3.  Run processes

    ```sh
    roscore
    ROS_NAMESPACE='stereo_camera' rosrun stereo_image_proc stereo_image_proc
    roslaunch rtabmap_ros rtabmap.launch stereo:=true rviz:=false rtabmap_viz:=false

    python main.py
    ```
