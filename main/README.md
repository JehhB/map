# Main Software

## How to use

1.  Install packages
    1. Follow [ROS installation guide](http://wiki.ros.org/Installation/Ubuntu) and download `ros-noetic-ros-base`, `ros-noetic-rtabmap-ros`, and `ros-noetic-stereo-image-proc`
    2. Follow [Miniconda installation](https://www.anaconda.com/docs/getting-started/miniconda/install)
2.  Prepare environments

    ```sh
    source ~/miniconda3/bin/activate
    source /opt/ros/noetic/setup.zsh

    conda create --prefix .venv python=3.13
    conda activate ./.venv
    ```

3.  Install dependencies

    ```sh
    pip install PyOpenGL PyOpenGL-accelerate pyopengltk opencv-contrib-python pillow pypubsub rospy cv_bridge sensor_msgs rospkg
    ```

4.  Run processes

    ```sh
    ROS_NAMESPACE='stereo_camera' rosrun stereo_image_proc stereo_image_proc
    roslaunch rtabmap_ros rtabmap.launch stereo:=true rviz:=false rtabmap_viz:=false
    python main.py
    ```
