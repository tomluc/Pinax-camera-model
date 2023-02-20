# Pinax calibration for underwater SLAM
"The Pinax-Model for Accurate and Efficient Refraction Correction of Underwater Cameras in Flat-Pane Housings" code release

In this version, the PinAx camera model is used to model underwater refraction for underwater SLAM using [ORB_SLAM](https://github.com/fickrie67/ORB_SLAM3.git).
This version has been tested with Ubuntu 20.04 running with UTM VM for mac M1.


# Build in ROS

ROS installation

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt install curl # if you haven't already installed curl

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update

sudo apt install ros-noetic-desktop-full

sudo apt install python3-catkin-tools python3-osrf-pycommon

source /opt/ros/noetic/setup.bash
```

make catkin folder in home directory:
```
mkdir catkin_ws
cd catkin_ws
mkdir build && mkdir src
catkin build
```

copy the repo to the catkin folder:
```
cd ~/catkin_ws/src
git clone https://github.com/fickrie67/pinax-camera-model.git
```

then build the repo:
```
catkin build
```

# Running in ROS 
run ros master and source the ros setup
```
source ~/catkin_ws/devel/setup.bash
roscore
```

play recorded video
```
rosbag play testBag.bag --loop
```

then calculate correction map for the video.
this will produce correctionMap.yaml file in src folder
```
cd ~/catkin_ws/src/pinax-camera-model/ROS

rosrun defraction_map_finder defraction_map_finder
```

adjust the video with correction map file
```
roslaunch jir_image_remapper image_remapper.launch
```

View the adjusted image
```
rosrun image_view image_view image:=<topics>

rosrun image_view image_view image:=/rectified/left/image

or

rqt_image_view
```

record video

```
rosrun image_view video_recorder _fps:=25 _filename:="/home/fickrie/test1.MP4" image:=/rectified/image
```

use the corrected video for UW-SLAM purposes. the corrected video can be seen in the /rectified/image topics. 

# Pinax-camera-model
When using this code in scientific work please cite:

Tomasz Łuczyński, Max Pfingsthorn, Andreas Birk
The Pinax-model for accurate and efficient refraction correction of underwater cameras in flat-pane housings
Ocean Engineering, Volume 133, 2017, Pages 9-22, ISSN 0029-8018, http://dx.doi.org/10.1016/j.oceaneng.2017.01.029.
(http://www.sciencedirect.com/science/article/pii/S0029801817300434)

Abstract: The calibration and refraction correction process for underwater cameras with flat-pane interfaces is presented that is very easy and convenient to use in real world applications while yielding very accurate results. The correction is derived from an analysis of the axial camera model for underwater cameras, which is among others computationally hard to tackle. It is shown how realistic constraints on the distance of the camera to the window can be exploited, which leads to an approach dubbed Pinax Model as it combines aspects of a virtual pinhole model with the projection function from the axial camera model. It allows the pre-computation of a lookup-table for very fast refraction correction of the flat-pane with high accuracy. The model takes the refraction indices of water into account, especially with respect to salinity, and it is therefore sufficient to calibrate the underwater camera only once in air. It is demonstrated by real world experiments with several underwater cameras in different salt and sweet water conditions that the proposed process outperforms standard methods. Among others, it is shown how the presented method leads to accurate results with single in-air calibration and even with just estimated salinity values.

******************** LICENSE & DISCALIMER*************************
Examples are released as Matlab and C/C++ ROS code. 
Unless specified otherwise in the respective files this 
code was developed within Jacobs Robotics Group, 
Jacobs University Bremen gGmbH. 

THIS SOFTWARE IS PROVIDED BY Jacobs Robotics ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO
, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL Jacobs Robotics BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Unless specified otherwise this code examples are released under
 Creative Commons CC BY-NC-ND 4.0 license (free for non-commercial use). 
 Details may be found here: https://creativecommons.org/licenses/by-nc-nd/4.0/

If you wish to commercialize the Pinax model please contact robotics@jacobs-university.de


