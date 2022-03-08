# Pinax calibration for underwater SLAM
"The Pinax-Model for Accurate and Efficient Refraction Correction of Underwater Cameras in Flat-Pane Housings" code release

In this version, the PinAx camera model is used to model underwater refraction for underwater SLAM
This version has been tested with Ubuntu 20.04 running with UTM VM for mac M1.

# Build in ROS
make catkin folder in home directory:
```
mkdir catkin_ws
cd catkin_ws
mkdir build && Src
catkin_make
```

copy the repo to the catkin folder:
```
cd catkin_ws/src
git clone https://github.com/fickrie67/Pinax-camera-model.git
```

then build the repo:
```
catkin_make
```

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


