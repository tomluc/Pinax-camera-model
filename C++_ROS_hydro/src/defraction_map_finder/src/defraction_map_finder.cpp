/*
* Copyright (c) 2017 Jacobs University Robotics Group
* All rights reserved.
*
*
* Unless specified otherwise this code examples are released under 
* Creative Commons CC BY-NC-ND 4.0 license (free for non-commercial use). 
* Details may be found here: https://creativecommons.org/licenses/by-nc-nd/4.0/
*
*
* If you are interested in using this code commercially, 
* please contact us.
*
* THIS SOFTWARE IS PROVIDED BY Jacobs Robotics ``AS IS'' AND ANY
* EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL Jacobs Robotics BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Contact: robotics@jacobs-university.de
*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <fstream>
#include <string>
#include <jir_refractive_image_geometry/refracted_pinhole_camera_model.hpp>
#include "defraction_map_finder/MapFinder.hpp"
#include "defraction_map_finder/CameraFactory.h"
#include <Eigen/Geometry>

using namespace cv;
using namespace std;
using namespace Eigen;

using namespace camodocal;


int main(int argc, char** argv)
{
	//input calibration file
	string intr="camera_left.yaml";
    
    //output correction file
    string mapsResults;
	mapsResults="correctionMap.yaml";
	
	//**** virtual camra params ****
	int W=1024;
	int H=768;
	float f=600;
	
	//setup parameters, d0 and d0off optimized in simulation, depending on d1, ng and nw
	float d_0=0.0014;
	float d0off=0.00082;
	float d_1=0.01;
	float n_g=1.492;
	float n_w=1.342;
	//*
	
	
	//**** create camera info ****
	sensor_msgs::CameraInfo info;
	
	info.height=H;
	info.width=W;
	
	info.K[0]=f;
	info.K[1]=0;
	info.K[2]=W/2.0;
	
	info.K[3]=0;
	info.K[4]=f;
	info.K[5]=H/2.0;
	
	info.K[6]=0;
	info.K[7]=0;
	info.K[8]=1;
	
	info.R[0] = info.R[4] = info.R[8] = 1.0;

	info.P[0] = f;
	info.P[2] = W/2.0;
	info.P[5] = f;
	info.P[6] = H/2.0;
	info.P[10] = 1.0;

	info.binning_x = 0;
	info.binning_y = 0;
	info.roi.height = H;
	info.roi.width = W;
	info.roi.do_rectify = false;	
	//*
	
	camodocal::CameraPtr distCam = CameraFactory::instance()->generateCameraFromYamlFile(intr);

	MapFinder find(W, H, d_0, d0off, d_1, n_g, n_w, info, distCam);
	cout<<"object created\n";
	find.generate3Dpoints();
	cout<<"3D points generated\n";

	find.projectPoints();

	cout<<"Points projected\n";
	
	find.saveMaps(mapsResults);
	cout<<"Results saved\n";
	
  return 0;
}
