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

#include "jir_rectification_remap_lib/rectification_remapper.h"

#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>



using namespace std;
using namespace cv;

namespace jir_rectification_remap_lib {  
	
	bool RectificationRemapper::loadMap(const std::string& filename){
		cv::FileStorage fs(filename, cv::FileStorage::READ);

		if( !fs.isOpened() ) {
			return false;
		}

		fs["mapx"] >> mapx;
		fs["mapy"] >> mapy;
		fs["mask"] >> mask;
		Mat K;
		fs["camera_info"] >> K;
		intrinsics=K;

		cv::Size s = mapx.size();
		H = s.height;
		W = s.width;
	
		info.height=H;
		info.width=W;
		
		info.K[0]=K.at<float>(0,0);
		info.K[1]=0;
		info.K[2]=K.at<float>(0,2);
		
		info.K[3]=0;
		info.K[4]=K.at<float>(1,1);
		info.K[5]=K.at<float>(1,2);
		
		info.K[6]=0;
		info.K[7]=0;
		info.K[8]=1;
		
		info.R[0] = info.R[4] = info.R[8] = 1.0;

		info.P[0] = K.at<float>(0,0);
		info.P[2] = K.at<float>(0,2);
		info.P[5] = K.at<float>(1,1);
		info.P[6] = K.at<float>(1,2);
		info.P[10] = 1.0;

		info.binning_x = 0;
		info.binning_y = 0;
		info.roi.height = H;
		info.roi.width = W;
		info.roi.do_rectify = false;

		fs["input_rows"] >> input_rows;
		fs["input_cols"] >> input_cols;

		return true;
	}
	
	Mat RectificationRemapper::getRemappedCameraIntrinsics() const {
		return intrinsics;
	}
	
	Mat RectificationRemapper::getMask() const {
		return mask;
	}
	
	sensor_msgs::CameraInfo RectificationRemapper::getCameraInfo(){
		return info;
	}
	
	bool RectificationRemapper::rectifyImage (const cv::Mat& input, cv::Mat& output){
		if( input.rows != input_rows || input.cols != input_cols ) {
			return false;
		}
		
		remap(input, output, mapx, mapy, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar::all(0));
		
		return true;
	}

}
