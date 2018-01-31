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

#include <opencv2/core/core.hpp>
#include <string>
#include <sensor_msgs/CameraInfo.h>

namespace jir_rectification_remap_lib {

class RectificationRemapper{
public:
	RectificationRemapper(){};
	~RectificationRemapper(){};

	bool loadMap(const std::string& filename);

	cv::Mat getRemappedCameraIntrinsics() const;
	cv::Mat getMask() const;

	sensor_msgs::CameraInfo getCameraInfo();

	bool rectifyImage (const cv::Mat& input, cv::Mat& output);

	inline bool operator()(const cv::Mat& input, cv::Mat& output) {
		return rectifyImage(input,output);
	}


	double getFocalLength() const {
		return info.K[0];
	}
	double getCx() const {
		return info.K[2];
	}
	double getCy() const {
		return info.K[5];
	}

	int expectedRows() const {
		return input_rows;
	}
	int expectedCols() const {
		return input_cols;
	}

	
private:
	cv::Mat mask;
	cv::Mat mapx;
	cv::Mat mapy;
	cv::Mat intrinsics;
	int input_rows, input_cols;
	sensor_msgs::CameraInfo info;
	int H;
	int W;
	double f;
	
};

}
