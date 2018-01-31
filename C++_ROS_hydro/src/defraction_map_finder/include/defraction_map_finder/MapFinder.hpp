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

#include <iostream>
#include <jir_refractive_image_geometry/refracted_pinhole_camera_model.hpp>
#include <jir_refractive_image_geometry_msgs/PlanarRefractionInfo.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <string>
#include <vector>
#include <sensor_msgs/CameraInfo.h>
#include <eigen3/Eigen/Eigen>
#include <jir_refractive_image_geometry/ray.hpp>
#include <jir_refractive_image_geometry/ray_operations.hpp>
#include <jir_refractive_image_geometry/refraction_operations.hpp>

#include "defraction_map_finder/CataCamera.h"



using namespace std;
using namespace cv;
using namespace Eigen;
using namespace jir_refractive_image_geometry;
using namespace camodocal;

class MapFinder {
	public:
	MapFinder(void){};
	~MapFinder(void){};
		
	MapFinder(int W, int H, float d_0, float d0off, float d_1, float n_g, float n_w, 
								const sensor_msgs::CameraInfo info,CameraPtr realCam); 

	
	void init_setup (int W, int H, float d_0, float d0off, float d_1, float n_g, float n_w, 
								const sensor_msgs::CameraInfo info,CameraPtr realCam); 
	void generate3Dpoints(); 
	void projectPoints(); 
	void saveMaps (const std::string& filename);

	cv::Mat getMask() const;

	private:
	Point2d projectPoint (Vector3d point3D, bool notRefracted); 
	int ImgWidth;
	int ImgHeight;
	Mat mapx;
	Mat mapy;
	jir_refractive_image_geometry::RefractedPinholeCameraModel VirtualCamera;
	jir_refractive_image_geometry::RefractedPinholeCameraModel RealCamera;
	
	float d0;
	float d0Virt_offset;
	float d1;
	float ng;
	float nw;
	
	vector <Vector3d> points3D;	
	jir_refractive_image_geometry_msgs::PlanarRefractionInfo RefractionInfo;
	sensor_msgs::CameraInfo camera_info;
	CameraPtr myCamera;
};


