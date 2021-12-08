
#include<string>
#include<iostream>


#include <Eigen/Geometry>
#include <Eigen/Core>

#include <opencv2/core.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/affine.hpp>


#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image.h>



#include<pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h> // filter point cloud


// segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_line.h> // line segmentation

// Circle Detection in Point cloud
#include <pcl/filters/extract_indices.h>


#include "data_type.h"


cv::Mat getRangeImage(std::vector<drillTarget> drillTargets,const params& params,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud); // returns range image in pcl format as well as in cv::mat format

cv::Affine3f calculateProfilometerPose(std::vector<drillTarget>& drillTargets,const params& params);

cv::Mat  createRangeImageMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered,
                            const cv::Affine3f& profilometerPose,
                             const params& params );

void writeRangeImageFile(const pcl::RangeImage& rangeImage,const std::string& cloudFolder, const cv::Affine3f& profilometerPose); // write range image info


void filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
													pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered,
													const std::string& axis,
													const float (&limits)[2]);

std::vector<drillTarget> readPosesFromFile (const std::string& pathTOAlignedHolesLoc); // dont implement this function, it is for testing 


// These functions are already implemented
cv::Matx33f rotation_from_euler(float roll, float pitch, float yaw); // 
Quaternion ToQuaternion(const float& yaw, const float& pitch, const float& roll); // yaw (Z), pitch (Y), roll (X)
EulerAngles ToEulerAngles(const Quaternion& q);
pcl::PointCloud<pcl::PointXYZ>::Ptr readPLYFile(const std::string& cloudPath); // read ply




int main(int argc, char **argv) {
        
        params params;
        // Read point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = readPLYFile(params.cloudPath);
        std::string pathToHoles= params.auditFolder + "/holes_dense4.txt"; // read from database
        std::vector<drillTarget> drillTargets = readPosesFromFile(pathToHoles ); //That comes from dataBase
    
       
        cv::Mat rangeImage = getRangeImage(drillTargets,params,cloud); 
        


    return 0;
}


 cv::Mat getRangeImage(std::vector<drillTarget> drillTargets,const params& params,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
        
      

        // filter PC
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        float filterCloudRegion[2]{params.filterParam,std::numeric_limits<float>::max()}; 
        std::string axis = "z";
        filterPointCloud(cloud,cloud_filtered,axis,filterCloudRegion);


        // calculate sensor pose
        cv::Affine3f profilometerPose =  calculateProfilometerPose(drillTargets, params);
        
        // Range image
        cv::Mat rangeImage = createRangeImageMap(cloud_filtered,profilometerPose,params);
        
        // write range image info and image in cv format
        std::string imagePath = params.auditFolder + "/rangeImage.png";
        cv::imwrite(imagePath,rangeImage);
        //writeRangeImageFile(rangeImageMap.first, params.auditFolder,profilometerPose);
        return rangeImage;
            
}

cv::Affine3f calculateProfilometerPose(std::vector<drillTarget>& drillTargets,const params& params){

        for (size_t i = 0; i < drillTargets.size(); i++){
            EulerAngles ea = ToEulerAngles(drillTargets.at(i).quartenion);
            drillTargets.at(i).rotation_matrix = rotation_from_euler(ea.roll,ea.pitch,ea.yaw);
            cv::Affine3f homogeneousmatrix{drillTargets.at(i).rotation_matrix, drillTargets.at(i).position};
            drillTargets.at(i).homogeneousMat = homogeneousmatrix;
        }
        
        // TCP Pose
        drillTarget middlePoint = drillTargets.at((std::floor(drillTargets.size()/2)));
        cv::Vec3f offset{0,0,params.sensorDistanceOffset};
        cv::Vec3f translationInSensorFrame = middlePoint.rotation_matrix * offset;
        cv::Affine3f robotPose = middlePoint.homogeneousMat.translate(translationInSensorFrame);
        


        // Profilometer calibration
        cv::Affine3f profilometerCalib;
        profilometerCalib.rotation(rotation_from_euler(params.profilometerCalibRotation[0],params.profilometerCalibRotation[1],params.profilometerCalibRotation[2]));
        cv::Vec3f profilometerTranslation{params.profilometerCalibTranslation};
       
        
        cv::Affine3f sensorPose = robotPose * profilometerCalib ; 
        cv::Affine3f sensorPoseFinal = sensorPose.translate(profilometerTranslation);
        return sensorPoseFinal;
    }

/*
cv::Affine3f calculateProfilometerPose(std::vector<drillTarget>& drillTargets,const params& params){

        for (size_t i = 0; i < drillTargets.size(); i++){
            EulerAngles ea = ToEulerAngles(drillTargets.at(i).quartenion);
            drillTargets.at(i).rotation_matrix = rotation_from_euler(ea.roll,ea.pitch,ea.yaw);
            cv::Affine3f homogeneousmatrix{drillTargets.at(i).rotation_matrix, drillTargets.at(i).position};
            drillTargets.at(i).homogeneousMat = homogeneousmatrix;
        }
        
        // TCP Pose
        drillTarget middlePoint = drillTargets.at((std::floor(drillTargets.size()/2)));
        cv::Vec3f translate2Sensor{0,0,params.sensorDistanceOffset};
        cv::Affine3f robotPose = middlePoint.homogeneousMat.translate(translate2Sensor);
        

       


        // Profilometer calibration
        cv::Affine3f profilometerCalib;
        profilometerCalib.rotation(rotation_from_euler(params.profilometerCalibRotation[0],params.profilometerCalibRotation[1],params.profilometerCalibRotation[2]));
        cv::Vec3f profilometerTranslation{params.profilometerCalibTranslation};
       
        
        cv::Affine3f sensorPose = robotPose * profilometerCalib ; 
        cv::Affine3f sensorPoseFinal = sensorPose.translate(profilometerTranslation);
        return sensorPoseFinal;
    }
*/

cv::Mat  createRangeImageMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered,
                                                           const cv::Affine3f& profilometerPose,
                                                         const params& params ){
        
        float angularResolution_x = (float) (params.angular_resolution * (M_PI/180.0f));  
        float angular_resolution_y = (float) (params.angular_resolution * (M_PI/180.0f));
        float maxAngleWidth     = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
        float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians

        cv::Affine3f sensorPose = profilometerPose;
        pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
        float noiseLevel=0.00;
        float minRange = 0.0f;
        int borderSize = 100;


        //pcl::RangeImage::Ptr range_image_ptr(new pcl::RangeImage);
        //pcl::RangeImage rangeImage = *range_image_ptr;
   	    pcl::RangeImage rangeImage;
           
           // Creating Range image
        rangeImage.createFromPointCloud(*cloud_filtered, angularResolution_x,angular_resolution_y, maxAngleWidth, maxAngleHeight,
                                  sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

        
        rangeImage.setUnseenToMaxRange();

      
        // normalize
        float min, max;
        rangeImage.getMinMaxRanges(min,max);

        writeRangeImageFile(rangeImage, params.auditFolder,profilometerPose);

        cv::Mat range_map(rangeImage.height, rangeImage.width, CV_8UC3);

        // create range map or binary image
        for (int h = 0; h < rangeImage.height; h++){
            for (int w = 0; w < rangeImage.width; w++){   
                if(isinf(rangeImage.at(w,h).range) || rangeImage.at(w,h).range > params.scanDisToWorkpiece ){
                range_map.at<cv::Vec3b>(h,w)[0] = 0;
                range_map.at<cv::Vec3b>(h,w)[1] = 0;
                range_map.at<cv::Vec3b>(h,w)[2] = 0;

            }else{
                range_map.at<cv::Vec3b>(h,w)[0]= ((rangeImage.at(w,h).range) / max)*255; // Normalize for color image
                range_map.at<cv::Vec3b>(h,w)[1]= ((rangeImage.at(w,h).range) / max)*255;
                range_map.at<cv::Vec3b>(h,w)[2]= ((rangeImage.at(w,h).range) / max)*255;
          }
        }  
      }
      return range_map;
}



void writeRangeImageFile(const pcl::RangeImage& rangeImage,const std::string& auditFolder,const cv::Affine3f& profilometerPose ){

        //std::string cloudFolder = "/home/oguz/vs_code/lma_cam2D/inputs/new_dataset_mockup/3";
        std::string rangeFile = auditFolder + "/rangeImage.txt";
        std::ofstream file (rangeFile);
        file << rangeImage << std::endl;
        file  <<"image_offset_x: " <<rangeImage.getImageOffsetX () << std::endl;
        file  <<"image_offset_y: " <<rangeImage.getImageOffsetY () << std::endl;
        float min,max;
        rangeImage.getMinMaxRanges (min,max);
        file  <<"max_range: " << max << std::endl;
        file  <<"min_range: " << min << std::endl;
         file  <<"Profilometer Pose"  << std::endl;
        file << "position_x: " <<  profilometerPose.translation()[0] << std::endl;
        file << "position_y: " <<  profilometerPose.translation()[1] << std::endl;
        file << "position_z: " <<  profilometerPose.translation()[2] << std::endl;
        file << "axis_x: " <<  profilometerPose.rvec()[0] << std::endl;
        file << "axis_y: " <<  profilometerPose.rvec()[1] << std::endl;
        file << "axis_z: " <<  profilometerPose.rvec()[2] << std::endl;
        file.close();
}




pcl::PointCloud<pcl::PointXYZ>::Ptr readPLYFile(const std::string& cloudPath){
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PLYReader Reader;
	Reader.read(cloudPath, *cloud);
	if ( cloud->width <1){ 
    	//PCL_ERROR ("Couldn't read file point_cloud.ply \n");
    	throw std::runtime_error("Couldn't read cloud ply file \n");
	}
	return cloud;
}

void filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
													pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered,
													const std::string& axis,
													const float (&limits)[2]){
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pass;
 	pass.setInputCloud (cloud);
  	pass.setFilterFieldName (axis);
  	pass.setFilterLimits (limits[0],limits[1]);
  	//pass.setFilterLimitsNegative (true);
  	pass.filter (*cloud_filtered);
	
}

Quaternion ToQuaternion(const float& yaw, const float& pitch, const float& roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

EulerAngles ToEulerAngles(const Quaternion& q) {   
    EulerAngles angles;
    // ZYX Convention
    // roll (x-axis rotation)
    float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}


cv::Matx33f rotation_from_euler(float roll, float pitch, float yaw){
    // roll and pitch and yaw in radian
    float su = sin(roll);
    float cu = cos(roll);
    float sv = sin(pitch);
    float cv = cos(pitch);
    float sw = sin(yaw);
    float cw = cos(yaw);
    cv::Matx33f Rot_matrix(3, 3);
    Rot_matrix(0, 0) = cv*cw;
    Rot_matrix(0, 1) = su*sv*cw - cu*sw;
    Rot_matrix(0, 2) = su*sw + cu*sv*cw;
    Rot_matrix(1, 0) = cv*sw;
    Rot_matrix(1, 1) = cu*cw + su*sv*sw;
    Rot_matrix(1, 2) = cu*sv*sw - su*cw;
    Rot_matrix(2, 0) = -sv;
    Rot_matrix(2, 1) = su*cv;
    Rot_matrix(2, 2) = cu*cv;
    return Rot_matrix;
}


std::vector<drillTarget> readPosesFromFile (const std::string& pathTOAlignedHolesLoc){
    // read holes location from file
    std::vector<drillTarget> res;
	std::vector<cv::Vec3f> positions; 
	std::vector<Quaternion> qs;
    std::ifstream file(pathTOAlignedHolesLoc);
    if (file.is_open()) { 
        std::string line;
        while (getline(file,line)){
             if(line[0] == 'i' || line.empty())
                 continue;
            else{
                drillTarget d;
                d.piloted = true; // read from file
                float id,x,y,z;
                Quaternion q;
                std::stringstream sin(line);
                sin >>id >>  d.position[0] >> d.position[1] >> d.position[2] >> d.quartenion.w >> d.quartenion.x >> d.quartenion.y >> d.quartenion.z;
                res.emplace_back(d);
            }
        }
    }
    
    
    return res;
}