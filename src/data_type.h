#pragma once



struct params{
    float edgeToleranceMin = 9.652f;
    float edgeToleranceMax = 14.478f;
    float centerToleranceMin = 16.764f;
    float centerToleranceMax = 28.956f;
    float filterParam= -30.0f; // downer boundary of the filter
    float scanDisToWorkpiece =800.0f; // Dense scan distance to workpiece, used to filter noises
    float angular_resolution = 0.01f;
    float angular_resolution_border = 1.0f; 
    float sensorDistanceOffset = 600.0f;
    float maxCorrespondenceDistance= 10.0f;
    float shiftInHoleCenter= 5.0f; // shift in hole centers, in case cad model dont represent the actual value 
    cv::Vec3f profilometerCalibTranslation{-30.2098, 0.88452506,6.37940233};
    cv::Vec3f profilometerCalibRotation{(float) (180.23855362f * (M_PI/180.0f)),(float) (-0.32691827f * (M_PI/180.0f)),(float) (179.78128758f * (M_PI/180.0f))};
    std::string auditFolder = "/home/oguz/vs_code/inspectDrillTargets/audits/audit_4"; 
    std::string cloudPath = "/home/oguz/vs_code/inspectDrillTargets/audits/audit_4/PointCloud4.ply"; 
};

struct Quaternion {
    float w, x, y, z;
};

struct EulerAngles {
    float roll, pitch, yaw;
};


struct boundingBox{
    cv::Point topLeft;
    cv::Point topRight;
    cv::Point bottomLeft;
    cv::Point bottomRight;
    
    void createBB(const cv::Vec3f& circle){
        this->topLeft.x= circle[0] - 2*circle[2];
        this->topLeft.y= circle[1] -   2*circle[2];

        this->topRight.x= circle[0] +  2*circle[2];
        this->topRight.y= circle[1] -   2*circle[2];

        this->bottomLeft.x= circle[0] -  2*circle[2];
        this->bottomLeft.y= circle[1] +  2*circle[2];

        this->bottomRight.x= circle[0] +  2*circle[2];
        this->bottomRight.y= circle[1] +  2*circle[2];
    }
    
};


struct drillTarget{
    cv::Vec3f position;
    cv::Vec3f position_matchedHole{0,0,0};
    Quaternion quartenion;
    cv::Matx33f rotation_matrix;
    cv::Affine3f homogeneousMat;
    bool piloted = false;
    bool fullSized =false;
    bool withinEdgeTolerance = true;
    bool withinCenterTolerance = true;
    bool misPrediction = false;
    float distanceToEdge;
    float distanceToCenter;
    float detectedHoleDiameter;
    int ID;
};
