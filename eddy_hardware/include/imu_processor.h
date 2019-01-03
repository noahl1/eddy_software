#ifndef IMUProcessor_H
#define IMUProcessor_H

#include "imu_3dm_gx4/FilterOutput.h"
#include "imu_3dm_gx4/MagFieldCF.h"
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "math.h"
#include "riptide_msgs/Imu.h"
#include "riptide_msgs/ImuVerbose.h"
#include "tf/LinearMath/Vector3.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Vector3.h"

using namespace std;

class IMUProcessor
{

    private:
        ros::NodeHandle nh;
        ros::Subscriber sub1;
        ros::Subscriber sub2;
        ros::Publisher imuVerboseStatePub;
        ros::Publisher imuStatePub;

        //Infinite Input Response Low Pass Filter (IIRLPF) Variables
        int filterRate;
        double postIIRLPFBandwidth, timeDelay, alpha;
        geometry_msgs::Vector3 prevAngVel, prevLinAccel;

        //Output Messages
        riptide_msgs::ImuVerbose verbose_state;
        riptide_msgs::Imu imu_state;

        //MagCallback Variables
        float mag_X_B, mag_Y_B, mag_Z_B, mag_X_W, mag_Y_W, mag_Z_W, heading;
        double declination;
        tf::Matrix3x3 BtoM;

        //FilterCallback Variables
        tf::Vector3 tf;

    public:
        IMUProcessor();
        template <typename T>
        void LoadParam(string param, T &var);
        void MagCallback(const imu_3dm_gx4::MagFieldCF::ConstPtr& mag_msg);
        void FilterCallback(const imu_3dm_gx4::FilterOutput::ConstPtr &filter_msg);
        void ConvertRadiansToDegrees();
        void AdjustEulerAngles();
        void PopulateIMUState();
        void Loop();
};

#endif