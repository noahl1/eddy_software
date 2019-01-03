#include "eddy_hardware/imu_processor.h"

#define PI 3.141592653

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_processor");
    IMUProcessor imu;
    imu.Loop();
}

IMUProcessor::IMUProcessor() : nh("imu_processor") 
{
    imuVerboseStatePub = nh.advertise<riptide_msgs::ImuVerbose>("/state/imu_verbose", 1);
    imuStatePub = nh.advertise<riptide_msgs::Imu>("/state/imu", 1);
    sub1 = nh.subscribe<imu_3dm_gx4::FilterOutput>("/imu/filter", 1, &IMUProcessor::FilterCallback, this);
    sub2 = nh.subscribe<imu_3dm_gx4::MagFieldCF>("/imu/magnetic_field", 1, &IMUProcessor::MagCallback, this);

    IMUProcessor::LoadParam<double>("declination", declination);
    IMUProcessor::LoadParam<int>("filterRate", filterRate);
    IMUProcessor::LoadParam<double>("postIIRLPFBandwidth", postIIRLPFBandwidth);

    //Infinite Input Response Low Pass Filter
    timeDelay = 1.0 / filterRate;
    alpha = (2 * PI * timeDelay * postIIRLPFBandwidth) / (2 * PI * timeDelay * postIIRLPFBandwidth + 1);

    prevAngVel.x = 0;
    prevAngVel.y = 0;
    prevAngVel.z = 0;
    prevLinAccel.x = 0;
    prevLinAccel.y = 0;
    prevLinAccel.z = 0;
}

template <typename T>
void IMUProcessor::LoadParam(string param, T &var)
{
    try
    {
        if(!nh.getParam(param, var))
        {
            throw 0;
        }
    }
    catch(int e)
    {
        string ns = nh.getNamespace();
        ROS_ERROR("test_pub_sub Namespace: %s", ns.c_str());
        ROS_ERROR("Critical! Param \"%s/%s\" does not exist or is not accessed correctly. Shutting down.", ns.c_str(), param.c_str());
        ros::shutdown();
    }
}

/*
    Reads the magnetometer data and computes the heading
*/
void MagCallback(const imu_3dm_gx4::MagFieldCF::ConstPtr& mag_msg);
{
    //magnetometer body-frame components
    magX_B = mag_msg->mag_field_components.x;
    magY_B = mag_msg->mag_field_components.y;
    magZ_B = mag_msg->mag_field_components.z;

    //normalize the body-frame components
    IMUProcessor::Norm(magX_B, magY_B, magZ_B);

    //convert body-frame to world-frame multiplying the rotation matrix
    mag_X_W = BtoM.getRow(0).x()*mag_X_B + BtoM.getRow(0).y()*mag_Y_B + BtoM.getRow(0).z()*mag_Z_B;
    mag_Y_W = BtoM.getRow(1).x()*mag_X_B + BtoM.getRow(1)).y()*mag_Y_B + BtoM.getRow(1).z()*mag_Z_B;

    heading = atan(mag_X_W, mag_Y_Z) * 180/PI;

    heading += declination;

    //keep the heading in range [-180, 180] degrees
    if(heading > 180.0) {
        heading -= 360;
    }
    else if(heading < -180.0){
        heading += 360;
    }

    verbose_state.heading = heading;

    //Flip the sign of the z-axis so that up is positive
    verbose_state.euler_rpy.z = -verbose_state.heading;
}
/*
    Normalizes vector components [v1], [v2], and [v3] then rewrites the values to the address
    Parameters
    [v1]    
    [v2]
    [v3]
*/
void IMUProcessor::Norm(float &v1, float &v2, float &v3) {
  float magnitude = sqrt(v1*v1 + v2*v2 + v3*v3);
  v1 = v1/magnitude;
  v2 = v2/magnitude;
  v3 = v3/magnitude;
}

/*
    read the values from the IMU driver and update verbose_state
*/
void IMUProcessor::FilterCallback(const imu_3dm_gx4::FilterOutput::ConstPtr &filter_msg)
{
    verbose_state.header = filter_msg.header;
    verbose_state.header.frame_id = "base_link";

    verbose_state.raw_euler_rpy = filter_msg.euler_rpy;
    verbose_state.euler_rpy.x = filter_msg.euler_rpy.x;
    verbose_state.euler_rpy.y = filter_msg.euler_rpy.y;
    //set the euler_rpy.z in the MagCallback()
    verbose_state.gyro_bias = filter_msg.gyro_bias;
    verbose_state.euler_rpy_status = filter_msg.euler_rpy_status;
    verbose_state.heading_update = filter_msg.heading_update;
    verbose_state.heading_update_uncertainty = filter_msg.heading_update_uncertainty;
    verbose_state.heading_update_source = filter_msg.heading_update_source;
    verbose_state.heading_update_flags = filter_msgs.heading_update_flags;
    verbose_state.raw_linear_accel = filter_msg.heading_update_flags;
    verbose_state.linear_accel = filter_msg.linear_acceleration;
    verbose_state.linear_accel_status = filter_msg.linear_acceleration_status;
    verbose_state.raw_ang_vel = filter_msg.angular_velocity;
    verbose_state.ang_vel = filter_msg.angular_velocity;
    verbose_state.ang_vel_status = filter_msg.ang_vel_status;
    //compute rotation matrix
    tf.setValue(verbose_state.euler_rpy.x, verbose_state.euler_rpy.y, 0);
    BtoM.setRPY(tf.x(), tf.y(), tf.z()); //worldFrame = BtoM * bodyFrame
    //convert angular values from radians to degrees
    IMUProcessor::ConvertRadiansToDegrees();
    //adjust euler angles to be consistent with the vehicle's axes
    IMUProcessor::AdjustEulerAngles();
    
}

/*
    converts angular values from radians to degrees
*/
void IMUProcessor::ConvertRadiansToDegrees(){
    verbose_state.raw_euler_rpy.x *= (180.0/PI);
    verbose_state.raw_euler_rpy.y *= (180.0/PI);
    verbose_state.raw_euler_rpy.z *= (180.0/PI);
    verbose_state.euler_rpy.x *= (180.0/PI);
    verbose_state.euler_rpy.y *= (180.0/PI);
    verbose_state.gyro_bias.x *= (180.0/PI);
    verbose_state.gyro_bias.y *= (180.0/PI);
    verbose_state.gyro_bias.z *= (180.0/PI);
    verbose_state.heading_update *= (180/PI);
    verbose_state.heading_update_uncertainty *= (180/PI);
    verbose_state.raw_ang_vel.x *= (180.0/PI);
    verbose_state.raw_ang_vel.y *= (180.0/PI);
    verbose_state.raw_ang_vel.z *= (180.0/PI);
}

/*
    adjust heading and signs of Euler angles to be conistent with the AUVs axes. NED frame 
*/
void IMUProcessor::AdjustEulerAngles() {
    //for now, assume IMU is mounted correctly
}

/*
    populate the imu_state message
*/
void IMUProcessor::PopulateIMUState() {
    imu_state.header = verbose_state.header;
    imu_state.euler_rpy = verbose_state.euler_rpy;
    imu_state.linear_accel = verbose_state.linear_accel;
    imu_state.ang_vel = verbose_state.ang_vel;
    imu_state.ang_accel = verbose_state.ang_accel;
}

void IMUProcessor::Loop()
{
      while (!ros::isShuttingDown())
      {
        ros::spinOnce();
        rate.sleep();
      }
}