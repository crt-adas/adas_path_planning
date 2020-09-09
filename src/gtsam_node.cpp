#include <ros/ros.h>
#include "ros_utils.hpp"
#include "kinematics.hpp"
#include "estimators.hpp"
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>

#include <gtsam/geometry/Pose2.h>

#include <gtsam_node/ArticulatedAngles.h>
#include <gtsam_node/CanData.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "ackermann_msgs/AckermannDrive.h"


int main(int argc, char **argv)
{
    using namespace gtsam_node;
    using namespace ast;
    using namespace ast::ros;
    ::ros::init(argc, argv, "gtsam_node");
    NodeHandle nh;
    
    /*----------------| Get Params |----------------*/
    CarParams carParams;
    MheParams mheParams;
    ParamsIn(carParams, mheParams, nh);
    /*---------------------------------------------*/
    /*------------| Global Var and Obj |-----------*/
    Vec5 qLocTrailer;        
    Vec4 qTrailerLoc;
    Vec4 qOneTrailerEst;
    /*---------------------------------------------*/
    /*----------------| ROS Subscribers and Publishers  |-----------------------------------------*/

    auto canDataIn = nh.Input<CanData>("/processed_can_data");
    boost::shared_ptr<CanData> canData(new CanData());

    auto perceptionPoseCamIn = nh.Input<geometry_msgs::PoseStamped>("/pose_estimator/charger_pose/location_cam");
    auto perceptionPoseGpsIn = nh.Input<geometry_msgs::PoseStamped>("/pose_estimator/charger_pose/location_gps");
    boost::shared_ptr<geometry_msgs::PoseStamped> perceptionPose(new geometry_msgs::PoseStamped());

    auto perceptionTwistWeightedOut = nh.Output<geometry_msgs::Twist>("mhe_node/weighted_estimated/twist");
    geometry_msgs::Twist perceptionTwistWeightedData;

    auto articulatedAnglesWeightedOut = nh.Output<ArticulatedAngles>("mhe_node/weighted_estimated/articulated_angles");
    ArticulatedAngles articulatedAnglesWeightedData;

    auto poseWithCovarianceIn = nh.Input<geometry_msgs::PoseWithCovarianceStamped>("mhe_node/perception_data/pose_with_covariance");
    //geometry_msgs::PoseWithCovarianceStamped poseWithCovarianceData;
    
    auto poseWithCovarianceWeightedOut = nh.Output<geometry_msgs::PoseWithCovarianceStamped>("mhe_node/weighted_estimated/pose_with_covariance");
    geometry_msgs::PoseWithCovarianceStamped poseWithCovarianceWeightedData;

    auto ackermannDriveOut = nh.Output<ackermann_msgs::AckermannDrive>("mhe_node/can_data/ackermann_drive");
    ackermann_msgs::AckermannDrive ackermannDriveData;
    /*------------------------------------------------------------------------------------------*/

    ROS_INFO_STREAM("Estimator loop rate: "<< mheParams.loopRate <<"");
    Real t = 0;
    bool firstPoseMeasured = false;
    ::ros::Rate loop_rate(mheParams.loopRate);
    while(::ros::ok())
    {
        
        auto now = ::ros::Time::now();
     
        *canData = canDataIn(); 

        ackermannDriveData.speed = canData->tachoVelocity;
        ackermannDriveData.steering_angle = canData->steeringAngle;
       
        
        if(mheParams.perceptionGPS)
        {
            *perceptionPose = perceptionPoseGpsIn(); 
        }else
        {
            *perceptionPose = perceptionPoseCamIn();  
        }
    

        if(perceptionPose->pose.position.x != 0 && perceptionPose->pose.position.y != 0)
        {
            
            // trailer number 1      

            auto perceptionTh = tf::getYaw(perceptionPose->pose.orientation);
            qLocTrailer[0] = canData->beta1;
            qLocTrailer[1] = continuousAngle(perceptionTh, qOneTrailerEst[1]);
            qLocTrailer[2] = perceptionPose->pose.position.x - carParams.L*cos(perceptionTh);
            qLocTrailer[3] = perceptionPose->pose.position.y - carParams.L*sin(perceptionTh);
            qLocTrailer[4] = canData->steeringAngle;
            qTrailerLoc = {qLocTrailer[0],qLocTrailer[1],qLocTrailer[2],qLocTrailer[3] };

            auto simFunc = [&](const Vec4& qTrailer, Vec4& dq, const double t)
            {
                if(!carParams.moveGuidancePoint)
                    dq = OneTrailerKinematicsGPRear(carParams, qTrailer, ackermannDriveData);
                else
                    dq = OneTrailerKinematicsGPFront(carParams, qTrailer, ackermannDriveData);
            };
                
            Vec4 qOneTrailerPred = qOneTrailerEst;
            boost::numeric::odeint::integrate(simFunc, qOneTrailerPred, 0.0, 1.0/(Real)mheParams.loopRate, 1.0/(Real)mheParams.loopRate);
            
            ///estimation here, result to = qOneTrailerEst



















            poseWithCovarianceWeightedData.pose.pose.orientation = tf::createQuaternionMsgFromYaw(qOneTrailerEst[1]);
            poseWithCovarianceWeightedData.pose.pose.position.x = qOneTrailerEst[2]  + carParams.L*cos(qOneTrailerEst[1]);
            poseWithCovarianceWeightedData.pose.pose.position.y = qOneTrailerEst[3]  + carParams.L*sin(qOneTrailerEst[1]);
            poseWithCovarianceWeightedData.header.stamp = ::ros::Time::now();
            poseWithCovarianceWeightedOut(poseWithCovarianceWeightedData);

            articulatedAnglesWeightedData.trailer1 = qOneTrailerEst[0];
            articulatedAnglesWeightedOut(articulatedAnglesWeightedData);

            perceptionTwistWeightedData.angular.z = qOneTrailerEst[1];
            perceptionTwistWeightedData.linear.x = qOneTrailerEst[2] + carParams.L*cos(qOneTrailerEst[1]);
            perceptionTwistWeightedData.linear.y = qOneTrailerEst[3] + carParams.L*sin(qOneTrailerEst[1]);
            perceptionTwistWeightedOut(perceptionTwistWeightedData);

        }
        auto loopTime =  ::ros::Time::now() - now;
        ROS_INFO_STREAM("loop time" << loopTime.toSec());

        t += 1/((Real) mheParams.loopRate);
        ::ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}
