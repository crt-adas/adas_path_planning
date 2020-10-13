#include <ros/ros.h>
#include "ros_utils.hpp"
#include "kinematics.hpp"
#include "estimators.hpp"
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>
#include <Eigen/Dense>
#include <unistd.h>

#include <gtsam/config.h>
#include <gtsam/base/VectorSpace.h>
#include <gtsam/base/Vector.h>
#include <gtsam/dllexport.h>
#include <boost/serialization/nvp.hpp>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/base/OptionalJacobian.h>

#include <gtsam_node/ArticulatedAngles.h>
#include <gtsam_node/CanData.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "ackermann_msgs/AckermannDrive.h"


#include <visualization_msgs/Marker.h>
#include <cmath>

using namespace std;
using namespace gtsam;






class PolyPointFactor: public NoiseModelFactor2<Vector8,Vector5> {
  
  private:
    using This = PolyPointFactor;
    using Base = gtsam::NoiseModelFactor2<Vector8, Vector5>;

    Vector8 mpoly_;
    Vector5 mpoint_;
    
  public:
        /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<PolyPointFactor> shared_ptr;

        // The constructor requires the variable key, the nodes, and the noise model
    PolyPointFactor(Key i, Key j, Vector8 mpoly, Vector5 mpoint, const SharedNoiseModel& model)
        : Base(model, i, j), mpoly_(mpoly), mpoint_(mpoint) {}

    virtual ~PolyPointFactor() 
    {}

    double getPoly(const double& x) const 
    {
        double y;
        y = (0.001*pow(x,7))-(0.036*pow(x,6))+(0.294*pow(x,5))-(0.808*pow(x,4))-(0.505*pow(x,3))+(4.54*pow(x,2))-(2.492*x)+ 1.500;
        return y; // order x^7 -> x^0 
    }
       

    Vector evaluateError(const Vector8& p8, const Vector5& p5,
        boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const override 
    {
        // The measurement function for a GPS-like measurement is simple:
        // error_x = pose.x - measurement.x
        // error_y = pose.y - measurement.y
        // Consequently, the Jacobians are:
        // [ derror_x/dx  derror_x/dy  derror_x/dtheta ] = [1 0 0]
        // [ derror_y/dx  derror_y/dy  derror_y/dtheta ] = [0 1 0]
/*
       
       
        gtsam::Vector err(13); 
        err = Vector::Zero(13);

        err[9] = getPoly(p5[0]) - p5[1];
 */
        gtsam::Vector err(13); 
        err = Vector::Zero(13);

        err[0] = p8[0] - mpoly_[0];
        err[1] = p8[1] - mpoly_[1];
        err[2] = p8[2] - mpoly_[2];
        err[3] = p8[3] - mpoly_[3];
        err[4] = p8[4] - mpoly_[4];
        err[5] = p8[5] - mpoly_[5];
        err[6] = p8[6] - mpoly_[6];
        err[7] = p8[7] - mpoly_[7];
        

        err[8] = p5[0] - mpoint_[0];
        err[9] = getPoly(p5[0]) - mpoint_[1];

        err[10] = p5[2] - mpoint_[2];
        err[11] = p5[3] - mpoint_[3];
        err[12] = p5[4] - mpoint_[4];
        

        ROS_INFO_STREAM("\n x: " << p5[0] << "\n");
        ROS_INFO_STREAM("\n y: " << mpoint_[1] << "\n");
        ROS_INFO_STREAM("\n f(x): " << getPoly(p5[0]) << "\n");
        ROS_INFO_STREAM("\nerrY: " << err[9] << "\n");
        
       


        if (H1) (*H1) = (Matrix(13, 8) <<   1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished();
        
      
        double errybyx;
        errybyx = ((0.007*pow(p5[0],6))-(0.216*pow(p5[0],5))+(1.47*pow(p5[0],4))-(3.232*pow(p5[0],3))-(1.515*pow(p5[0],2))+(9.08*p5[0])-2.492);


        if (H2) (*H2) = (Matrix(13, 5) <<   0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0,
                                            1.0, 0.0, 0.0, 0.0, 0.0, 
                                            errybyx, -1.0, 0.0, 0.0, 0.0, 
                                            0.0, 0.0, 1.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 1.0, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 1.0).finished();
        
         


        
       
        return err;

    }


/*
    virtual void print(const std::string& s, const gtsam::KeyFormatter& keyFormatter) const override
    {
        std::cout << s << "PolyPointFactorOutput(" << keyFormatter(this->key1()) << "," << keyFormatter(this->key2()) << ")\n";
        //gtsam::traits<gtsam::Vector1>::Print(u_[0], "  u: ");
        model_.print("  modelParams: ");
        this->noiseModel_->print("  noise model: ");
    }



    virtual bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9) const override
    {
        const This* e = dynamic_cast<const This*>(&expected);
        return e != nullptr && Base::equals(*e, tol) &&
            gtsam::traits<gtsam::Vector8>::Equals(this->mpoly_, e->mpoly_, tol) &&
            gtsam::traits<gtsam::Vector5>::Equals(this->mpoint_, e->mpoint_, tol);
    }
    */
  // The second is a 'clone' function that allows the factor to be copied. Under most
  // circumstances, the following code that employs the default copy constructor should
  // work fine.
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new PolyPointFactor(*this))); }

  // Additionally, we encourage you the use of unit testing your custom factors,
  // (as all GTSAM factors are), in which you would need an equals and print, to satisfy the
  // GTSAM_CONCEPT_TESTABLE_INST(T) defined in Testable.h, but these are not needed below.
};  // PolyPointFactor


double getPolyY(const double& x) 
{
    double y;
    y = (0.001*pow(x,7))-(0.036*pow(x,6))+(0.294*pow(x,5))-(0.808*pow(x,4))-(0.505*pow(x,3))+(4.54*pow(x,2))-(2.492*x)+ 1.500;
    return y; // order x^7 -> x^0 
}
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


    /*------------| Rviz |-----------*/
    ::ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("GTSAM_points", 10);
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
    
    bool firstPoseMeasured = false;
    //::ros::Rate loop_rate(mheParams.loopRate);





    sleep(5);
    ::ros::Rate loop_rate(0.1);
    
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
    
        visualization_msgs::Marker points, pointsInit, PointOpt, line_strip;
        PointOpt.header.frame_id = pointsInit.header.frame_id = points.header.frame_id = line_strip.header.frame_id = "/gtsam_frame";
        points.header.stamp = line_strip.header.stamp = PointOpt.header.stamp = pointsInit.header.stamp = now;
        pointsInit.ns = PointOpt.ns = points.ns = line_strip.ns = "points_and_lines";
        pointsInit.action = PointOpt.action = points.action = line_strip.action = visualization_msgs::Marker::ADD;
        pointsInit.pose.orientation.w = PointOpt.pose.orientation.w = points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

        points.id = 0;
        line_strip.id = 1;
        pointsInit.id = 2;
        PointOpt.id = 3;

        points.type = visualization_msgs::Marker::POINTS;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        pointsInit.type = visualization_msgs::Marker::POINTS;
        PointOpt.type = visualization_msgs::Marker::POINTS;
        // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.05;
        points.scale.y = 0.05;
        points.scale.z = 0.0;
        
        pointsInit.scale.x = 0.05;
        pointsInit.scale.y = 0.05;
        pointsInit.scale.z = 0.0;

        PointOpt.scale.x = 0.05;
        PointOpt.scale.y = 0.05;
        PointOpt.scale.z = 0.0;

        // Points are green
        points.color.g = 1.0;
        points.color.a = 1.0;

        // Line strip is blue
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;

        line_strip.scale.x = 0.05;
        line_strip.scale.y = 0.05;
        line_strip.scale.z = 0.0;
        
        pointsInit.color.r = 1.0;
        pointsInit.color.a = 1.0;

        PointOpt.color.r = 1.0;
        PointOpt.color.b = 1.0;
        PointOpt.color.a = 1.0;

        

        //if(perceptionPose->pose.position.x != 0 && perceptionPose->pose.position.y != 0)
        //{
            
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
        //*************************************************//
        
        
        NonlinearFactorGraph graph;

        gtsam::Vector polyPriorSigmas(8);
        polyPriorSigmas << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        auto priorPolyNoise = gtsam::noiseModel::Diagonal::Sigmas(polyPriorSigmas);

        gtsam::Vector polyPrior(8);
        polyPrior << 0.001, -0.036, 0.294, -0.808, -0.505, 4.54, -2.492, 1.500; // order x^7 -> x^0 
        graph.addPrior(0, polyPrior, priorPolyNoise);


        gtsam::Vector pointSigmas(13);
        pointSigmas <<0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ,0.0, 0.3, 0.0, 0.0, 0.0;
        auto pointNoise = gtsam::noiseModel::Diagonal::Sigmas(pointSigmas);


        geometry_msgs::Point pois;


        gtsam::Vector po1(5);
        po1 << -1.0, 8.0, 0.0, 0.0, 0.0;
        pois.x = po1[0];
        pois.y = po1[1];
        points.points.push_back(pois);
        
        gtsam::Vector po2(5);
        po2 << -0.5, 3.8, 0.0, 0.0, 0.0;
        pois.x = po2[0];
        pois.y = po2[1];
        points.points.push_back(pois);

        gtsam::Vector po3(5);
        po3 << 0.0, 1.5, 0.0, 0.0, 0.0;
        pois.x = po3[0];
        pois.y = po3[1];
        points.points.push_back(pois);

        gtsam::Vector po4(5);
        po4 << 1.0, 2.5, 0.0, 0.0, 0.0;
        pois.x = po4[0];
        pois.y = po4[1];
        points.points.push_back(pois);

        gtsam::Vector po5(5);
        po5 << 2.0, 4.9, 0.0, 0.0, 0.0;
        pois.x = po5[0];
        pois.y = po5[1];
        points.points.push_back(pois);


        graph.emplace_shared<PolyPointFactor>(0, 1, polyPrior, po1, pointNoise);
        graph.emplace_shared<PolyPointFactor>(0, 2, polyPrior, po2, pointNoise);
        graph.emplace_shared<PolyPointFactor>(0, 3, polyPrior, po3, pointNoise);
        graph.emplace_shared<PolyPointFactor>(0, 4, polyPrior, po4, pointNoise);
        graph.emplace_shared<PolyPointFactor>(0, 5, polyPrior, po5, pointNoise);


        graph.print("\nFactor Graph:\n");  // print



        Values initialEstimate;
        geometry_msgs::Point poisInit;
        gtsam::Vector po0init(8);
        po0init << 0.001, -0.036, 0.294, -0.808, -0.505, 4.54, -2.492, 1.500;


        gtsam::Vector po1init(5);
        po1init << -1.1, 8.1, 0.0, 0.0, 0.0;
        poisInit.x = po1init[0];
        poisInit.y = po1init[1];
        pointsInit.points.push_back(poisInit);

        gtsam::Vector po2init(5);
        po2init << -0.6, 4.0, 0.0, 0.0, 0.0;
        poisInit.x = po2init[0];
        poisInit.y = po2init[1];
        pointsInit.points.push_back(poisInit);

        gtsam::Vector po3init(5);
        po3init << 0.2, 1.6, 0.0, 0.0, 0.0;
        poisInit.x = po3init[0];
        poisInit.y = po3init[1];
        pointsInit.points.push_back(poisInit);
        

        gtsam::Vector po4init(5);
        po4init << 1.2, 2.8, 0.0, 0.0, 0.0;
        poisInit.x = po4init[0];
        poisInit.y = po4init[1];
        pointsInit.points.push_back(poisInit);


        gtsam::Vector po5init(5);
        po5init << 1.8, 5.0, 0.0, 0.0, 0.0;
        poisInit.x = po5init[0];
        poisInit.y = po5init[1];
        pointsInit.points.push_back(poisInit);


        initialEstimate.insert(0, po0init);
        initialEstimate.insert(1, po1init);
        initialEstimate.insert(2, po2init);
        initialEstimate.insert(3, po3init);
        initialEstimate.insert(4, po4init);
        initialEstimate.insert(5, po5init);

        initialEstimate.print("\nInitial Estimate:\n");  // print


        // 4. Optimize using Levenberg-Marquardt optimization. The optimizer
        // accepts an optional set of configuration parameters, controlling
        // things like convergence criteria, the type of linear system solver
        // to use, and the amount of information displayed during optimization.
        // Here we will use the default set of parameters.  See the
        // documentation for the full set of parameters.
        LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
        Values result = optimizer.optimize();
        result.print("Final Result:\n");
        geometry_msgs::Point poisOpt;
        

        
         
        Vector5 x1_update = result.at<Vector5>(1);
        poisOpt.x = x1_update[0];
        poisOpt.y = x1_update[1];
        PointOpt.points.push_back(poisOpt);
        Vector5 x2_update = result.at<Vector5>(2);
        poisOpt.x = x2_update[0];
        poisOpt.y = x2_update[1];
        PointOpt.points.push_back(poisOpt);
        Vector5 x3_update = result.at<Vector5>(3);
        poisOpt.x = x3_update[0];
        poisOpt.y = x3_update[1];
        PointOpt.points.push_back(poisOpt);
        Vector5 x4_update = result.at<Vector5>(4);
        poisOpt.x = x4_update[0];
        poisOpt.y = x4_update[1];
        PointOpt.points.push_back(poisOpt);
        Vector5 x5_update = result.at<Vector5>(5);
        poisOpt.x = x5_update[0];
        poisOpt.y = x5_update[1];
        PointOpt.points.push_back(poisOpt);
        
        





        // 5. Calculate and print marginal covariances for all variables
        Marginals marginals(graph, result);
        cout << "polynomial covariance:\n" << marginals.marginalCovariance(0) << endl;
        cout << "p1 covariance:\n" << marginals.marginalCovariance(1) << endl;
        cout << "p2 covariance:\n" << marginals.marginalCovariance(2) << endl;
        cout << "p3 covariance:\n" << marginals.marginalCovariance(3) << endl;
        cout << "p4 covariance:\n" << marginals.marginalCovariance(4) << endl;
        cout << "p5 covariance:\n" << marginals.marginalCovariance(5) << endl;

        /////*************************************************////////////////
        for (double i = 0; i < 100; ++i)
        {
        geometry_msgs::Point p;
        p.x = ((double)i/10) - 5;
        p.y = getPolyY(p.x);
        p.z = 0;
        line_strip.points.push_back(p);
        }
        
        marker_pub.publish(line_strip);
        marker_pub.publish(points);
        marker_pub.publish(pointsInit);
        marker_pub.publish(PointOpt);
        /////*************************************************////////////////
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

        //}
        auto loopTime =  ::ros::Time::now() - now;
        ROS_INFO_STREAM("loop time" << loopTime.toSec());

        
       
        //::ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}
