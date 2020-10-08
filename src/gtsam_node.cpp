#include <ros/ros.h>
#include "ros_utils.hpp"
#include "kinematics.hpp"
#include "estimators.hpp"
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>
#include <Eigen/Dense>


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


using namespace std;
using namespace gtsam;


static Vector p5old(5);

static Vector errold(13);



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
       
        */
        gtsam::Vector err(13);
        err[9] = getPoly(p5[0]) - p5[1];



        ROS_INFO_STREAM("\nY: " << p5[1] << "\n");
        ROS_INFO_STREAM("\nx: " << p5[0] << "\n");
        ROS_INFO_STREAM("\nf(x): " << getPoly(p5[0]) << "\n");
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
        
        
        double errybyy;
        double errybyx;

        errybyy = (err[9] - errold[9])/(p5[1] - p5old[1]);

        errybyx = (err[9] - errold[9])/(p5[0] - p5old[0]);


        if (H2) (*H2) = (Matrix(13, 5) <<   0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0,
                                            1.0, 0.0, 0.0, 0.0, 0.0, 
                                            errybyx, -1, 0.0, 0.0, 0.0, 
                                            0.0, 0.0, 1.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 1.0, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 1.0).finished();
        
        
         


        
        p5old = p5;
        errold = err;
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

            gtsam::Vector po1(5);
            po1 << -1.0, 8.0, 0.0, 0.0, 0.0;
            gtsam::Vector po2(5);
            po2 << -0.5, 3.8, 0.0, 0.0, 0.0;
            gtsam::Vector po3(5);
            po3 << 0.0, 1.5, 0.0, 0.0, 0.0;
            gtsam::Vector po4(5);
            po4 << 1.0, 2.5, 0.0, 0.0, 0.0;
            gtsam::Vector po5(5);
            po5 << 2.0, 4.9, 0.0, 0.0, 0.0;




            graph.emplace_shared<PolyPointFactor>(0, 1, polyPrior, po1, pointNoise);
            graph.emplace_shared<PolyPointFactor>(0, 2, polyPrior, po2, pointNoise);
            graph.emplace_shared<PolyPointFactor>(0, 3, polyPrior, po3, pointNoise);
            graph.emplace_shared<PolyPointFactor>(0, 4, polyPrior, po4, pointNoise);
            graph.emplace_shared<PolyPointFactor>(0, 5, polyPrior, po5, pointNoise);


            graph.print("\nFactor Graph:\n");  // print


 


            Values initialEstimate;
            gtsam::Vector po0init(8);
            po0init << 0.001, -0.036, 0.294, -0.808, -0.505, 4.54, -2.492, 1.500;
            gtsam::Vector po1init(5);
            po1init << -1.0, 8.1, 0.0, 0.0, 0.0;
            gtsam::Vector po2init(5);
            po2init << -0.5, 4.0, 0.0, 0.0, 0.0;
            gtsam::Vector po3init(5);
            po3init << 0.0, 1.6, 0.0, 0.0, 0.0;
            gtsam::Vector po4init(5);
            po4init << 1.0, 2.8, 0.0, 0.0, 0.0;
            gtsam::Vector po5init(5);
            po5init << 2.0, 5.0, 0.0, 0.0, 0.0;

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

            // 5. Calculate and print marginal covariances for all variables
            Marginals marginals(graph, result);
            cout << "polynomial covariance:\n" << marginals.marginalCovariance(0) << endl;
            cout << "p1 covariance:\n" << marginals.marginalCovariance(1) << endl;
            cout << "p2 covariance:\n" << marginals.marginalCovariance(2) << endl;
            cout << "p3 covariance:\n" << marginals.marginalCovariance(3) << endl;
            cout << "p4 covariance:\n" << marginals.marginalCovariance(4) << endl;
            cout << "p5 covariance:\n" << marginals.marginalCovariance(5) << endl;



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

        t += 1/((Real) mheParams.loopRate);
        ::ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}
