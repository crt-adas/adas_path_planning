#include <ros/ros.h>
#include "ros_utils.hpp"
#include "kinematics.hpp"
#include "estimators.hpp"
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>
#include <Eigen/Dense>
#include <unistd.h>
#include <math.h>

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
#include <gtsam/inference/LabeledSymbol.h>

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



class CircleFactor: public NoiseModelFactor1<Vector5> {
  
  Vector3 circle_;

 public:
  
  typedef boost::shared_ptr<CircleFactor> shared_ptr;

  CircleFactor(Key j, Vector3 circle, const SharedNoiseModel& model):
    NoiseModelFactor1<Vector5>(model, j), circle_(circle) {}

  virtual ~CircleFactor() {}

  Vector evaluateError(const Vector5& po,
                       boost::optional<Matrix&> H = boost::none) const override {

    /*                 
    double err = 1/(pow(po[0]-circle_[0],2)+pow(po[1]-circle_[1],2)-pow(circle_[2],2));
    
    double ctrval = 0.05;
    double jX = (ctrval*(2*po[0] - 2*circle_[0]))/pow(pow(po[0]-circle_[0],2)+pow(po[1]-circle_[1],2)-pow(circle_[2],2),2);
    double jY = (ctrval*(2*po[1] - 2*circle_[1]))/pow(pow(po[0]-circle_[0],2)+pow(po[1]-circle_[1],2)-pow(circle_[2],2),2);
   
    if (H) (*H) = (Matrix(1, 5) << jX, jY, 0.0, 0.0, 0.0).finished();
    return (Vector(1) << err*ctrval).finished(); 
    /**/
    //
    double ctrval = 0.05;
    double err = (1/(pow(2,(ctrval*((pow(po[0]-circle_[0],2)+pow(po[1]-circle_[1],2)-pow(circle_[2],2))/pow(circle_[2],2))))))-1;
    
    
    double jX = (-0.6931471*ctrval*(po[0]-circle_[0])*pow(2,(1-(ctrval*((pow(po[0]-circle_[0],2)+pow(po[1]-circle_[1],2)-pow(circle_[2],2))/pow(circle_[2],2)) ))))/pow(circle_[2],2);
    double jY = (-0.6931471*ctrval*(po[1]-circle_[1])*pow(2,(1-(ctrval*((pow(po[0]-circle_[0],2)+pow(po[1]-circle_[1],2)-pow(circle_[2],2))/pow(circle_[2],2)) ))))/pow(circle_[2],2);
   
    if (H) (*H) = (Matrix(1, 5) << jX, jY, 0.0, 0.0, 0.0).finished();
    //ROS_INFO_STREAM("errY: " << err << "");
    //ROS_INFO_STREAM("errYJ: " << jY << "");
    return (Vector(1) << err).finished();
    /**/ 
  }

  

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new CircleFactor(*this))); }

};  // CircleFactor





class PolyPointFactor: public NoiseModelFactor2<Vector8,Vector5> {
  
  private:
    using This = PolyPointFactor;
    using Base = gtsam::NoiseModelFactor2<Vector8, Vector5>;

    Vector8 mpoly_;
    Vector5 mpoint_;
    gtsam::Key j_;
  public:
        
    typedef boost::shared_ptr<PolyPointFactor> shared_ptr;

    PolyPointFactor(Key i, Key j, Vector8 mpoly, Vector5 mpoint, const SharedNoiseModel& model)
        : Base(model, i, j), mpoly_(mpoly), mpoint_(mpoint),j_(j) {}

    virtual ~PolyPointFactor() 
    {}

    double getPoly(const double& x, const Vector8& p8) const 
    {
        double y;
        y = (p8[0]*pow(x,7))+(p8[1]*pow(x,6))+(p8[2]*pow(x,5))+(p8[3]*pow(x,4))+(p8[4]*pow(x,3))+(p8[5]*pow(x,2))+(p8[6]*x)+p8[7];

        
        return y; // order p0 x^7 -> p7 x^1 
    }

    double diffPoly(const double& x, const Vector8& p8) const 
    {
        double y;
        y = (7*p8[0]*pow(x,6))+(6*p8[1]*pow(x,5))+(5*p8[2]*pow(x,4))+(4*p8[3]*pow(x,3))+(3*p8[4]*pow(x,2))+(2*p8[5]*x)+p8[6];
        
        return y; // order p0 x^7 -> p7 x^1 
    }  

    double diff2Poly(const double& x, const Vector8& p8) const 
    {
        double y;
        y = (42*p8[0]*pow(x,5))+(30*p8[1]*pow(x,4))+(20*p8[2]*pow(x,3))+(12*p8[3]*pow(x,2))+(6*p8[4]*x)+(2*p8[5]);
        
        return y; // order p0 x^7 -> p7 x^1 
    }  



     Vector evaluateError(const Vector8& p8, const Vector5& p5,
        boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const override 
    {
        //PrintKey(j_);
        gtsam::Vector err(13); 
        err = Vector::Zero(13);
        err[9] =  p5[1] - getPoly(p5[0],p8);
        //ROS_INFO_STREAM("errY: " << err[9] << "");

        double errYx = diffPoly(p5[0],p8);
        double errY2x = diff2Poly(p5[0],p8);
        double curve = abs(errY2x)/pow((pow(errYx,2)+1),1.5);
        double ctrVal = 1;
        double curveMax = 0.1;
        //err[11] =  1/(pow(2,ctrVal*(curveMax - curve)));
        err[9] = err[9] + 1/(pow(2,ctrVal*(curveMax - curve)));
        ROS_INFO_STREAM("Curve: " << curve << "");
        //ROS_INFO_STREAM("errCurve: " << err[11] << "");

        //curve = (abs(42*p0*x^5+30*p1*x^4+20*p2*x^3+12*p3*x^2+6*p4*x+2*p5)/(1+(7*p0*x^6+6*p1*x^5+5p2*x^4+4*p3*x^3+3*p4*x^2+2*p5*x+p6)^2)^(3/2))

        //errCurve = 1/2^(a*(m-(((abs(42*p0*x^5+30*p1*x^4+20*p2*x^3+12*p3*x^2+6*p4*x+2*p5)/(1+(7*p0*x^6+6*p1*x^5+5p2*x^4+4*p3*x^3+3*p4*x^2+2*p5*x+p6)^2)^(3/2))))
        
/**/    
        gtsam::Vector crvJp(7); 
        crvJp = Vector::Zero(7);
        double lnCtr = -0.6931471*ctrVal;
        double deno = (pow(2,ctrVal*(curveMax - curve)));
        double eqp1 = (errYx* abs(errY2x))/(pow( (pow(errYx,2)+1),2.5));
        double eqp2 = pow( (pow(errYx,2)+1) ,1.5) *abs(errY2x);


        crvJp[0] = (lnCtr*(( (21* pow(p5[0],6)* eqp1)  - ((42*pow(p5[0],5) *  errY2x)  / eqp2))))   / deno;
       
        crvJp[1] = (lnCtr*(( (18* pow(p5[0],5)* eqp1)  - ((30*pow(p5[0],4) *  errY2x)  / eqp2))))   / deno;
        
        crvJp[2] = (lnCtr*(( (15* pow(p5[0],4)* eqp1)  - ((20*pow(p5[0],3) *  errY2x)  / eqp2))))   / deno;
       
        crvJp[3] = (lnCtr*(( (12* pow(p5[0],3)* eqp1)  - ((12*pow(p5[0],2) *  errY2x)  / eqp2))))   / deno;
      
        crvJp[4] = (lnCtr*(( (9*  pow(p5[0],2)* eqp1)  - ((6*      p5[0]   *  errY2x)  / eqp2))))   / deno;
   
        crvJp[5] = (lnCtr*(( (6*      p5[0]*    eqp1)  - ((2*                 errY2x)  / eqp2))))   / deno;
       
        crvJp[6] = (lnCtr*   3*     errYx* abs(errY2x)) / (pow( (pow(errYx,2)+1),2.5)*deno);
        /*
        if (isnan(crvJp[0])) {crvJp[0] = 0.0;}
        if (isnan(crvJp[1])) {crvJp[1] = 0.0;}
        if (isnan(crvJp[2])) {crvJp[2] = 0.0;}
        if (isnan(crvJp[3])) {crvJp[3] = 0.0;}
        if (isnan(crvJp[4])) {crvJp[4] = 0.0;}
        if (isnan(crvJp[5])) {crvJp[5] = 0.0;}
        if (isnan(crvJp[6])) {crvJp[6] = 0.0;}
        
        /*
        ROS_INFO_STREAM("crvJp6: " << crvJp[6] << "");
        ROS_INFO_STREAM("crvJp5: " << crvJp[5] << "");
        ROS_INFO_STREAM("crvJp4: " << crvJp[4] << "");
        ROS_INFO_STREAM("crvJp3: " << crvJp[3] << "");
        ROS_INFO_STREAM("crvJp2: " << crvJp[2] << "");
        ROS_INFO_STREAM("crvJp1: " << crvJp[1] << "");
        ROS_INFO_STREAM("crvJp0: " << crvJp[0] << "");
        */


        if (H1) (*H1) = (Matrix(13, 8) <<   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                            -pow(p5[0],7)+crvJp[0], -pow(p5[0],6)+crvJp[1], -pow(p5[0],5)+crvJp[2], -pow(p5[0],4)+crvJp[3], -pow(p5[0],3)+crvJp[4], -pow(p5[0],2)+crvJp[5], -p5[0]+crvJp[6], -1.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished();
          //crvJp0, crvJp1, crvJp2, crvJp3, crvJp4, crvJp5, crvJp6, 0.0,
          //crvJp[0], crvJp[1], crvJp[2], crvJp[3], crvJp[4], crvJp[5], crvJp[6], 0.0,
        //(-1)*errYx
        //double kdot = (0.6931471*ctrVal)/(pow(2,ctrVal*( curveMax - curve)));
        //-pow(p5[0],7), -pow(p5[0],6), -pow(p5[0],5), -pow(p5[0],4), -pow(p5[0],3), -pow(p5[0],2), -p5[0], -1.0,
        if (H2) (*H2) = (Matrix(13, 5) <<   0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0,                                            
                                            0.0, 0.0, 0.0, 0.0, 0.0, 
                                            0.0, 1.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 1.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 1.0, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 1.0).finished();
        return err;
    }
   
    gtsam::NonlinearFactor::shared_ptr clone() const override {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new PolyPointFactor(*this))); }

};  // PolyPointFactor





class ppFactor: public NoiseModelFactor2<Vector5,Vector5> {
  
  private:
    using This = ppFactor;
    using Base = gtsam::NoiseModelFactor2<Vector5, Vector5>;
  public:
    typedef boost::shared_ptr<ppFactor> shared_ptr;
    ppFactor(Key i, Key j, Vector5 pointA, Vector5 pointB, const SharedNoiseModel& model)
        : Base(model, i, j) {}

    virtual ~ppFactor() 
    {}
     Vector evaluateError(const Vector5& pA, const Vector5& pB,
        boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const override 
    {
        
        gtsam::Vector err(10); 
        err = Vector::Zero(10);
        double dis = 0.1;
        err[0] =  pA[0] - pB[0] + dis ;
        err[5] =  pB[0] - pA[0] - dis ;

        if (H1) (*H1) = (Matrix(10, 5) <<   1.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0,
                                            -1.0, 0.0, 0.0, 0.0, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0).finished();
      
        if (H2) (*H2) = (Matrix(10, 5) <<   -1.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0,
                                            1.0, 0.0, 0.0, 0.0, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0,                                            
                                            0.0, 0.0, 0.0, 0.0, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 0.0).finished();
        return err;
    }
   
    gtsam::NonlinearFactor::shared_ptr clone() const override {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new ppFactor(*this))); }

};  // point<->point Factor




double getPolyY(const double& x, const Vector8& p8)  //only for illustration purposes 
{
    double y;
    y = (p8[0]*pow(x,7))+(p8[1]*pow(x,6))+(p8[2]*pow(x,5))+(p8[3]*pow(x,4))+(p8[4]*pow(x,3))+(p8[5]*pow(x,2))+(p8[6]*x)+ p8[7];
    return y; // order x^7 -> x^0 
}

int main(int argc, char **argv)
{

    using namespace gtsam_node;
    using namespace ast;
    using namespace ast::ros;
    ::ros::init(argc, argv, "gtsam_node");
    NodeHandle nh;
    
    /*--------------------------------------------| Get Params |---------------------------------------------*/
    CarParams carParams;
    MheParams mheParams;
    ParamsIn(carParams, mheParams, nh); //TODO: change mhe to graph (just old name )
    /*-------------------------------------------------------------------------------------------------------*/
    
    /*-----------------------------------------------| Rviz |-------------------------------------------------*/
    ::ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("GTSAM_points", 10);
    /*--------------------------------------------------------------------------------------------------------*/
    
    ROS_INFO_STREAM("Estimator loop rate: "<< mheParams.loopRate <<"");

    sleep(2);
    ::ros::Rate loop_rate(mheParams.loopRate);
    while(::ros::ok())
    {
        auto now = ::ros::Time::now();
        /*---------------------------------| Rviz initialization |--------------------------------------------*/
        visualization_msgs::Marker points, pointsInit, PointOpt, line_strip, circleObst;
        circleObst.header.frame_id = PointOpt.header.frame_id = pointsInit.header.frame_id = points.header.frame_id = line_strip.header.frame_id = "/gtsam_frame";
        circleObst.header.stamp = points.header.stamp = line_strip.header.stamp = PointOpt.header.stamp = pointsInit.header.stamp = now;
        circleObst.ns = pointsInit.ns = PointOpt.ns = points.ns = line_strip.ns = "points_and_lines";
        circleObst.action = pointsInit.action = PointOpt.action = points.action = line_strip.action = visualization_msgs::Marker::ADD;
        circleObst.pose.orientation.w = pointsInit.pose.orientation.w = PointOpt.pose.orientation.w = points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

        points.id = 0; line_strip.id = 1; pointsInit.id = 2; PointOpt.id = 3; circleObst.id = 4;
        points.type = visualization_msgs::Marker::POINTS;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        pointsInit.type = visualization_msgs::Marker::POINTS;
        PointOpt.type = visualization_msgs::Marker::POINTS;
        circleObst.type = visualization_msgs::Marker::LINE_STRIP;
        // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.2; points.scale.y = 0.2;  points.scale.z = 0.0;
        pointsInit.scale.x = 0.05; pointsInit.scale.y = 0.05; pointsInit.scale.z = 0.0;
        PointOpt.scale.x = 0.05; PointOpt.scale.y = 0.05; PointOpt.scale.z = 0.0;
        points.color.g = 1.0; points.color.a = 1.0;
        line_strip.color.b = 1.0; line_strip.color.a = 1.0;
        line_strip.scale.x = 0.02; line_strip.scale.y = 0.02; line_strip.scale.z = 0.0;
        circleObst.scale.x = 0.01;circleObst.scale.y = 0.01;circleObst.scale.z = 0.0; circleObst.color.a = 1.0;
        circleObst.color.b = 1.0; circleObst.color.g = 1.0;
        pointsInit.color.r = 1.0; pointsInit.color.a = 1.0;
        PointOpt.color.r = 1.0; PointOpt.color.b = 1.0; PointOpt.color.a = 1.0;
        /*--------------------------------------------------------------------------------------------------------*/


        NonlinearFactorGraph graph;
        /*---------------------------------------| polynomal node |-----------------------------------------------*/
        gtsam::Vector polyPriorSigmas(8);
        polyPriorSigmas << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ;
        auto priorPolyNoise = gtsam::noiseModel::Diagonal::Sigmas(polyPriorSigmas);
        gtsam::Vector polyParams(8);
        polyParams << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ; // order x^7 -> x^0 
        //polyParams << 0.001, -0.036, 0.294, -0.808, -0.505, 4.54, -2.492, 1.500; // order x^7 -> x^0 
        //graph.addPrior(Symbol('l', 0), polyParams, priorPolyNoise);
        /*--------------------------------------------------------------------------------------------------------*/

        /*--------------------------------| polynomal<->point factor |--------------------------------------------*/
        gtsam::Vector polyPointSigmas(13);
        polyPointSigmas << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ;
        auto polyPointNoise = gtsam::noiseModel::Diagonal::Sigmas(polyPointSigmas);

        gtsam::Vector po1(5), po2(5), poj(5) ;
        po1 << -3.0, 3.0, 0.0, 0.5, 0.0;
        poj << 0.0, 0.0, 0.0, 0.5, 0.0;
        po2 << 3.0, -3.0, 0.0, 0.5, 0.0;
        gtsam::Vector poPriorSigmas(5);
        poPriorSigmas << 0.0, 0.0, 0.0, 0.0, 0.0;
        auto priorPoNoise = gtsam::noiseModel::Diagonal::Sigmas(poPriorSigmas);
        graph.addPrior(Symbol('p', 0), po1, priorPoNoise);   //add prior on first point
        graph.addPrior(Symbol('p', 59), po2, priorPoNoise);   //add prior on first point
        geometry_msgs::Point pois; 
        pois.x = po1[0];
        pois.y = po1[1];
        points.points.push_back(pois);
        pois.x = po2[0];
        pois.y = po2[1];
        points.points.push_back(pois);

        std::vector<gtsam::Vector5> poses;
        for(size_t j = 0; j < 60; ++j) // create vector of poses
        {
          poses.push_back(poj);
        }
        
        for(size_t j = 0; j < poses.size(); ++j) //add polynoaml<->point factors to the graph
        {
          graph.push_back(boost::make_shared<PolyPointFactor>(Symbol('l', 0), Symbol('p', j), polyParams, poses[j], polyPointNoise));
          ROS_INFO_STREAM("key: "<< j <<"");
        }
        /*--------------------------------------------------------------------------------------------------------*/


        /*-----------------------------------| point<->point factor |---------------------------------------------*/
        gtsam::Vector ppSigmas(10);
        ppSigmas << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        auto ppNoise = gtsam::noiseModel::Diagonal::Sigmas(ppSigmas);

        for (size_t j = 0; j < (poses.size()-1); ++j)   //add distance factor in x direction between each neighbouring point 
        {
          graph.push_back(boost::make_shared<ppFactor>(Symbol('p', j), Symbol('p', j+1), poses[j], poses[j+1], ppNoise));
        }
        /*--------------------------------------------------------------------------------------------------------*/
        

        /*--------------------------------------| obstacle  factor |----------------------------------------------*/
        auto circleNoise = noiseModel::Diagonal::Sigmas(Vector1(0.0));  // 10cm std on x,y
        gtsam::Vector circle0(3),circle1(3); //x, y, r
        circle0 <<  0.5, 2.6, 0.5;
        circle1 << 0.2, 0.0, 0.6;
        
        for(size_t j = 0; j < poses.size(); ++j) 
        {
          graph.push_back(boost::make_shared<CircleFactor>(Symbol('p', j), circle0, circleNoise));
          graph.push_back(boost::make_shared<CircleFactor>(Symbol('p', j), circle1, circleNoise));
        }
        /*--------------------------------------------------------------------------------------------------------*/
        ROS_INFO_STREAM("/////////////////////////////////////////////////////////");
        //graph.print("\nFactor Graph:\n");  // cout the graph

        /*----------------------------| graph optimization initialization |----------------------------------------*/
        Values initialEstimate;
        
        gtsam::Vector po0init(8);
        //po0init << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; 
        po0init << 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0;
        initialEstimate.insert(Symbol('l', 0), po0init);

        geometry_msgs::Point poisInit;
        gtsam::Vector po1init(5);
        po1init << -3.1, 3.1, 0.0, 0.5, 0.0;
        poisInit.x = po1init[0];
        poisInit.y = po1init[1];
        pointsInit.points.push_back(poisInit);
        
        for (size_t j = 0; j < poses.size(); ++j)   //points.size()
        {
          initialEstimate.insert(Symbol('p', j), po1init);
        }
        //initialEstimate.print("\nInitial Estimate:\n");  // print
        /*--------------------------------------------------------------------------------------------------------*/
        



        /*-------------------------------------| optimize the graph  |---------------------------------------------*/
        LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
        Values result = optimizer.optimize();
        //result.print("Final Result:\n");
        geometry_msgs::Point poisOpt;
        
        Vector8 polyUpdate = result.at<Vector8>(Symbol('l', 0));
        for (size_t j = 0; j < poses.size(); ++j)   //points.size()
        {
          Vector5 x_update = result.at<Vector5>(Symbol('p', j));
          poisOpt.x = x_update[0];
          poisOpt.y = x_update[1];
          PointOpt.points.push_back(poisOpt);
        }

        Marginals marginals(graph, result);
        /*
        cout << "polynomial covariance:\n" << marginals.marginalCovariance(Symbol('l', 0)) << endl;
        cout << "p1 covariance:\n" << marginals.marginalCovariance(Symbol('p', 0)) << endl;
        cout << "p2 covariance:\n" << marginals.marginalCovariance(Symbol('p', 1)) << endl;
        cout << "p3 covariance:\n" << marginals.marginalCovariance(Symbol('p', 2)) << endl;
        cout << "p4 covariance:\n" << marginals.marginalCovariance(Symbol('p', 3)) << endl;
        cout << "p5 covariance:\n" << marginals.marginalCovariance(Symbol('p', 4)) << endl;
        /**/
        /*--------------------------------------------------------------------------------------------------------*/

        /*------------------------------------| graph result to rviz  |-------------------------------------------*/
        
        auto circle1bst = circleObst;
        circle1bst.id = 5;
        for (double i = 0; i < 100; ++i)
        {
        geometry_msgs::Point p;
        p.x = ((double)i/10) - 5;
        p.y = getPolyY(p.x, polyUpdate);
        p.z = 0;
        line_strip.points.push_back(p);
        }
        for (double i = 0; i < (2 * M_PI); i = i + (M_PI/50))
        {
        geometry_msgs::Point p;
        p.x = circle0[2] * cos(i) + circle0[0];
        p.y = circle0[2] * sin(i) + circle0[1];
        p.z = 0;
        circleObst.points.push_back(p);
        }
        
        for (double i = 0; i < (2 * M_PI); i = i + (M_PI/50))
        {
        geometry_msgs::Point p;
        p.x = circle1[2] * cos(i) + circle1[0];
        p.y = circle1[2] * sin(i) + circle1[1];
        p.z = 0;
        circle1bst.points.push_back(p);
        }
        

        marker_pub.publish(circleObst);
        marker_pub.publish(circle1bst);
        marker_pub.publish(line_strip);
        marker_pub.publish(points);
        //marker_pub.publish(pointsInit);
        marker_pub.publish(PointOpt);

        /*--------------------------------------------------------------------------------------------------------*/
        auto loopTime =  ::ros::Time::now() - now;
        ROS_INFO_STREAM("loop time" << loopTime.toSec());

        ::ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}

/*
gtsam::Vector crvJp(7); 
        crvJp = Vector::Zero(7);
        crvJp[0] = (-0.6931471*ctrVal*(((21* pow(p5[0],6)* errYx*  sqrt( pow(errYx,2)  +1))/ abs(errY2x))  - ((42*pow(p5[0],5) *  pow( (pow(errYx,2)+1) ,1.5))  / (errY2x*abs(errY2x))  )))  / (pow(2,ctrVal*(p5[3] - curveMax)));
        //if (isnan(crvJp[0])) {crvJp[0] = 0.0;}
        crvJp[1] = (-0.6931471*ctrVal*(((18* pow(p5[0],5)* errYx*  sqrt( pow(errYx,2)  +1))/ abs(errY2x))  - ((30*pow(p5[0],4) *  pow( (pow(errYx,2)+1) ,1.5))  / (errY2x*abs(errY2x))  )))  / (pow(2,ctrVal*(p5[3] - curveMax)));
        //if (isnan(crvJp[1])) {crvJp[1] = 0.0;}
        crvJp[2] = (-0.6931471*ctrVal*(((15* pow(p5[0],4)* errYx*  sqrt( pow(errYx,2)  +1))/ abs(errY2x))  - ((20*pow(p5[0],3) *  pow( (pow(errYx,2)+1) ,1.5))  / (errY2x*abs(errY2x))  )))  / (pow(2,ctrVal*(p5[3] - curveMax)));
        //if (isnan(crvJp[2])) {crvJp[2] = 0.0;}
        crvJp[3] = (-0.6931471*ctrVal*(((12* pow(p5[0],3)* errYx*  sqrt( pow(errYx,2)  +1))/ abs(errY2x))  - ((12*pow(p5[0],2) *  pow( (pow(errYx,2)+1) ,1.5))  / (errY2x*abs(errY2x))  )))  / (pow(2,ctrVal*(p5[3] - curveMax)));
        //if (isnan(crvJp[3])) {crvJp[3] = 0.0;}
        crvJp[4] = (-0.6931471*ctrVal*(((9*  pow(p5[0],2)* errYx*  sqrt( pow(errYx,2)  +1))/ abs(errY2x))  - ((6*      p5[0]   *  pow( (pow(errYx,2)+1) ,1.5))  / (errY2x*abs(errY2x))  )))  / (pow(2,ctrVal*(p5[3] - curveMax)));
        //if (isnan(crvJp[4])) {crvJp[4] = 0.0;}
        crvJp[5] = (-0.6931471*ctrVal*(((6*      p5[0]*    errYx*  sqrt( pow(errYx,2)  +1))/ abs(errY2x))  - ((2*                 pow( (pow(errYx,2)+1) ,1.5))  / (errY2x*abs(errY2x))  )))  / (pow(2,ctrVal*(p5[3] - curveMax)));
        //if (isnan(crvJp[5])) {crvJp[5] = 0.0;}
        crvJp[6] = (-0.6931471*ctrVal*   3*                errYx*  sqrt( pow(errYx,2)  +1))                                                                                     /(abs(errY2x)*pow(2,ctrVal*(p5[3] - curveMax)));
        //if (isnan(crvJp[6])) {crvJp[6] = 0.0;}
        */

        