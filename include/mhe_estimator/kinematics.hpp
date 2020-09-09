#pragma once

#include <boost/array.hpp>
#include <mhe_estimator/CanData.h>
#include <mhe_estimator/ArticulatedAngles.h>
#include "ackermann_msgs/AckermannDrive.h"
#include <tf/tf.h>


namespace mhe_estimator
{
    using namespace ast;

    typedef double Real;
    typedef boost::array<Real, 6> Vec6;
    typedef boost::array<Real, 5> Vec5;
    typedef boost::array<Real, 4> Vec4;
    typedef boost::array<Real, 3> Vec3;
    typedef boost::array<Real, 2> Vec2;


    struct CarParams
    {
        bool moveGuidancePoint, respectSteeringLimits;    
        Real L, L1, Lh1, L2, Lh2;
        Real steeringLimit, trailer1Limit, trailer2Limit, LinearVelLimit, SteeringVelLimit;
        Real upperTh, lowerTh, upperX, lowerX, upperY, lowerY;
        Real TrailerNumber;
    };

    struct MheParams
    {
        bool mheActive, WeightedActive, covarianceFromTopicStamp, perceptionGPS;
        Real loopRate;   
        Real noiseVariancePos, noiseVarianceTh, noiseVariancesteering, noiseVarianceTrailer1;
        Real noiseVarianceTrailer2, noiseVarianceLinearVel, noiseVarianceSteeringVel;
        Real WeightPos, WeightTh, WeightSteering, WeightTrailer1, WeightTrailer2;
           
    };
    
    inline int sign(Real val)
    {
      return (0 < val) - (val < 0);
    }
    
    inline Real sat(Real x, Real lim)
    {
      if(x > lim)
        return lim;
      else if(x < -lim)
        return - lim;
      else
        return x;
    }
    
    inline double continuousAngle(Real angle, Real lastAngle)
    {
        auto dAngle = fmod(angle, 2*M_PI) - fmod(lastAngle, 2*M_PI);

        if (dAngle > M_PI)
            return lastAngle + dAngle - 2.0 * M_PI;
        else if (dAngle < -M_PI)
            return lastAngle + dAngle + 2.0 * M_PI;
        else
            return lastAngle + dAngle;
    }
    //Kinematics
    inline Vec3 RDCarKinematicsGPFront(const CarParams& params,const  Vec3& q,const ackermann_msgs::AckermannDrive& u)
    {   
        Vec3 dq;
        Real vF = u.speed / cos(u.steering_angle);

        dq[0] = u.speed*(1/params.L)*tan(u.steering_angle);
        dq[1] = vF*cos(u.steering_angle + q[0]);
        dq[2] = vF*sin(u.steering_angle + q[0]);
        return dq;
    }

    inline Vec3 RDCarKinematicsGPRear(const CarParams& params,const Vec3& q,const ackermann_msgs::AckermannDrive& u)
    {
        Vec3 dq;
        dq[0] = u.speed*(1/params.L)*tan(u.steering_angle);
        dq[1] = u.speed*cos(q[0]);
        dq[2] = u.speed*sin(q[0]);
        return dq;
    }

    inline Vec4 OneTrailerKinematicsGPFront(const CarParams& params,const Vec4& q,const ackermann_msgs::AckermannDrive& u, bool brakeOnSingularity = true)
    {
        
        Vec4 dq;
        Real k1 = (1/params.L1)*tan(q[0] - atan((params.Lh1/params.L)*tan(u.steering_angle)));
        dq[0] = u.speed * (sin(q[0])/params.Lh1 - (1 + (params.L1/params.Lh1)*cos(q[0]))*k1);
        dq[1] = u.speed * ( -(params.L1/params.Lh1)*cos(q[0])*k1 + sin(q[0])/params.Lh1 );
        dq[2] = u.speed * cos(q[1]) * ( params.L1*sin(q[0])*k1 + cos(q[0]) );
        dq[3] = u.speed * sin(q[1]) * ( params.L1*sin(q[0])*k1 + cos(q[0]) );
        return dq;
    }

    inline Vec4 OneTrailerKinematicsGPRear(const CarParams& params,const Vec4& q,const ackermann_msgs::AckermannDrive& u, bool brakeOnSingularity = true)
    {
        Vec4 dq;
        Real k1 = (1/params.L1)*tan(q[0] - atan((params.Lh1/params.L)*tan(u.steering_angle)));


        dq[0] = u.speed * (sin(q[0])/params.Lh1 - (1 + (params.L1/params.Lh1)*cos(q[0]))*k1);
        dq[1] = u.speed*k1;
        dq[2] = u.speed*cos(q[1]);
        dq[3] = u.speed*sin(q[1]);
        return dq;
    }

    template <class T>
    void singleParamIn(T& ParamVar,std::string& paramName,ast::ros::NodeHandle& nh)
    {
        if (nh.hasParam(paramName))
        {
        nh.getParam(paramName,ParamVar);
        ROS_INFO_STREAM(""<<paramName<<" = "<<ParamVar<<"");
        }else
        {
        ROS_WARN_STREAM("parameter: " << paramName << " could not be found");
        }
    }

    void ParamsIn(mhe_estimator::CarParams& carParams,mhe_estimator::MheParams& mheParams,ast::ros::NodeHandle& nh)
    {
        std::string paramName;

        paramName = "/mhe_estimator/mheParam/mheActive";
        singleParamIn(mheParams.mheActive,paramName,nh);

        paramName = "/mhe_estimator/mheParam/WeightedActive";
        singleParamIn(mheParams.WeightedActive,paramName,nh);

        paramName = "/mhe_estimator/mheParam/loopRate";
        singleParamIn(mheParams.loopRate,paramName,nh);

        paramName = "/mhe_estimator/mheParam/perceptionGPS";
        singleParamIn(mheParams.perceptionGPS,paramName,nh);

        paramName = "/mhe_estimator/mheParam/covarianceFromTopicStamp";
        singleParamIn(mheParams.covarianceFromTopicStamp,paramName,nh);

        paramName = "/mhe_estimator/mheParam/noiseVariancePos";
        singleParamIn(mheParams.noiseVariancePos,paramName,nh);

        paramName = "/mhe_estimator/mheParam/noiseVarianceTh";
        singleParamIn(mheParams.noiseVarianceTh,paramName,nh);

        paramName = "/mhe_estimator/mheParam/noiseVariancesteering";
        singleParamIn(mheParams.noiseVariancesteering,paramName,nh);

        paramName = "/mhe_estimator/mheParam/noiseVarianceTrailer1";
        singleParamIn(mheParams.noiseVarianceTrailer1,paramName,nh);

        paramName = "/mhe_estimator/mheParam/noiseVarianceTrailer2";
        singleParamIn(mheParams.noiseVarianceTrailer2,paramName,nh);

        paramName = "/mhe_estimator/mheParam/noiseVarianceLinearVel";
        singleParamIn(mheParams.noiseVarianceLinearVel,paramName,nh);

        paramName = "/mhe_estimator/mheParam/noiseVarianceSteeringVel";
        singleParamIn(mheParams.noiseVarianceSteeringVel,paramName,nh);

        paramName = "/mhe_estimator/mheParam/WeightPos";
        singleParamIn(mheParams.WeightPos,paramName,nh);

        paramName = "/mhe_estimator/mheParam/WeightTh";
        singleParamIn(mheParams.WeightTh,paramName,nh);

        paramName = "/mhe_estimator/mheParam/WeightSteering";
        singleParamIn(mheParams.WeightSteering,paramName,nh);

        paramName = "/mhe_estimator/mheParam/WeightTrailer1";
        singleParamIn(mheParams.WeightTrailer1,paramName,nh);

        paramName = "/mhe_estimator/mheParam/WeightTrailer2";
        singleParamIn(mheParams.WeightTrailer2,paramName,nh);

        paramName = "/mhe_estimator/carParam/WagonNumbers";
        singleParamIn(carParams.TrailerNumber,paramName,nh);

        paramName = "/mhe_estimator/carParam/L";
        singleParamIn(carParams.L,paramName,nh);

        paramName = "/mhe_estimator/carParam/L1";
        singleParamIn(carParams.L1,paramName,nh);

        paramName = "/mhe_estimator/carParam/Lh1";
        singleParamIn(carParams.Lh1,paramName,nh);

        paramName = "/mhe_estimator/carParam/L2";
        singleParamIn(carParams.L2,paramName,nh);

        paramName = "/mhe_estimator/carParam/Lh2";
        singleParamIn(carParams.Lh2,paramName,nh);

        paramName = "/mhe_estimator/carParam/steeringLimit";
        singleParamIn(carParams.steeringLimit,paramName,nh);

        paramName = "/mhe_estimator/carParam/trailer1Limit";
        singleParamIn(carParams.trailer1Limit,paramName,nh);

        paramName = "/mhe_estimator/carParam/trailer2Limit";
        singleParamIn(carParams.trailer2Limit,paramName,nh);

        paramName = "/mhe_estimator/carParam/LinearVelLimit";
        singleParamIn(carParams.LinearVelLimit,paramName,nh);

        paramName = "/mhe_estimator/carParam/SteeringVelLimit";
        singleParamIn(carParams.SteeringVelLimit,paramName,nh);

        paramName = "/mhe_estimator/carParam/moveGuidancePoint";
        singleParamIn(carParams.moveGuidancePoint,paramName,nh);
        
        paramName = "/mhe_estimator/carParam/respectSteeringLimits";
        singleParamIn(carParams.respectSteeringLimits,paramName,nh);

        paramName = "/mhe_estimator/carParam/upperTh";
        singleParamIn(carParams.upperTh,paramName,nh);

        paramName = "/mhe_estimator/carParam/lowerTh";
        singleParamIn(carParams.lowerTh,paramName,nh);

        paramName = "/mhe_estimator/carParam/upperX";
        singleParamIn(carParams.upperX,paramName,nh);

        paramName = "/mhe_estimator/carParam/lowerX";
        singleParamIn(carParams.lowerX,paramName,nh);

        paramName = "/mhe_estimator/carParam/upperY";
        singleParamIn(carParams.upperY,paramName,nh);

        paramName = "/mhe_estimator/carParam/lowerY";
        singleParamIn(carParams.lowerY,paramName,nh);
    
        ROS_INFO_STREAM("End of receiving parameters");

    }


}
