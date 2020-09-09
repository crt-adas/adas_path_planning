
#include "ros_utils.hpp"
#include "kinematics.hpp"
#include <boost/array.hpp>

#include <casadi/casadi.hpp>

#include <gtsam_node/CanData.h>
#include <gtsam_node/ArticulatedAngles.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "ackermann_msgs/AckermannDrive.h"


namespace gtsam_node
{
   
    void estimateEst(Vec3& qCarEst,const Vec3& qLoc,const Vec3& qPred,const MheParams& estParams)
    {   
           
        qCarEst[0] = estParams.WeightTh*qPred[0] + (1-estParams.WeightTh)*qLoc[0];
        qCarEst[1] = estParams.WeightPos*qPred[1] + (1-estParams.WeightPos)*qLoc[1];
        qCarEst[2] = estParams.WeightPos*qPred[2] + (1-estParams.WeightPos)*qLoc[2];
        
    }

    void estimateEstTrailer(Vec4& qTrailerEst,const Vec4& qTrailerLoc,const Vec4& qTrailerPred,const MheParams& estParams)
    {   
        qTrailerEst[0] = estParams.WeightTrailer1*qTrailerPred[0] + (1-estParams.WeightTrailer1)*qTrailerLoc[0];           
        qTrailerEst[1] = estParams.WeightTh*qTrailerPred[1] + (1-estParams.WeightTh)*qTrailerLoc[1];
        qTrailerEst[2] = estParams.WeightPos*qTrailerPred[2] + (1-estParams.WeightPos)*qTrailerLoc[2];
        qTrailerEst[3] = estParams.WeightPos*qTrailerPred[3] + (1-estParams.WeightPos)*qTrailerLoc[3];
    }
}
