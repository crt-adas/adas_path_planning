
#include "ros_utils.hpp"
#include "kinematics.hpp"
#include <boost/array.hpp>

#include <casadi/casadi.hpp>

#include <mhe_estimator/CanData.h>
#include <mhe_estimator/ArticulatedAngles.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "ackermann_msgs/AckermannDrive.h"


namespace mhe_estimator
{
    using namespace casadi;

    void mheSetupCar(casadi::Function& solver, const mhe_estimator::CarParams& params, const mhe_estimator::MheParams& mheParams,const long unsigned int N_mhe)
    {
        
        double A = N_mhe;
        double B = N_mhe+1;
        SX theta = SX::sym("theta"); SX x = SX::sym("x"); SX y = SX::sym("y"); SX beta0 = SX::sym("beta0");  
        SX u1 = SX::sym("u1"); SX u2 = SX::sym("u2"); 

        SX states = vertcat(theta, x, y, beta0); 
        SX controls = vertcat(u1, u2);  //u1 =dbeta     u2= linear vel 
        unsigned int n_states = states.size1(); unsigned int n_controls = controls.size1();

        SX rhs = vertcat(u2*(1/params.L)*tan(beta0), u2*cos(theta), u2*sin(theta), u1); //system r.h.s  theta, x, y, beta0
        Function f = Function("f",{states,controls}, {rhs}); //nonlinear mapping function f(x,u)
        
        SX U = SX::sym("U",n_controls,A);     //A = N_MHE 
        SX X = SX::sym("X",n_states,B);       //B = N_MHE +1
        SX P = SX::sym("P",n_states,B+A+B+A); //state,control,covOfState,covOfControl 
        
        SX obj = 0;
        SX g;

        for(int k = 0; k < B; k++) // Create states objective function 
        {
            SX h_x = vertcat(X(0,k), X(1,k), X(2,k), X(3,k));
            SX y_tilde = vertcat(P(0,k), P(1,k), P(2,k), P(3,k));
            SX V = SX::zeros(n_states,n_states);
            V(0,0) = P(0,B+A+k); 
            V(1,1) = P(1,B+A+k); 
            V(2,2) = P(2,B+A+k); 
            V(3,3) = P(3,B+A+k);
            obj = obj + mtimes(mtimes(((y_tilde - h_x).T()), V), (y_tilde - h_x));
        }
        for(int k = 0; k < A; k++) // Create control objective function 
        {
            SX con = vertcat(U(0,k), U(1,k));
            SX u_tilde = vertcat(P(0,B+k), P(1,B+k));
            SX W = SX::zeros(n_controls,n_controls);
            W(0,0) = P(0,B+A+B+k); 
            W(1,1) = P(1,B+A+B+k); 
            obj = obj + mtimes(mtimes(((u_tilde - con).T()), W), (u_tilde - con));
        }

        for(int k = 0; k < A; k++) //  multiple shooting constraints
        {
            SX con = vertcat(U(0,k), U(1,k));
            SX st = vertcat(X(0,k), X(1,k), X(2,k), X(3,k));
            SX st_next = vertcat(X(0,k+1), X(1,k+1), X(2,k+1), X(3,k+1));
            SXVector func = f(SXVector{st,con});
            SX f_value = SX::vertcat(func);
            SX st_next_euler = st + ((1/mheParams.loopRate)*f_value);
            g = vertcat(g, (st_next-st_next_euler));
        }

        SX OPT_variables; //optimization variable

        for(int j = 0; j < X.size2(); j++) //reshape X and apend
        {
            for(int i = 0; i < X.size1(); i++)
            {
                OPT_variables = vertcat(OPT_variables, X(i,j));
            }
        }
        for(int j = 0; j < U.size2(); j++) //reshape U and apend
        {
            for(int i = 0; i < U.size1(); i++)
            {
                OPT_variables = vertcat(OPT_variables, U(i,j));
            }
        }

        
        SXDict nlp = {{"f", obj}, {"x", OPT_variables}, {"g", g}, {"p", P}};
        Dict opts;
        /*
        opts["qpsol"] = "qpoases";
//      opts["max_iter"] =2;

        opts["print_iteration"] = true;
        Dict qopts;
        qopts["sparse"]=true;

        opts["qpsol_options"] = qopts;

        */
        opts["ipopt.max_iter"] = 2000;
        opts["ipopt.print_level"] = 0;
        opts["print_time"] = false;
        opts["ipopt.acceptable_tol"] = 1e-6;
        opts["ipopt.acceptable_obj_change_tol"] = 1e-4;
        opts["ipopt.print_timing_statistics"] = "no";
        opts["ipopt.print_info_string"] = "no";

        solver = nlpsol("nlpsol", "ipopt", nlp, opts);


        //solver = nlpsol("nlpsol", "sqpmethod", nlp, opts);
        
    }

    void mheSetupTrailer(casadi::Function& solver,const mhe_estimator::CarParams& params,const mhe_estimator::MheParams& mheParams,const long unsigned int N_mhe)
    {
        double A = N_mhe;
        double B = N_mhe+1;
        SX beta1 = SX::sym("beta1"); SX theta = SX::sym("theta"); SX x = SX::sym("x"); SX y = SX::sym("y"); SX beta0 = SX::sym("beta0"); 
        SX u1 = SX::sym("u1"); SX u2 = SX::sym("u2"); 

        SX states1 = vertcat(beta1,theta, x); 
        SX states2 = vertcat(y, beta0); 
        SX states = vertcat(states1, states2); // due to casadi limitation
        SX controls = vertcat(u1, u2);  //u1 =dbeta     u2= linear vel 
        unsigned int n_states = states.size1(); unsigned int n_controls = controls.size1();

        SX rhs;
        SX k1 = (1/params.L1)*tan(beta1 - atan((params.Lh1/params.L)*tan(beta0)));
        if(!params.moveGuidancePoint)  //OneTrailerKinematicsGPRear
        {
            SX dq0 = u2 * (sin(beta1)/params.Lh1 - (1 + (params.L1/params.Lh1)*cos(beta1))*(k1)); //beta1
            SX dq1 = u2 * k1; //theta
            SX dq2 = u2 * cos(theta); //x
            SX dq3 = u2 * sin(theta); //y
            SX dq4 = u1; //beta0
            
            SX rhs1 = vertcat(dq0, dq1, dq2); //due to casadi limitation 
            SX rhs2 = vertcat(dq3, dq4);
            rhs = vertcat(rhs1, rhs2); //system r.h.s
        } 
        else    //OneTrailerKinematicsGPFront
        {
            SX dq0 = u2 * (sin(beta1)/params.Lh1 - (1 + (params.L1/params.Lh1)*cos(beta1))*k1); //beta1
            SX dq1 = u2 * ( -(params.L1/params.Lh1)*cos(beta1)*k1 + sin(beta1)/params.Lh1 ); //theta
            SX dq2 = u2 * cos(theta) * ( params.L1*sin(beta1)*k1 + cos(beta1) ); //x
            SX dq3 = u2 * sin(theta) * ( params.L1*sin(beta1)*k1 + cos(beta1) ); //y
            SX dq4 = u1; //beta0
            
            
            SX rhs1 = vertcat(dq0, dq1, dq2); //due to casadi limitation 
            SX rhs2 = vertcat(dq3, dq4);
            rhs = vertcat(rhs1, rhs2); //system r.h.s
        }
            
        //ROS_INFO_STREAM("rhstRAILER: " << rhs <<" ");
        
        Function f = Function("f",{states,controls}, {rhs}); //nonlinear mapping function f(x,u)
        
        SX U = SX::sym("U",n_controls,A); //A = N_MHE 
        SX X = SX::sym("X",n_states,B); //B = N_MHE +1
        SX P = SX::sym("P",n_states,B+A+B+A); 
            
        SX obj = 0;
        SX g;
        
        for(int k = 0; k < B; k++) // Create states objective function 
        {
            SX h_x1 = vertcat(X(0,k), X(1,k), X(2,k));
            SX h_x2 = vertcat(X(3,k), X(4,k) );
            SX h_x = vertcat(h_x1, h_x2);

            SX y_tilde1 = vertcat(P(0,k), P(1,k), P(2,k));
            SX y_tilde2 = vertcat(P(3,k), P(4,k));
            SX y_tilde = vertcat(y_tilde1, y_tilde2);

            SX V = SX::zeros(n_states,n_states);
            V(0,0) = P(0,B+A+k); 
            V(1,1) = P(1,B+A+k); 
            V(2,2) = P(2,B+A+k); 
            V(3,3) = P(3,B+A+k);
            V(4,4) = P(4,B+A+k);
            obj = obj + mtimes(mtimes(((y_tilde - h_x).T()), V), (y_tilde - h_x));
        }
        for(int k = 0; k < A; k++) // Create control objective function 
        {
            SX con = vertcat(U(0,k), U(1,k));
            SX u_tilde = vertcat(P(0,B+k), P(1,B+k));
            SX W = SX::zeros(n_controls,n_controls);
            W(0,0) = P(0,B+A+B+k); 
            W(1,1) = P(1,B+A+B+k); 
            obj = obj + mtimes(mtimes(((u_tilde - con).T()), W), (u_tilde - con));
        }

        for(int k = 0; k < A; k++) //  multiple shooting constraints
        {
            SX con = vertcat(U(0,k), U(1,k));

            SX st1 = vertcat(X(0,k), X(1,k), X(2,k));
            SX st2 = vertcat(X(3,k), X(4,k));
            SX st = vertcat(st1,st2);


            SX st_next1 = vertcat(X(0,k+1), X(1,k+1), X(2,k+1));
            SX st_next2 = vertcat(X(3,k+1), X(4,k+1));
            SX st_next = vertcat(st_next1, st_next2);

            SXVector func = f(SXVector{st,con});
            SX f_value = SX::vertcat(func);
            SX st_next_euler = st + ((1/mheParams.loopRate)*f_value);
            g = vertcat(g, (st_next-st_next_euler));
        }

        //ROS_INFO_STREAM("g " << g <<" ");
        
        SX OPT_variables; //optimization variable
        
        for(int j = 0; j < X.size2(); j++) //reshape X and apend
        {
            for(int i = 0; i < X.size1(); i++)
            {
                OPT_variables = vertcat(OPT_variables, X(i,j));
            }
        }
        for(int j = 0; j < U.size2(); j++) //reshape U and apend
        {
            for(int i = 0; i < U.size1(); i++)
            {
                OPT_variables = vertcat(OPT_variables, U(i,j));
            }
        }
        //ROS_INFO_STREAM("g.size " << g.size1() <<" ");

        
        SXDict nlp = {{"f", obj}, {"x", OPT_variables}, {"g", g}, {"p", P}};
        Dict opts;

//        opts["qpsol"] = "gurobi";
//        opts["hessian_approximation"] = "limited-memory";

//        opts["max_iter"] =10;

//        opts["print_iteration"] = true;
//        opts["warn_initial_bounds"] = true;
//        Dict qopts;
//       qopts["gurobi.Threads"]=4;
//       qopts["verbose"]=false;

////        qopts["sparse"]=true;
////         qopts["printLevel"]="low";
//        qopts["linsol_plugin"]="gurobi";

//        opts["qpsol_options"] = qopts;
//       opts["qpsol_options"] = qopts;





        opts["ipopt.max_iter"] = 2000;
        opts["ipopt.print_level"] = 0;
        opts["print_time"] = false;
        opts["ipopt.acceptable_tol"] = 1e-6;
        opts["ipopt.acceptable_obj_change_tol"] = 1e-4;
        opts["ipopt.print_timing_statistics"] = "no";
        opts["ipopt.print_info_string"] = "no";

        solver = nlpsol("nlpsol", "ipopt", nlp, opts);
    }
  
  ////////////////////////////////////////////////////////////////////////////////////////////////////////
    template <class T, class T2, long unsigned int A, long unsigned int B>
    void estimateMhe(casadi::DM& argx0, const boost::array<T, B>& q4wLoc,
        const boost::array<T2, A>& control2w,const boost::array<T, B>& q4wCov,
        const boost::array<T2, A>& control2wCov, const mhe_estimator::CarParams& carParams,
        const mhe_estimator::MheParams& mheParams,const casadi::Function& solver)  
    {
        unsigned int n_states = 4; unsigned int n_controls = 2;
        std::map<std::string, DM> arg, res;
        arg["lbg"] = SX::zeros(n_states*A); //size = n_state * N_Mhe
        arg["ubg"] = SX::zeros(n_states*A);
        
        std::vector<double> lbx((n_controls*A)+(n_states*B), 0.0);
        std::vector<double> ubx((n_controls*A)+(n_states*B), 0.0);
        
        for(int i = 0; i < (n_states*B); i += n_states) 
        {
            lbx[i] = carParams.lowerTh;  //theta lower bound
            lbx[i+1]   = carParams.lowerX;    //x lower bound
            lbx[i+2] = carParams.lowerY;  //y lower bound
            lbx[i+3] = -carParams.steeringLimit;
        
        
            ubx[i] = carParams.upperTh;   //theta upper bound
            ubx[i+1]   = carParams.upperX;    //x upper bound
            ubx[i+2] = carParams.upperY;  //y upper bound
            ubx[i+3] = carParams.steeringLimit; //beta upper bound
        
        }
        for(int i = ((n_states*B)); i < ((n_controls*A)+(n_states*B)); i += n_controls) 
        {
            lbx[i] = -carParams.SteeringVelLimit;    //v lower bound
            lbx[i+1] = -carParams.LinearVelLimit;  //beta lower bound
        

            ubx[i] = carParams.SteeringVelLimit;    //v upper bound
            ubx[i+1] = carParams.LinearVelLimit;  //beta upper bound        
            
        }
        arg["lbx"] = lbx;
        arg["ubx"] = ubx;
        //--------------ALL OF THE ABOVE IS JUST A PROBLEM SET UP-------------//
        
        
        DM argp = SX::zeros(n_states,B+A+B+A);
        for(int j = 0; j < B; j++) //measured states
        {
            boost::array<double, 4> qloc = q4wLoc[j];
            argp(0,j) = qloc[0];
            argp(1,j) = qloc[1];
            argp(2,j) = qloc[2];
            argp(3,j) = qloc[3];
        }
        for(int j = 0; j < A; j++) //measured controls
        {
            boost::array<double, 2> qctr = control2w[j];
            //argp(0,j + B) = 0.0;   //we do not have measured control dbeta 
            argp(1,B+j) = qctr[1];
        }
        for(int j = 0; j < B; j++) //state cov matrix
        {
            boost::array<double, 4> qCov = q4wCov[j];
            argp(0,B+A+j) = qCov[0];
            argp(1,B+A+j) = qCov[1];
            argp(2,B+A+j) = qCov[2];
            argp(3,B+A+j) = qCov[3];
        }
        for(int j = 0; j < A; j++) //control cov matrix
        {
            boost::array<double, 2> ctrCov = control2wCov[j];
            argp(0,B+A+B+j) = 0.0;   //we do not have measured control dbeta 
            argp(1,B+A+B+j) = ctrCov[1];
        }

        arg["p"] = argp;
        arg["x0"] = argx0; //Comes as function parameter

        res = solver(arg); // solve MHE
        argx0 = res.at("x");
        
    }

    template <class T, class T2, long unsigned int A, long unsigned int B>
    void estimateMheTrailer(casadi::DM& argx0Trailer,const boost::array<T, B>& q5wLocTrailer,
        const boost::array<T2, A>& control2wTrailer,const boost::array<T, B>& q5wCovTrailer,
        const boost::array<T2, A>& control2wCovTrailer,const mhe_estimator::CarParams& carParams,
        const mhe_estimator::MheParams& mheParams,const casadi::Function& solver)  //check the ref and const
    {
        
        unsigned int n_states = 5; unsigned int n_controls = 2;
        std::map<std::string, DM> arg, res;
        arg["lbg"] = SX::zeros(n_states*A); //size = n_state * N_Mhe 
        arg["ubg"] = SX::zeros(n_states*A);
        
        std::vector<double> lbx((n_controls*A)+(n_states*B), 0.0);
        std::vector<double> ubx((n_controls*A)+(n_states*B), 0.0);
        
        for(int i = 0; i < (n_states*B); i += n_states) 
        {
            lbx[i] = -carParams.trailer1Limit; 
            lbx[i+1] = carParams.lowerTh;   //theta lower bound
            lbx[i+2] = carParams.lowerX;    //x lower bound
            lbx[i+3] = carParams.lowerY;    //y lower bound
            lbx[i+4] = -carParams.steeringLimit;
        
            ubx[i] = carParams.trailer1Limit; 
            ubx[i+1] = carParams.upperTh;   //theta upper bound
            ubx[i+2] = carParams.upperX;    //x upper bound
            ubx[i+3] = carParams.upperY;    //y upper bound
            ubx[i+4] = carParams.steeringLimit; 
        }

        ////////////////////////////////////////////

        for(int i = (n_states*B); i < ((n_controls*A)+(n_states*B)); i += n_controls) 
        {
            lbx[i] = -carParams.SteeringVelLimit;    //dbeta lower bound
            lbx[i+1] = -carParams.LinearVelLimit;  //linear.vel lower bound

            ubx[i] = carParams.SteeringVelLimit;  //dbeta upper bound
            ubx[i+1] = carParams.LinearVelLimit;  //linear.vel upper bound  
        }
        arg["lbx"] = lbx;
        arg["ubx"] = ubx;
        //--------------ALL OF THE ABOVE IS JUST A PROBLEM SET UP-------------//

        DM argp = SX::zeros(n_states,B+A+B+A);
        for(int j = 0; j < B; j++) //measured state
        {
            boost::array<double, 5> qloc = q5wLocTrailer[j];
            argp(0,j) = qloc[0]; //beta1
            argp(1,j) = qloc[1]; //theta
            argp(2,j) = qloc[2]; //x
            argp(3,j) = qloc[3]; //y
            argp(4,j) = qloc[4]; //beta0
        }
        for(int j = 0; j < A; j++) //measured control
        {
            boost::array<double, 2> qctr = control2wTrailer[j];
            //argp(0,j+B) = 0.0;   //we do not have measured control dbeta 
            argp(1,j+B) = qctr[1];
        }
        for(int j = 0; j < B; j++) //state cov matrix
        {
            boost::array<double, 5> qCov = q5wCovTrailer[j];
            argp(0,B+A+j) = qCov[0]; //beta1
            argp(1,B+A+j) = qCov[1]; //theta
            argp(2,B+A+j) = qCov[2]; //x
            argp(3,B+A+j) = qCov[3]; //y
            argp(4,B+A+j) = qCov[4]; //beta0
        }
        for(int j = 0; j < A; j++) //control cov matrix
        {
            boost::array<double, 2> ctrCov = control2wCovTrailer[j];
            argp(0,B+A+B+j) = 0.0;   //we do not have measured control dbeta 
            argp(1,B+A+B+j) = ctrCov[1];
        }


        arg["p"] = argp;
        arg["x0"] = argx0Trailer; //Comes as parameter

        try
        {
          res = solver(arg); // solve MHE
          argx0Trailer = res.at("x");
        } catch (casadi::CasadiException& ex)
        {
          ROS_ERROR_STREAM(ex.what());

        }


    
    }
    
    void estimateEst(Vec3& qCarEst,const Vec3& qLoc,const Vec3& qPred,const MheParams& estParams)
    {   
           
        qCarEst[0] = estParams.WeightTh*qPred[0] + (1-estParams.WeightTh)*qLoc[0];
        qCarEst[1] = estParams.WeightPos*qPred[1] + (1-estParams.WeightPos)*qLoc[1];
        qCarEst[2] = estParams.WeightPos*qPred[2] + (1-estParams.WeightPos)*qLoc[2];
        
    }

    void estimateEstTrailer(Vec4& qTrailerEst,const Vec4& qTrailerLoc,const Vec4& qTrailerPred,const MheParams& estParams)
    {   qTrailerEst[0] = estParams.WeightTrailer1*qTrailerPred[0] + (1-estParams.WeightTrailer1)*qTrailerLoc[0];           
        qTrailerEst[1] = estParams.WeightTh*qTrailerPred[1] + (1-estParams.WeightTh)*qTrailerLoc[1];
        qTrailerEst[2] = estParams.WeightPos*qTrailerPred[2] + (1-estParams.WeightPos)*qTrailerLoc[2];
        qTrailerEst[3] = estParams.WeightPos*qTrailerPred[3] + (1-estParams.WeightPos)*qTrailerLoc[3];
    }


}
