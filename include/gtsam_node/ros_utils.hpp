#pragma once

#include "ros/ros.h"
#include "tf/tf.h"
#include <dynamic_reconfigure/server.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>

namespace ast { namespace ros
{
  inline ::ros::Rate operator"" _Hz(long double hz)
  {
    return ::ros::Rate(hz);
  }
  inline  ::ros::Duration operator"" _s(long double sec)
  {
    return ::ros::Duration(sec);
  }

  template <typename T> class TopicIn;
  template <typename T> class TopicOut;

  class NodeHandle : public ::ros::NodeHandle
  {
  public:
    using RosNodeHandle = ::ros::NodeHandle;
    using VoidConstPtr = ::ros::VoidConstPtr;
    using TransportHints = ::ros::TransportHints;
    using Subscriber = ::ros::Subscriber;
    using ServiceServer = ::ros::ServiceServer;

    template<class M>
    Subscriber subscribe(const std::string& topic, uint32_t queue_size,
                         const std::function<void(const M&)>& callback,
                         const VoidConstPtr& tracked_object = VoidConstPtr(),
                         const TransportHints& transport_hints = TransportHints())
    {
      return RosNodeHandle::subscribe<M>(topic, queue_size,
                                         static_cast<boost::function<void(const M&)>>(callback),
                                         tracked_object, transport_hints);
    }

    template<class S>
    ServiceServer advertiseService(const std::string& service,
                                   const std::function<bool(typename S::Request&, typename S::Response&)> callback,
                                   const VoidConstPtr& tracked_object = VoidConstPtr())
    {
      return RosNodeHandle::advertiseService<typename S::Request, typename S::Response>(
            service,
            static_cast<boost::function<bool(typename S::Request&, typename S::Response&)>>(callback),
            tracked_object);
    }

    template <typename T>
    TopicIn<T> Input(std::string topicName)
    {
      return TopicIn<T>(*this, topicName);
    }

    template <typename T>
    TopicOut<T> Output(std::string topicName)
    {
      return TopicOut<T>(*this, topicName);
    }
  };

  template<typename T>
  class TopicIn
  {
    ::ros::Subscriber sub;
    T msg;
    bool isNew = false;
  public:
    TopicIn(NodeHandle& nh, std::string topicName) : sub(nh.subscribe<T>(topicName, 2, [&](const auto& message)
    {
      msg = message;
      isNew = true;
    }))
    {
    }

    const T& operator()()
    {
      return msg;
    }

    bool IsNew() const { return isNew; }

    bool ClearNew()
    {
      bool n = isNew;
      isNew = false;
      return n;
    }

  };


  template<typename T>
  class TopicOut
  {
  ::ros::Publisher pub;
  public:
    TopicOut(NodeHandle& nh, std::string topicName) : pub(nh.advertise<T>(topicName, 10))
    {
    }

    T operator()(T msg)
    {
      try
      {
        pub.publish(msg);
      }
      catch(const ::ros::serialization::StreamOverrunException& ex)
      {
        ROS_WARN_STREAM(ex.what());
      }
      return msg;
    }

    void operator()(const boost::shared_ptr<T>& msg)
    {
        try
        {
          pub.publish(msg);
        }
        catch(const ::ros::serialization::StreamOverrunException& ex)
        {
          ROS_WARN_STREAM(ex.what());
        }
    }
  };

  inline geometry_msgs::Pose2D poseTo2D(geometry_msgs::PoseStamped pose)
  {
    geometry_msgs::Pose2D qFinal;
    qFinal.x = pose.pose.position.x;
    qFinal.y = pose.pose.position.y;
    tf::Pose tfPose;
    tf::poseMsgToTF(pose.pose, tfPose);
    qFinal.theta = tf::getYaw(tfPose.getRotation());
    return qFinal;
  }

  template <typename F>
  inline void timedLoop(double freq, F f)
  {
    ::ros::Rate loopRate(freq);
    while(::ros::ok())
    {
      f();
      loopRate.sleep();
    }
  }

  template <typename F>
  inline void timedMainLoop(double freq, F f)
  {
    ::ros::Rate loopRate(freq);
    while(::ros::ok())
    {
      f();
      ::ros::spinOnce();
      loopRate.sleep();
    }
  }

} }
