//
// Created by korisd on 10/1/19.
//

#ifndef TF_MONITOR_TFMONITOR_H
#define TF_MONITOR_TFMONITOR_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf_monitor_msgs/GetStampedTopicTf.h>
#include <tf_monitor_msgs/GetTf.h>

#include <map>

#include "TfBroadcaster.h"
#include "TfFastListener.h"

static const std::string TRANSFORM_TYPE = "geometry_msgs/TransformStamped";

typedef std::string TopicName;
typedef std::string OutFrame;

/*
 * TODO topics that don't receive an update within a given parameter should shut themselves down and
 *  signal for deletion and removal from the map.
 */

class TfMonitor
{
public:
    TfMonitor();

private:
    void updateTimerCb( const ros::TimerEvent &event );

    bool requestStampedTopicTfSrv( tf_monitor_msgs::GetStampedTopicTfRequest &req, tf_monitor_msgs::GetStampedTopicTfResponse &res );
    bool requestTfSrv( tf_monitor_msgs::GetTfRequest &req, tf_monitor_msgs::GetTfResponse &res );
    // ROS Basics
    ros::NodeHandle _nh;
    ros::Timer _update_timer;

    // ROS TF2
    tf2_ros::Buffer _buffer;
    tf2_ros::TransformListener _listener;

    // ROS Services
    ros::ServiceServer _request_stamped_topic_tf_srv;
    ros::ServiceServer _request_tf_srv;

    // Map of TF Broadcasters TODO consider changing to a vector, map might actually be slower in this case
    std::map<std::string, TfBroadcaster> _broadcasters;

    // Map of Requests index is a pair of topic name, out frame
    std::map<std::pair<TopicName, OutFrame>, std::unique_ptr<TfFastListenerBase>> _transforms;


};

#endif //TF_MONITOR_TFMONITOR_H
