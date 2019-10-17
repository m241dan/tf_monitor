//
// Created by korisd on 10/1/19.
//

#ifndef TF_MONITOR_TFBROADCASTER_H
#define TF_MONITOR_TFBROADCASTER_H
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

class TfBroadcaster
{
public:
    TfBroadcaster( const std::string& topic_name );
    std::string GetName();
private:
    // Functions
    void tfSubCb( const geometry_msgs::TransformStamped::ConstPtr& msg );

    // Variables
    const std::string               _name;
    ros::NodeHandle                 _nh;
    ros::Subscriber                 _tf_sub;
    tf2_ros::TransformBroadcaster   _broadcaster;
};


#endif //TF_MONITOR_TFBROADCASTER_H
