//
// Created by korisd on 10/1/19.
//

#ifndef TF_MONITOR_TFFASTLISTENER_H
#define TF_MONITOR_TFFASTLISTENER_H

#include <ros/ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/transform_datatypes.h>
#include <string>


// TODO this should subscribe to a specific topic and transform it when it can
//  needs a better name, needs better creation enforcement?

class TfFastListenerBase
{
public:
    TfFastListenerBase( const std::string& topic_name,
                        const std::string& out_frame,
                        const std::string& out_topic_name_recommendation,
                        tf2_ros::Buffer &buffer )
            : _topic_name( topic_name ),
              _out_frame( out_frame ),
              _buffer( buffer )
    {
        if( out_topic_name_recommendation.empty() )
            _out_topic_name = topic_name + "_in_" + out_frame;
        else
            _out_topic_name = out_topic_name_recommendation;
    }

    std::string GetTopic()
    { return _topic_name; }

    std::string GetOutFrame()
    { return _out_frame; }

    std::string GetOutTopicName()
    { return _out_topic_name; }

protected:
    std::string _topic_name;
    std::string _out_frame;
    std::string _out_topic_name;

    //ROS
    ros::NodeHandle _nh;
    tf2_ros::Buffer &_buffer;
    ros::Publisher _pub;
};


template<typename M>
class TfFastListener : public TfFastListenerBase
{
public:
    TfFastListener( const std::string &topic_name, const std::string &out_frame, const std::string &out_topic_name_recommendation,
                    tf2_ros::Buffer &buffer )
            : TfFastListenerBase( topic_name, out_frame, out_topic_name_recommendation, buffer ),
              _filter( _sub, _buffer, out_frame, 10, 0 )
    {
        _pub            = _nh.advertise<M>( _out_topic_name, 1, true );
        _sub.subscribe( _nh, _topic_name, 10 );
        _filter.registerCallback( boost::bind( &TfFastListener::tfCb, this, _1 ));
    }

private:
    void tfCb( const typename M::ConstPtr &msg )
    {
        M out_msg;
        try
        {
            _buffer.transform( *msg, out_msg, _out_frame );
            _pub.publish( out_msg );
        }
        catch( tf2::TransformException &ex )
        {
            ROS_WARN( "Failure %s\n", ex.what()); //Print exception which was caught
        }
    }

    message_filters::Subscriber<M> _sub;
    tf2_ros::MessageFilter<M> _filter;
};


#endif //TF_MONITOR_TFFASTLISTENER_H
