//
// Created by korisd on 10/1/19.
//

#include "../include/tf_monitor/TfMonitor.h"

TfMonitor::TfMonitor() : _listener( _buffer )
{
    _update_timer                   = _nh.createTimer( ros::Duration( 1.0 ), boost::bind( &TfMonitor::updateTimerCb, this, _1 ));
    _request_stamped_topic_tf_srv   = _nh.advertiseService( "/tf_monitor/RequestStampedTopicTf", &TfMonitor::requestStampedTopicTfSrv, this );
    _request_tf_srv                 = _nh.advertiseService( "/tf_monitor/RequestTf", &TfMonitor::requestTfSrv, this );

}

void TfMonitor::updateTimerCb( const ros::TimerEvent &event )
{
    ros::master::V_TopicInfo topics;
    ros::master::getTopics( topics );

    for( auto &topic : topics )
    {
        if( topic.datatype == TRANSFORM_TYPE )
        {
            if( _broadcasters.find( topic.name ) == _broadcasters.end())
            {
                _broadcasters.emplace( topic.name, topic.name );
            }
        }
    }
    ROS_INFO( "%lu Broadcasters Running.", _broadcasters.size());
}

bool TfMonitor::requestStampedTopicTfSrv( tf_monitor_msgs::GetStampedTopicTfRequest &req, tf_monitor_msgs::GetStampedTopicTfResponse &res )
{
    auto key = std::make_pair( req.topic_name, req.out_frame );

    if( _transforms.find( key ) == _transforms.end())
    {
        _transforms.insert(
                std::make_pair(
                        key,
                        std::make_unique<TfFastListener<geometry_msgs::TransformStamped>>(
                                req.topic_name,
                                req.out_frame,
                                req.out_topic_name_recommendation,
                                _buffer ) ) );
    }
    res.out_topic_name = _transforms[key]->GetOutTopicName();
    return true;
}

bool TfMonitor::requestTfSrv( tf_monitor_msgs::GetTfRequest &req, tf_monitor_msgs::GetTfResponse &res )
{
    try
    {
        res.tf = _buffer.lookupTransform( req.out_frame, req.in_frame, ros::Time(0), ros::Duration(req.wait) );
    }
    catch( tf2::TransformException &ex )
    {
        ROS_WARN( "%s", ex.what() );
        return false;
    }
    return true;
}