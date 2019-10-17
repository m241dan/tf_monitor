//
// Created by korisd on 10/1/19.
//

#include "../include/tf_monitor/TfBroadcaster.h"

TfBroadcaster::TfBroadcaster( const std::string& topic_name ) : _name( topic_name )
{
    _tf_sub = _nh.subscribe( topic_name, 1, &TfBroadcaster::tfSubCb, this );
}

std::string TfBroadcaster::GetName()
{
    return _name;
}

void TfBroadcaster::tfSubCb( const geometry_msgs::TransformStamped::ConstPtr &msg )
{
    if( !msg->child_frame_id.empty() && !msg->header.frame_id.empty() )
        _broadcaster.sendTransform( *msg );
}
