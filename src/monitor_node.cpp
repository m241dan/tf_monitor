//
// Created by korisd on 10/1/19.
//

#include <ros/ros.h>
#include "../include/tf_monitor/TfMonitor.h"
#include "../include/tf_monitor/TfFastListener.h"

int main( int argc, char *argv[] )
{
    ros::init( argc, argv, "TF_Monitor" );
    TfMonitor monitor;
    ros::spin();
    return 0;
}