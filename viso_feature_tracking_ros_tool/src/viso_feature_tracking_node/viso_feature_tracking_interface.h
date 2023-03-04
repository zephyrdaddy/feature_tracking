/*
 * Copyright 2015. All rights reserved.
 * Institute of Measurement and Control Systems
 * Karlsruhe Institute of Technology, Germany
 *
 * authors:
 *  Johannes Graeter (johannes.graeter@kit.edu)
 *  and others
 */

#pragma once

#include <ros/ros.h>

#include "viso_feature_tracking/viso_feature_tracking_parameters.h"

#include "viso_feature_tracking/viso_feature_tracker_module.h"

#include <sensor_msgs/Image.h>

namespace viso_feature_tracking_ros_tool {
class VisoFeatureTrackingInterface {

public:
    ///@brief ctor
    VisoFeatureTrackingInterface(ros::NodeHandle node_handle, ros::NodeHandle private_handle);

private: // attributes
    ///@brief ros stuff
    ros::Publisher publisher_matches_;
    ros::Subscriber subscriber_image_;

    ///@brief params for setting up the node
    viso_feature_tracking::Configuration config;
    viso_feature_tracking::Parameters viso_params;

    std::unique_ptr<viso_feature_tracking::VisoFeatureTrackerModule> tracker_module_;

private: // methods
    ///@brief process the input from ros, execute sampler, publish it as odometry
    void process(const sensor_msgs::ImageConstPtr& input);

    ///@brief publish feature matching as rostopic
    void publish();

};
}
