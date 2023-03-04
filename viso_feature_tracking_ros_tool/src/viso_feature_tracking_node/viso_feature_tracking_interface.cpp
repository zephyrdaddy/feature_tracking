#include "viso_feature_tracking_interface.h"
#include <chrono>

#include <cv_bridge/cv_bridge.h>

#include <feature_tracking_core/tracker_libviso.h>
#include <matches_msg_ros/MatchesMsg.h>
#include <rosinterface_handler/utilities.hpp>
//#include <eigen_conversions/eigen_msg.h>
#include <opencv2/opencv.hpp>

namespace viso_feature_tracking_ros_tool {

namespace cl = std::chrono;

using MatchesMsg = matches_msg_ros::MatchesMsg;

VisoFeatureTrackingInterface::VisoFeatureTrackingInterface(ros::NodeHandle node_handle,
                                                           ros::NodeHandle private_node_handle) {
    rosinterface_handler::setLoggerLevel(private_node_handle);

    // read the configuration
    std::string configFileName;
    if (!private_node_handle.getParam("config", configFileName)) {
        throw std::runtime_error("No configuration specified");
    }
    config = viso_feature_tracking::VisoFeatureTrackingParameters(configFileName).get_config();
    viso_params = viso_feature_tracking::VisoFeatureTrackingParameters(configFileName).get_matcher_parameters();
    ROS_DEBUG_STREAM("tracklength=" << viso_params.maxTracklength);
    ROS_DEBUG_STREAM("refinement=" << viso_params.refinement);
    
    // instantiate tracker with config
    tracker_module_.reset(new viso_feature_tracking::VisoFeatureTrackerModule(config, viso_params));
    // tracker = std::make_shared<feature_tracking::TrackerLibViso>(viso_params);

    /*
     * Publisher
     */
    publisher_matches_ =
        node_handle.advertise<matches_msg_ros::MatchesMsg>(config.matches_msg_name, config.matches_msg_queue_size);

    /*
     * subscriber setup
     */
    subscriber_image_ = node_handle.subscribe(
        config.image_msg_name, config.image_msg_queue_size, &VisoFeatureTrackingInterface::process, this);

    rosinterface_handler::showNodeInfo();
}

void VisoFeatureTrackingInterface::process(const sensor_msgs::ImageConstPtr& input) {
    auto start = cl::high_resolution_clock::now();
    // execute tracker and write matches
    cv_bridge::CvImagePtr cv_bridge_ptr = cv_bridge::toCvCopy(input);

    ROS_DEBUG_STREAM("viso_feature_tracking: image_size=" << cv_bridge_ptr->image.rows << " x "
                                                          << cv_bridge_ptr->image.cols
                                                          << "\nscaling factor="
                                                          << config.scale_factor);
    tracker_module_->process(cv_bridge_ptr->image, input->header.stamp);

    auto end = cl::high_resolution_clock::now();
    int64_t duration = cl::duration_cast<cl::milliseconds>(end - start).count();
    ROS_INFO_STREAM("Duration feature matching and tracking: " << duration << " ms");
    // publish
    publish();

    tracker_module_->reduce();

}

void VisoFeatureTrackingInterface::publish() {
    MatchesMsg out_msg = tracker_module_->get_matches_msg();

    publisher_matches_.publish(out_msg);
}

} // end of namespace
