#include "gamma_correction.h"

GammaCorrectorModule::GammaCorrectorModule(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle, const std::string name) {
  name_ = name;
  image_preproc::GammaCorrectorParams gamma_corrector_params;

  privateNodeHandle.param("gamma", gamma_corrector_params.gamma, 1.0);
  privateNodeHandle.param("auto_gamma", gamma_corrector_params.auto_gamma, false);
  gamma_corrector_.setParams(gamma_corrector_params);
  
  std::string srcTopic = nodeHandle.resolveName("src/image");
  std::string tgtTopic = nodeHandle.resolveName("tgt/image");
  srv_.reset(new ReconfigureServer(privateNodeHandle));
  dynamic_reconfigure::Server<image_preproc_ros_tool::GammaCorrectionConfig>::CallbackType f(
          boost::bind(&GammaCorrectorModule::reconfigureRequest, this, _1, _2));
  srv_->setCallback(f);

  pub_ = nodeHandle.advertise<sensor_msgs::Image>(tgtTopic, 10);
  sub_ = nodeHandle.subscribe<sensor_msgs::Image>(srcTopic, 10,
                                                  &GammaCorrectorModule::handleImage, this,
                                                  ros::TransportHints().tcpNoDelay());
}

void GammaCorrectorModule::reconfigureRequest(const image_preproc_ros_tool::GammaCorrectionConfig& cfg, uint32_t) {
  image_preproc::GammaCorrectorParams gamma_corrector_params;
  gamma_corrector_params.gamma = cfg.gamma;
  gamma_corrector_params.auto_gamma = cfg.auto_gamma;
  gamma_corrector_.setParams(gamma_corrector_params);
}

void GammaCorrectorModule::handleImage (const sensor_msgs::Image::ConstPtr& imgMsg) {
  ROS_INFO_STREAM(name_ << " Gamma correction Entered callback!");
    try {
        cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(imgMsg);
        cv_bridge::CvImage outImage;
        outImage.header = imgMsg->header;
        outImage.encoding = imgMsg->encoding;

        gamma_corrector_.processImage(cvPtr->image, outImage.image);

        pub_.publish(outImage);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR_STREAM("Failed to convert ROS image into CV image: " << e.what());
    }
  ROS_INFO_STREAM(name_ << " gamma correction leaving callback!");

}

