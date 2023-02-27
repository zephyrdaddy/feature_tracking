#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "gamma_correction_module.h"

namespace image_preproc_ros_tool {

class GammaCorrectionNodelet : public nodelet::Nodelet {

    virtual void onInit();
    std::unique_ptr<GammaCorrectorModule> m_;
};

void GammaCorrectionNodelet::onInit() {
    m_ = std::make_unique<GammaCorrectorModule>(this->getNodeHandle(), this->getPrivateNodeHandle(), this->getName());
}
}

PLUGINLIB_EXPORT_CLASS(image_preproc_ros_tool::GammaCorrectionNodelet, nodelet::Nodelet);
