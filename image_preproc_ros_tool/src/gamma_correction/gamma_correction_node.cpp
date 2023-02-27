#include "gamma_correction_module.h"

int main (int argc, char *argv[]) {
    // ROS setup and parameter handling
    ros::init(argc, argv, "gamma_correction_module", ros::init_options::AnonymousName);

    ros::NodeHandle publicNH;
    ros::NodeHandle privateNH("~");


    GammaCorrectorModule corrector(publicNH, privateNH, ros::this_node::getName());
    ros::spin();
    exit(EXIT_SUCCESS);
}
