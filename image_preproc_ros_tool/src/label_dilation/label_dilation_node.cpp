#include <image_preproc/label_dilation.h>

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "label_dilation_node");

    image_preproc_ros_tool::LabelDilation label_dilation(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}
