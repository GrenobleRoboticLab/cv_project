#include "cv_embed/CVCarDriver.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "RoadTracker");
    CVCarDriver c;
    ros::spin();
    return 0;
}
