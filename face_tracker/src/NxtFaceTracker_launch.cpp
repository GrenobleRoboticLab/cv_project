#include <ros/ros.h>

#include "face_tracker/NodeNxtFaceTracker.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "KinectFaceTracker");
    
    LRG::NodeNxtFaceTracker node;
    node.Init();
    
    ros::spin();
    return 0;
}
