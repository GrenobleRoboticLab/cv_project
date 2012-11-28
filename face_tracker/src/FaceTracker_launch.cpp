#include <ros/ros.h>

#include "face_tracker/NodeFaceTracker.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "KinectFaceTracker");
    
    LRG::NodeFaceTracker node;
    node.Init();
    
    ros::spin();
    return 0;
}
