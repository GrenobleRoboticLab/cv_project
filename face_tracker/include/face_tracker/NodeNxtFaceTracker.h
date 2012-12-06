#ifndef _NODEFACETRACKER_H__
#define _NODEFACETRACKER_H__

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <cv.h>
#include <math.h>
#include "LunarNXT/Order.h"

#include "face_tracker/FaceTracker.h"

namespace LRG {

/* ---------------------------------------------------------------------------------------- */

class NodeNxtFaceTracker
{
public:
    NodeNxtFaceTracker();
    virtual ~NodeNxtFaceTracker() { ; }

    Status                             Init();

    void                               ImageCallback(const sensor_msgs::ImageConstPtr& msg);
    
private:
    ros::NodeHandle                    m_NodeHandle;
    image_transport::ImageTransport    m_ImageTransport;
    image_transport::Subscriber        m_ImageSubscriber;
    image_transport::Publisher         m_ImagePublisher;
    ros::Publisher                     m_OrderPublisher;
    FaceTracker                        m_FaceTracker;

    Status                             InitSubscriber();
    Status                             InitPublisher();
    Status                             InitFaceTracker();
};

/* ---------------------------------------------------------------------------------------- */

} // namespace LRG

#endif // _NODEFACETRACKER_H__
