#include "face_tracker/NodeFaceTracker.h"

using namespace LRG;
namespace enc = sensor_msgs::image_encodings;

/* ---------------------------------------------------------------------------------------- */

NodeFaceTracker::NodeFaceTracker()
    : m_ImageTransport(m_NodeHandle)
{
    ;
}

Status NodeFaceTracker::Init()
{
    Status         ret = E_FAIL;

    if (
         ISOK(InitSubscriber())    &&
         ISOK(InitPublisher())     &&
         ISOK(InitFaceTracker())
       )
         ret = S_OK;

    return ret;
}

void NodeFaceTracker::ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImage       cvBImage; 
    
    try
    {
        cv_bridge::CvImagePtr    cvBImagePtr = cv_bridge::toCvCopy(msg, enc::BGR8);
        
        cvBImage.image    = cvBImagePtr->image;
        cvBImage.encoding = cvBImagePtr->encoding;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }    
    
    std::vector<Face>    vFaces;
    
    if (ISOK(m_FaceTracker.GetLoadStatus()))
        m_FaceTracker.Track(cvBImage.image, vFaces);
    
    for (unsigned int i = 0; i < vFaces.size(); i++)
    {
        vFaces[i].DrawOn(cvBImage.image, cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0));
    }
    
    m_ImagePublisher.publish(cvBImage.toImageMsg());
}

Status NodeFaceTracker::InitSubscriber()
{
    Status         ret = E_FAIL;
    std::string    sSubTopicName;

    if (m_NodeHandle.getParam("/FaceTracker/sub_topic_name", sSubTopicName))
    {
        m_ImageSubscriber = m_ImageTransport.subscribe(sSubTopicName, 1, &NodeFaceTracker::ImageCallback, this);
        ret = S_OK;
    }
    else
        ROS_ERROR("Error getting 'sub_topic_name' param.");

    return ret;
}

Status NodeFaceTracker::InitPublisher()
{
    Status         ret = E_FAIL;
    std::string    sPubTopicName;

    if (m_NodeHandle.getParam("/FaceTracker/pub_topic_name", sPubTopicName))
    {
        m_ImagePublisher = m_ImageTransport.advertise(sPubTopicName, 1);
        ret = S_OK;
    }
    else
        ROS_ERROR("Error getting 'sub_topic_name' param.");

    return ret;
}

Status NodeFaceTracker::InitFaceTracker()
{
    Status         ret = E_FAIL;
    std::string    sFaceCascadeName,
                   sEyesCascadeName;

    if (m_NodeHandle.getParam("/FaceTracker/face_cascade_name", sFaceCascadeName) && m_NodeHandle.getParam("/FaceTracker/eyes_cascade_name", sEyesCascadeName))
    {
        m_FaceTracker = FaceTracker(sFaceCascadeName, sEyesCascadeName);
        ret = m_FaceTracker.Load();
    }
    else
        ROS_ERROR("Error getting cascade file name param.");

    return ret;
}


/* ---------------------------------------------------------------------------------------- */
