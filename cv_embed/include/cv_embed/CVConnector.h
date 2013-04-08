#ifndef CVCONNECTOR_H
#define CVCONNECTOR_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Empty.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <nxt_adventurer/Order.h>
#include <math.h>

#include "CVImageAnalyzer.h"

// classe de test, ne pas s'y fier
class CVConnector
{
public:
    CVConnector();
    virtual ~CVConnector() { ; }

    bool                                init();

private:
    ros::NodeHandle                     m_NodeHandle;
    ros::Publisher                      m_OrderPublisher;
    ros::Subscriber                     m_ReadySubscriber;
    image_transport::ImageTransport     m_ImageTransport;
    image_transport::Subscriber         m_ImageSubscriber;
    nxt_adventurer::Order               m_Order;

    CVImageAnalyzer                     m_test;

    bool                                m_bReady;

    bool                                initSubscriber();
    bool                                initPublisher();

    void                                computeOrder(uint freeW, uint rightW, uint leftW, uint aheadW, uint maxW);
    void                                sendOrder();

    uint                                getWeight(const cv::Rect & rect, const std::vector<cv::Vec4i> & lines, const std::vector<cv::Point2f> & features);
    std::vector<cv::Vec4i>              linesFilter(const std::vector<cv::Vec4i> & lines, const std::vector<cv::Point2f> & features, cv::Mat & output);

    void                                imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void                                readyCallback(const std_msgs::Empty::ConstPtr & msg);
};


#endif // CVCONNECTOR_H
