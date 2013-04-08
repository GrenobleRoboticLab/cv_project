#include "cv_embed/CVConnector.h"

#include "cv_embed/CVMathHelper.h"

CVConnector::CVConnector()
    :   m_ImageTransport(m_NodeHandle),
        m_test(100, 200)
{
    init();
}

void CVConnector::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImage                      cvBImage;
    std::vector<std::vector<cv::Point> >    contours;
    std::vector<cv::Rect>                   rects;

    try
    {
        cv_bridge::CvImagePtr    cvBImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        cvBImage.image    = cvBImagePtr->image;
        cvBImage.encoding = cvBImagePtr->encoding;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    m_test.setImageOrg(cvBImage.image);
    m_test.findContours(contours);

    for(uint i = 0; i< contours.size(); i++ )
    {
        cv::Rect    boundedRect;
        computeArea(contours[i], boundedRect);

        if(boundedRect.area() > 1000 && boundedRect.area() < (600*400))
        {
            rects.push_back(boundedRect);
            cv::rectangle(cvBImage.image, boundedRect, cv::Scalar(255, 0, 0), 2);
        }
    }

    cv::namedWindow("Detected Lines", 1);
    cv::imshow("Detected Lines", cvBImage.image);
    cv::waitKey(5);

    /*
    cv_bridge::CvImage          cvBImage;
    cv::Mat                     tmpMat;
    cv::Mat                     tmpMat2;
    cv::Rect                    freeSpace,
                                aheadSpace,
                                rightSpace,
                                leftSpace;

    std::vector<cv::Vec4i>      vLines;
    std::vector<cv::Point2f>    vFeatures;
    cv::RNG rng(12345);

    try
    {
        cv_bridge::CvImagePtr    cvBImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        cvBImage.image    = cvBImagePtr->image;
        cvBImage.encoding = cvBImagePtr->encoding;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    freeSpace.x = 0;
    freeSpace.width = cvBImage.image.cols;
    freeSpace.y = cvBImage.image.rows / 1.5;
    freeSpace.height = cvBImage.image.rows - freeSpace.y;

    leftSpace.x = 0;
    leftSpace.width = aheadSpace.x = aheadSpace.width = rightSpace.width = cvBImage.image.cols / 3;
    leftSpace.y = aheadSpace.y = rightSpace.y = 0;
    leftSpace.height = aheadSpace.height = rightSpace.height = freeSpace.y;

    rightSpace.x = 2 * aheadSpace.x;

    cv::cvtColor(cvBImage.image, cvBImage.image, CV_RGB2GRAY);
    cv::goodFeaturesToTrack(cvBImage.image, vFeatures, 1000, 0.1, 5);

    cv::Canny(cvBImage.image, tmpMat2, 100, 255, 3);


    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    // cv::Canny(cvBImage.image, tmpMat, 100, 100*2, 3);

    cv::threshold(cvBImage.image,tmpMat,100,255,CV_THRESH_BINARY);
    cv::findContours(tmpMat, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    cv::cvtColor(cvBImage.image, cvBImage.image, CV_GRAY2BGR);

    for(uint i = 0; i< contours.size(); i++ )
    {
        cv::Rect    boundedRect;
        double      dArea   = std::abs<double>(computeArea(contours[i], &boundedRect));

        if (boundedRect.width > 20 && boundedRect.height > 20 && dArea > 500)
        {
            cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            cv::drawContours(cvBImage.image, contours, i, color, 3, 8, hierarchy, 0, cv::Point());
            cv::rectangle(cvBImage.image, boundedRect, color, 2);
            std::cout << boundedRect.x << ":" << boundedRect.y << "\t" << boundedRect.width << ":" << boundedRect.height << std::endl;
        }
    }

    cv::HoughLinesP(tmpMat2, vLines, 1, CV_PI/180, 80, 30, 10);

    for (size_t i = 0; i < vLines.size(); i++)
        cv::line(cvBImage.image, cv::Point(vLines[i][0], vLines[i][1]), cv::Point(vLines[i][2], vLines[i][3]), cv::Scalar(0, 0, 255), 3, 8);

    for (size_t i = 0; i < vFeatures.size(); i++)
        cv::circle(cvBImage.image, vFeatures.at(i), 2, cv::Scalar(0, 255, 0), 3);

    uint    weightM = ((2 * vLines.size()) + vFeatures.size()),
            weightF = getWeight(freeSpace, vLines, vFeatures),
            weightA = getWeight(aheadSpace, vLines, vFeatures),
            weightL = getWeight(leftSpace, vLines, vFeatures),
            weightR = getWeight(rightSpace, vLines, vFeatures);


    cv::rectangle(cvBImage.image, freeSpace, cv::Scalar(0, 0, 0), 3);
    cv::rectangle(cvBImage.image, rightSpace, cv::Scalar(0, 0, 0), 3);
    cv::rectangle(cvBImage.image, aheadSpace, cv::Scalar(0, 0, 0), 3);
    cv::rectangle(cvBImage.image, leftSpace, cv::Scalar(0, 0, 0), 3);

    // vLines = linesFilter(vLines, vFeatures, cvBImage.image);
    computeOrder(weightF, weightR, weightL, weightA, weightM);
    if (m_bReady)
        sendOrder();

    cv::namedWindow("Detected Lines", 1);
    cv::imshow("Detected Lines", cvBImage.image);
    cv::namedWindow("thres", 1);
    cv::imshow("thres", tmpMat);
    cv::waitKey(5);*/
}

void CVConnector::readyCallback(const std_msgs::Empty::ConstPtr &msg)
{
    m_bReady = true;
}

bool CVConnector::init()
{
    m_bReady = true;
    return (initSubscriber() && initPublisher());
}

bool CVConnector::initSubscriber()
{
    bool        bRet = false;
    std::string sSubTopicName;

    m_ReadySubscriber = m_NodeHandle.subscribe("ard_done", 1, &CVConnector::readyCallback, this);

    //if (m_NodeHandle.getParam("/RoadTracker/sub_topic_name", sSubTopicName))
    {
        m_ImageSubscriber = m_ImageTransport.subscribe("/usb_cam/image_raw", 1, &CVConnector::imageCallback, this);
        bRet = true;
    }
    //else
      //  ROS_ERROR("Error getting 'sub_topic_name' param.");

    return bRet;
}

bool CVConnector::initPublisher()
{
    bool        bRet = false;
    std::string sPubTopicName;

//    if (m_NodeHandle.getParam("/RoadTracker/pub_topic_name", sPubTopicName))
    {
        m_OrderPublisher = m_NodeHandle.advertise<nxt_adventurer::Order>("car_order", 1);
        bRet = true;
    }
    //else
      //  ROS_ERROR("Error getting 'pub_topic_name' param.");

    m_Order.order = 3;
    m_Order.effort = 30;
    m_OrderPublisher.publish(m_Order);
    //sendOrder();

    return bRet;
}

void CVConnector::computeOrder(uint freeW, uint rightW, uint leftW, uint aheadW, uint maxW)
{
    m_Order.effort   = 1;
    if (freeW < (maxW/4))
    {
        std::cout << "marche avant" << std::endl;
        if (aheadW > maxW / 4)
        {
            m_Order.order        = 1;
            m_Order.direction    = (rightW > leftW);
        }
        else
        {
            m_Order.order        = 0;
            m_Order.direction    = true;
        }
    }
    else
    {
        std::cout << "marche ariere" << std::endl;
        m_Order.order = 2;
        m_Order.direction = !(rightW > leftW);
    }
}

void CVConnector::sendOrder()
{
    m_OrderPublisher.publish(m_Order);
    m_bReady = false;
}

unsigned int CVConnector::getWeight(const cv::Rect &rect, const std::vector<cv::Vec4i> &lines, const std::vector<cv::Point2f> &features)
{
    unsigned int nRet = 0;

    for (size_t i = 0; i < features.size(); i++)
    {
        if (features[i].x > rect.x && features[i].x < (rect.x + rect.width)
         && features[i].y > rect.y && features[i].y < (rect.y + rect.height))
            nRet++;
    }

    for (size_t i = 0; i < lines.size(); i++)
    {
        if ((lines[i][0] > rect.x && lines[i][0] < (rect.x + rect.width))
         || (lines[i][2] > rect.y && lines[i][2] < (rect.y + rect.height)))
            nRet++;
    }

    return nRet;
}

std::vector<cv::Vec4i> CVConnector::linesFilter(const std::vector<cv::Vec4i> &lines, const std::vector<cv::Point2f> &features, cv::Mat & output)
{
    std::vector<cv::Vec4i>  vLines;
    for (size_t i = 0; i < features.size(); i++)
    {
        for (size_t j = 0; j < lines.size(); j++)
        {
            if (DPoin2tLine(features.at(i), lines.at(j)) < 5.0)
                vLines.push_back(lines.at(j));
        }
    }

    for (size_t k = 0; k < vLines.size(); k++)
        cv::line(output, cv::Point(vLines[k][0], vLines[k][1]), cv::Point(vLines[k][2], vLines[k][3]), cv::Scalar(0,0,255), 3, 8);

    return vLines;
}
