#include "cv_embed/CVImageAnalyzer.h"

CVImageAnalyzer::CVImageAnalyzer(double dThresh, double dMaxVal)
{
    m_dThresh   = dThresh;
    m_dMaxVal   = dMaxVal;
}

void    CVImageAnalyzer::setImageOrg(const cv::Mat & imageOrg)
{
    m_ImageOrg = imageOrg;
    cv::cvtColor(m_ImageOrg, m_ImageGray, CV_BGR2GRAY);
}

void    CVImageAnalyzer::setThresh(double dThresh)
{
    m_dThresh = dThresh;
}

void    CVImageAnalyzer::setMaxVal(double dMaxVal)
{
    m_dMaxVal = dMaxVal;
}

void    CVImageAnalyzer::goodFeaturesToTrack(std::vector<cv::Point2f> & vFeatures, int nCountMax, double dQualityLevel, double dMinDist)
{
    cv::goodFeaturesToTrack(m_ImageGray, vFeatures, nCountMax, dQualityLevel, dMinDist);
}

void    CVImageAnalyzer::findContours(std::vector<std::vector<cv::Point> > & contours, cv::Point offsetPoint)
{
    cv::Mat                 tmpMat;
    std::vector<cv::Vec4i>  hierarchy;

    cv::threshold(m_ImageGray, tmpMat, m_dThresh, m_dMaxVal, CV_THRESH_BINARY);
    cv::findContours(tmpMat, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, offsetPoint);
}

void    CVImageAnalyzer::houghLines(std::vector<cv::Vec4i> & vOutLines)
{
    cv::Mat tmpMat;

    cv::Canny(m_ImageGray, tmpMat, m_dThresh, m_dMaxVal);
    cv::HoughLinesP(tmpMat, vOutLines, 1, CV_PI/180, 80, 30, 10);
}

void    CVImageAnalyzer::extractObject(const std::vector<cv::Rect> & rects, std::vector<cv::Mat> & images)
{
    images.clear();
    for (uint i = 0; i < rects.size(); i++)
        images.push_back(cv::Mat(m_ImageOrg, rects[i]));
}

void    CVImageAnalyzer::hideGround(const  std::vector<cv::Rect> & rects, cv::Mat & image)
{
    image = cv::Mat(m_ImageOrg.size(), m_ImageOrg.type());

    for (uint i = 0; i < rects.size(); i++)
        cv::rectangle(image, rects[i], cv::Scalar(255, 255, 255), -1);
}

double  CVImageAnalyzer::getImgWidth()  { return m_ImageOrg.cols; }
double  CVImageAnalyzer::getImgHeight() { return m_ImageOrg.rows; }

cv::Point CVImageAnalyzer::getEdge(VEdge vertical, HEdge horizontal)
{
    cv::Point ret;

    if (vertical == VE_TOP)
        ret.y = 0;
    else if (vertical == VE_MIDDLE)
        ret.y = m_ImageOrg.rows / 2;
    else if (vertical == VE_BOT)
        ret.y = m_ImageOrg.rows;

    if (horizontal == HE_LEFT)
        ret.x = 0;
    else if (horizontal == HE_CENTER)
        ret.x = m_ImageOrg.cols / 2;
    else if (horizontal == HE_RIGHT)
        ret.x = m_ImageOrg.cols;

    return ret;
}
