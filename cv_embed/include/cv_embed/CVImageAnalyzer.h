#ifndef CVIMAGEANALYZER_H
#define CVIMAGEANALYZER_H

#include <opencv2/opencv.hpp>

enum VEdge {
    VE_TOP,
    VE_BOT,
    VE_MIDDLE
};

enum HEdge {
    HE_LEFT,
    HE_RIGHT,
    HE_CENTER
};

class CVImageAnalyzer
{
public:
    CVImageAnalyzer(double dThresh, double dMaxVal);
    virtual ~CVImageAnalyzer() { ; }

    void        setImageOrg(const cv::Mat & imageOrg);
    void        setThresh(double dThresh);
    void        setMaxVal(double dMaxVal);

    void        goodFeaturesToTrack(std::vector<cv::Point2f> & vFeatures, int nCountMax, double dQualityLevel, double dMinDist);
    void        findContours(std::vector<std::vector<cv::Point> > & contours, cv::Point offsetPoint = cv::Point());
    void        houghLines(std::vector<cv::Vec4i> & vOutLines);

    void        extractObject(const std::vector<cv::Rect> & rects, std::vector<cv::Mat> & images);
    void        hideGround(const  std::vector<cv::Rect> & rects, cv::Mat & image);

    double      getImgWidth();
    double      getImgHeight();

    cv::Point   getEdge(VEdge vertical = VE_MIDDLE, HEdge horizontal = HE_CENTER);

private:
    cv::Mat     m_ImageOrg;
    cv::Mat     m_ImageGray;

    double      m_dThresh;
    double      m_dMaxVal;
};

#endif // CVIMAGEANALYZER_H
