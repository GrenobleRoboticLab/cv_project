#ifndef _FACETRACKER_H__
#define _FACETRACKER_H__

#include <ros/ros.h>
#include "face_tracker/LRG_Tools.h"

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace LRG {

/* ---------------------------------------------------------------------------------------- */

/* Helper Functions */
inline void ShowPicture(const std::string & sName, const cv::Mat & img)
{
  cv::namedWindow(sName, CV_WINDOW_AUTOSIZE);
  cv::moveWindow(sName, 100, 100);
  cv::imshow(sName, img);
}

/* ---------------------------------------------------------------------------------------- */

class Face
{
public:
    Face() { m_bHasLEye = m_bHasREye = m_bHasHead = false; }
    Face(const cv::Rect & rect);
    Face(const Face & face) { InitFrom(face); }
    virtual ~Face() { ; }

    const Face&  operator=(const Face & face);

    void         InitFrom(const Face & face);

    Status       SetHead(const cv::Rect & rect);
    Status       SetLEye(const cv::Rect & rect);
    Status       SetREye(const cv::Rect & rect);

    Status       DrawOn(cv::Mat & mat, const cv::Scalar & colorEye, const cv::Scalar & colorHead);

private:
    cv::Point    m_HeadCenter,
                 m_LEyeCenter,
                 m_REyeCenter;

    double       m_dXHeadRadius,
                 m_dYHeadRadius,
                 m_dLEyeRadius,
                 m_dREyeRadius;

    bool         m_bHasHead,
                 m_bHasLEye,
                 m_bHasREye;
};

/* ---------------------------------------------------------------------------------------- */

class FaceTracker
{
public:
    FaceTracker() { m_LoadStatus = E_FAIL; }
    FaceTracker(const cv::String & sFaceCascadeFile, const cv::String & sEyesCascadeFile);
    FaceTracker(const FaceTracker & faceTracker) { InitFrom(faceTracker); }
    
    virtual ~FaceTracker() { ; }

    const FaceTracker&       operator=(const FaceTracker & faceTracker);

    void                     InitFrom(const FaceTracker & faceTracker);

    Status                   SetCascadeFiles(const cv::String & sFaceCascadeFile, const cv::String & sEyesCascadeFile);
    Status                   Load();

    Status                   Track(const cv::Mat & mat, std::vector<Face> & vFaces);

    Status                   GetLoadStatus() { return m_LoadStatus; }

private:
    cv::String               m_sFaceCascadeFile,
                             m_sEyesCascadeFile;
    cv::CascadeClassifier    m_FaceCascade,
                             m_EyesCascade;
    cv::RNG                  m_Rng;

    Status                   m_LoadStatus;
};

} // namespace LRG

#endif // _FACETRACKER_H__

