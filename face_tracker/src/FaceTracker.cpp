#include "face_tracker/FaceTracker.h"

using namespace LRG;

/* ---------------------------------------------------------------------------------------- */

Face::Face(const cv::Rect & rect)
{
    m_bHasLEye = m_bHasREye = false;
    SetHead(rect);
}

const Face& Face::operator=(const Face & face)
{
    InitFrom(face);
    return *this;
}

void Face::InitFrom(const Face & face)
{
    m_HeadCenter   = face.m_HeadCenter;
    m_LEyeCenter   = face.m_LEyeCenter;
    m_REyeCenter   = face.m_REyeCenter;

    m_dXHeadRadius = face.m_dXHeadRadius;
    m_dYHeadRadius = face.m_dYHeadRadius;
    m_dLEyeRadius  = face.m_dLEyeRadius;
    m_dREyeRadius  = face.m_dREyeRadius;

    m_bHasHead     = face.m_bHasHead;
    m_bHasLEye     = face.m_bHasLEye;
    m_bHasREye     = face.m_bHasREye;
}

Status Face::SetHead(const cv::Rect & rect)
{
    Status ret = E_FAIL;

    if (!m_bHasLEye && !m_bHasREye)
    {
        m_HeadCenter   = cv::Point(rect.x + (rect.width / 2), rect.y + (rect.height / 2));
        m_dXHeadRadius = rect.width / 2;
        m_dYHeadRadius = rect.height / 2;
        m_bHasHead     = true;
        ret            = S_OK;
    }
    else
        ret = E_CONFLICT;

    return ret;
}

Status Face::SetLEye(const cv::Rect & rect)
{
    Status ret = E_FAIL;

    if (m_bHasHead)
    {
        m_LEyeCenter  = cv::Point(m_HeadCenter.x + (rect.x / 2), m_HeadCenter.y + (rect.y / 2));
        m_dLEyeRadius = cvRound((rect.width + rect.height) / 4);
        m_bHasLEye    = true;
        ret           = S_OK;      
    }
    else
        ret = E_NOTINIT;

    return ret;
}

Status Face::SetREye(const cv::Rect & rect)
{
    Status ret = E_FAIL;

    if (m_bHasHead)
    {
        m_LEyeCenter  = cv::Point(m_HeadCenter.x + (rect.x / 2), m_HeadCenter.y + (rect.y / 2));
        m_dREyeRadius = cvRound((rect.width + rect.height) / 4);
        m_bHasREye    = true;
        ret           = S_OK;
    }
    else
        ret = E_NOTINIT;

    return ret;
}

Status Face::DrawOn(cv::Mat & mat, const cv::Scalar & eyeColor, const cv::Scalar & headColor)
{
    Status ret = S_OK;

    if (m_bHasHead)
        cv::ellipse(mat, m_HeadCenter, cv::Size(m_dXHeadRadius, m_dYHeadRadius), 0, 0, 360, headColor, 4, 8, 0 );

    if (m_bHasLEye)
        cv::circle(mat, m_LEyeCenter, m_dLEyeRadius, eyeColor, 4, 8, 0 );       

    if (m_bHasREye)
        cv::circle(mat, m_LEyeCenter, m_dLEyeRadius, eyeColor, 4, 8, 0 );

    return ret;
}

bool Face::GetHeadCenter(cv::Point & point) const
{
    bool bRet = false;

    if (m_bHasHead)
    {
        point = m_HeadCenter;
        bRet = true;
    }

    return bRet;
}

bool Face::GetHeadRadius(double & dRadius) const
{
    bool bRet = false;
    
    if (m_bHasHead)
    {
        dRadius = m_dXHeadRadius;
	bRet = true;
    }

    return bRet;
}

/* ---------------------------------------------------------------------------------------- */

FaceTracker::FaceTracker(const cv::String & sFaceCascadeFile, const cv::String & sEyesCascadeFile)
{
    m_sFaceCascadeFile = cv::String(sFaceCascadeFile);
    m_sEyesCascadeFile = cv::String(sEyesCascadeFile);
    Load();
    m_Rng              = cv::RNG(12345);
}

const FaceTracker& FaceTracker::operator=(const FaceTracker & faceTracker)
{
    InitFrom(faceTracker);
    return *this;
}

void FaceTracker::InitFrom(const FaceTracker & faceTracker)
{
    m_sFaceCascadeFile = faceTracker.m_sFaceCascadeFile;
    m_sEyesCascadeFile = faceTracker.m_sFaceCascadeFile;
    m_FaceCascade      = faceTracker.m_FaceCascade;
    m_EyesCascade      = faceTracker.m_EyesCascade;
    m_Rng              = cv::RNG(faceTracker.m_Rng);
}

Status FaceTracker::SetCascadeFiles(const cv::String & sFaceCascadeFile, const cv::String & sEyesCascadeFile)
{
    m_sFaceCascadeFile = cv::String(sFaceCascadeFile);
    m_sEyesCascadeFile = cv::String(sEyesCascadeFile);
    
    return Load();
}

Status FaceTracker::Load()
{
    Status    ret        = E_FAIL,
              faceStatus = E_FAIL,
              eyesStatus = E_FAIL;

    if (m_FaceCascade.load(m_sFaceCascadeFile))
        faceStatus = S_OK;
    else
        ROS_ERROR("Unable to load face cascade file.");

    if (m_EyesCascade.load(m_sEyesCascadeFile))
        eyesStatus = S_OK;
    else
        ROS_ERROR("Unable to load eyes cascade file.");

    if (ISOK(faceStatus) && ISOK(eyesStatus))
        ret = S_OK;

    m_LoadStatus = ret;

    return ret;
}

Status FaceTracker::Track(const cv::Mat & mat, std::vector<Face> & vFaces)
{
    Status ret = E_FAIL;

    std::vector<cv::Rect> faces;
    cv::Mat frameGray;

    if (vFaces.size() > 0)
        vFaces.clear();

    cv::cvtColor(mat, frameGray, CV_BGR2GRAY);
    cv::equalizeHist(frameGray, frameGray);

    m_FaceCascade.detectMultiScale(frameGray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));

    for(unsigned int i = 0; i < faces.size(); i++ )
    {
        Face    face(faces[i]);
        cv::Mat faceROI = frameGray(faces[i]);
        
        std::vector<cv::Rect> eyes;
        m_EyesCascade.detectMultiScale(faceROI, eyes, 1.1, 2, 0 |CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));

        if (eyes.size() == 2)
        {
            face.SetLEye(eyes[0]);
            face.SetREye(eyes[1]);
        }

        vFaces.push_back(face);
    }

    if (vFaces.size() > 0)
        ret = S_OK;

    return ret;
}


/* ---------------------------------------------------------------------------------------- */
