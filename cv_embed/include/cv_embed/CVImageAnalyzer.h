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

/** permet d'extraire des informations d'une image BGR */
class CVImageAnalyzer
{
public:
    CVImageAnalyzer(double dThresh, double dMaxVal);
    virtual ~CVImageAnalyzer() { ; }

    /** assigne l'image dorigine */
    void        setImageOrg(const cv::Mat & imageOrg);
    /** assigne le seuil utilisé par les méthodes canny et threshold d'opencv */
    void        setThresh(double dThresh);
    /** assigne le seuil utilisé par la méthode canny ou la valeur max de la méthode threshold */
    void        setMaxVal(double dMaxVal);

    /** retourne les coins (angles) détecté dans l'image */
    void        goodFeaturesToTrack(std::vector<cv::Point2f> & vFeatures, int nCountMax, double dQualityLevel, double dMinDist);
    /** rempli une liste de polygones représentant les objets décellés (appel la méthode threshold d'opencv) */
    void        findContours(std::vector<std::vector<cv::Point> > & contours, cv::Point offsetPoint = cv::Point());
    /** rempli une liste de ligne avec les lignes detectées dans l'image (utilise la méthode canny d'opencv) */
    void        houghLines(std::vector<cv::Vec4i> & vOutLines);

    /** rempli une liste d'image pour chaque rectangle passé en paramètre, chaque image de sortie étant un clip de l'image d'origine */
    void        extractObject(const std::vector<cv::Rect> & rects, std::vector<cv::Mat> & images);
    /** expérimentation, ne sert à rien pour l'instant */
    void        hideGround(const  std::vector<cv::Rect> & rects, cv::Mat & image);

    /** retourne la largeur de l'image */
    double      getImgWidth();
    /** retourne la hauteur de l'image */
    double      getImgHeight();

    /** retourne la position du point demandé */
    cv::Point   getEdge(VEdge vertical = VE_MIDDLE, HEdge horizontal = HE_CENTER);

private:
    cv::Mat     m_ImageOrg;
    cv::Mat     m_ImageGray;

    double      m_dThresh;
    double      m_dMaxVal;
};

#endif // CVIMAGEANALYZER_H
