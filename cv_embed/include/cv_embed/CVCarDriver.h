#ifndef CVCARDRIVER_H
#define CVCARDRIVER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <nxt_adventurer/Order.h>
#include <std_msgs/Empty.h>

#include "CVImageAnalyzer.h"
#include "CVMathHelper.h"

typedef std::vector<cv::Vec2i>  WAY;
typedef std::vector<WAY>        WAYS;

typedef std::vector<cv::Vec4i>  LINES;
typedef std::vector<cv::Rect>   RECTS;

class CVCarDriver
{
public:
    CVCarDriver();
    virtual ~CVCarDriver();

private:
    ros::NodeHandle                 m_NodeHandle;
    void                            init();

    // usb_cam communication
    image_transport::ImageTransport m_ImageTransport;
    image_transport::Subscriber     m_ImageSubscriber;
    void                            imageCallback(const sensor_msgs::ImageConstPtr & imagePtr);

    // arduino communication
    ros::Subscriber                 m_ReadySubscriber;
    ros::Publisher                  m_OrderPublisher;
    nxt_adventurer::Order           m_Order;
    bool                            m_bCarReady;
    bool                            sendOrder();
    void                            carReadyCallback(const std_msgs::Empty::ConstPtr & msgPtr);

    // traitement
    CVImageAnalyzer                 m_Analyzer;
    WAYS                            m_AvailableWays;
    LINES                           m_LinesFound;
    RECTS                           m_ObjectsFound;
    void                            findWays();
    void                            findWaysFromObjects();
    void                            findWaysFromLines();
    bool                            freeSpaceAvailable();
    void                            computeOrder();

    // retourne false si aucun objet n'est detecté, on part alors du principe que nous avons besoin des lignes.
    bool                            findObjects();
    // retourne false si il y a trop ou trop peu de ligne pour le calcul de WAYS, on part alors du principe que nous allons essayer de trouver les objets manuellement
    bool                            findLines();
    // essaye de trouver des objets a aprtir des corner de la méthode "goodFeaturesToTrack" => pas pour tout de suite
    bool                            findObjectsFromFeatures();
};

#endif // CVCARDRIVER_H
