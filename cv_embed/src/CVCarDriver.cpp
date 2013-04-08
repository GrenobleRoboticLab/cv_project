#include "cv_embed/CVCarDriver.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

CVCarDriver::CVCarDriver() : m_ImageTransport(m_NodeHandle), m_Analyzer(100, 200)
{
    m_bCarReady = true;
    init();
}

CVCarDriver::~CVCarDriver() { ; }

void CVCarDriver::init()
{
    m_ReadySubscriber = m_NodeHandle.subscribe("ard_done", 1, &CVCarDriver::carReadyCallback, this);
    m_ImageSubscriber = m_ImageTransport.subscribe("/usb_cam/image_raw", 1, &CVCarDriver::imageCallback, this);

    m_OrderPublisher = m_NodeHandle.advertise<nxt_adventurer::Order>("car_order", 1);
    m_Order.order = 3;
    m_Order.effort = 30;
    m_OrderPublisher.publish(m_Order);
}

void CVCarDriver::imageCallback(const sensor_msgs::ImageConstPtr &imagePtr)
{
    cv_bridge::CvImage      cvBImage;
    cv_bridge::CvImagePtr   cvBImagePtr;

    try
    {
        cvBImagePtr         = cv_bridge::toCvCopy(imagePtr, sensor_msgs::image_encodings::BGR8);
        cvBImage.image      = cvBImagePtr->image;
        cvBImage.encoding   = cvBImagePtr->encoding;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    /* assigne l'image que l'on vient de recevoir à notre Analyzer */
    m_Analyzer.setImageOrg(cvBImage.image);
    /* cherche des chemins possible dans l'image */
    findWays();

    /* DEBUG */
    for (uint i = 0; i < m_ObjectsFound.size(); i++)
        cv::rectangle(cvBImage.image, m_ObjectsFound[i], cv::Scalar(255, 0, 0), 2);

    for (uint i = 0; i < m_LinesFound.size(); i++)
        cv::line(cvBImage.image, cv::Point(m_LinesFound.at(i)[0], m_LinesFound.at(i)[1]), cv::Point(m_LinesFound.at(i)[2], m_LinesFound.at(i)[3]), cv::Scalar(0, 255, 0), 2);

    cv::namedWindow("visual");
    cv::imshow("visual", cvBImage.image);
    cv::waitKey(5);
    /* FIN DEBUG */

    /* génère un ordre à partir des chemins trouvés */
    computeOrder();
    /* envoie l'ordre sur le topic */
    sendOrder();
}

bool CVCarDriver::sendOrder()
{
    /* si la voiture est prête, on envoie l'ordre */
    bool bRet = m_bCarReady;

    if (m_bCarReady)
    {
        m_bCarReady = false;
        m_OrderPublisher.publish(m_Order);
    }

    return bRet;
}

void CVCarDriver::carReadyCallback(const std_msgs::Empty::ConstPtr &msgPtr)
{
    m_bCarReady = true;
}

void CVCarDriver::findWays()
{
    /* regarde si la méthode findObject réussit, sinon on essaye de trouver les objets à partir des angles de la méthode goodFeaturesToTrack */
    if (!findObjects())
        findObjectsFromFeatures();
    /* récupère les lignes */
    findLines();
    /* construit les chemins en fonction des information précedemment perçue */
    findWaysFromObjects();
    findWaysFromLines();
}

void CVCarDriver::findWaysFromObjects()
{
    /* teste si le tiers inférieur de l'image est encombré */
    const bool      bFront  = freeSpaceAvailable();
    const uint      maxX    = m_Analyzer.getImgWidth();
    uint            current = 0;

    /* un chemin correspond à un ensemble de segments depuis la position de la voiture jusqu'au bord supérieur de l'image */
    cv::Point       wayTested(0, 0);
    /* part du principe que la position de la voiture correspond au point centrale de la dernière ligne de l'image */
    const cv::Point carPos(m_Analyzer.getEdge(VE_BOT, HE_CENTER));

    WAY             tmpWay;

    cv::Vec2i       eq1;

    /* pour chaque colone de notre image */
    for (;current <= maxX; current++)
    {
        bool    bOccupiedWay    = false;
        wayTested.x             = current;
        eq1                     = getLineEq<int>(carPos, wayTested);

        /* pour chaque objets trouvé */
        for (uint i = 0; i < m_ObjectsFound.size(); i++)
        {
            cv::Point   intersectPoint;
            cv::Rect*   pRect = &m_ObjectsFound[i];

            /* on va chercher si notre segment passe par un des cotés du rectangle courant */
            /* si oui, le segment est encombré alors on passe au segment suivant, sinon on continue jusqu'a avoir testé tous les objets */
            if (getIntersectStraight(eq1, getLineEq(pRect->x, pRect->y, pRect->x + pRect->width, pRect->y), intersectPoint))
            {
                if (inRange(intersectPoint.x, pRect->x, pRect->x + pRect->width) && inRange(intersectPoint.y, pRect->y, pRect->y + pRect->height))
                {
                    bOccupiedWay = true;
                    continue;
                }
            }
            if (getIntersectStraight(eq1, getLineEq(pRect->x, pRect->y, pRect->x, pRect->y + pRect->height), intersectPoint))
            {
                if (inRange(intersectPoint.x, pRect->x, pRect->x + pRect->width) && inRange(intersectPoint.y, pRect->y, pRect->y + pRect->height))
                {
                    bOccupiedWay = true;
                    continue;
                }
            }
            if (getIntersectStraight(eq1, getLineEq(pRect->x, pRect->y + pRect->height, pRect->x + pRect->width, pRect->y + pRect->height), intersectPoint))
            {
                if (inRange(intersectPoint.x, pRect->x, pRect->x + pRect->width) && inRange(intersectPoint.y, pRect->y, pRect->y + pRect->height))
                {
                    bOccupiedWay = true;
                    continue;
                }
            }
            if (getIntersectStraight(eq1, getLineEq(pRect->x + pRect->width, pRect->y, pRect->x + pRect->width, pRect->y + pRect->height), intersectPoint))
            {
                if (inRange(intersectPoint.x, pRect->x, pRect->x + pRect->width) && inRange(intersectPoint.y, pRect->y, pRect->y + pRect->height))
                {
                    bOccupiedWay = true;
                    continue;
                }
            }
        }

        /* si notre segment est encombré et que nous avons un chemin en construction, on copie le chemin dans les chemins possible et on vide le chemin temporaire */
        if (bOccupiedWay && tmpWay.size())
        {
            m_AvailableWays.push_back(tmpWay);
            tmpWay.clear();
        }
        // sinon on ajoute notre segment dans le chemin courrant */
        else
        {
            cv::Vec2i v;
            v[0]        = current;

            /* si le tiers inférieur de l'image est encombré, on se dirige vers l'arrière, sinon ver l'avant */
            if (bFront)
                v[1]    = 1;
            else
                v[1]    = -1;

            tmpWay.push_back(v);
        }
    }

    /* quand nous avons fini notre parcour, on ajoute le dernier chemin trouvé (si il existe) */
    if (tmpWay.size())
        m_AvailableWays.push_back(tmpWay);
}

void CVCarDriver::findWaysFromLines()
{
    // todo
}

bool CVCarDriver::freeSpaceAvailable()
{
    /* on teste si tous les coins des lignes et objets trouvé dans l'image ne sont pas dans le tiers inférieur de l'image */
    bool bRet = true;
    cv::Rect    freeRect(0, (m_Analyzer.getImgHeight() * 2 / 3), m_Analyzer.getImgWidth(), m_Analyzer.getImgHeight());

    for (uint i = 0; i < m_ObjectsFound.size(); i++)
    {
        cv::Rect*   r = &m_ObjectsFound[i];
        if (freeRect.contains(cv::Point(r->x, r->y))
         || freeRect.contains(cv::Point(r->x + r->width, r->y))
         || freeRect.contains(cv::Point(r->x, r->y + r->height))
         || freeRect.contains(cv::Point(r->x + r->width, r->y + r->height)))
        {
            bRet = false;
            continue;
        }
    }

    if (!bRet)
    {
        for (uint i = 0; i < m_LinesFound.size(); i++)
        {
            if (freeRect.contains(cv::Point(m_LinesFound[i][0], m_LinesFound[i][1]))
             || freeRect.contains(cv::Point(m_LinesFound[i][2], m_LinesFound[i][3])))
            {
                bRet = false;
                continue;
            }
        }
    }

    return bRet;
}

void CVCarDriver::computeOrder()
{
    WAY*    pBestWay = NULL;
    double  avr;

    /* il est possible qu'auncun chemin n'est été trouvé :'( */
    if (!m_AvailableWays.size())
    {
        std::cout << "No way available" << std::endl;
    }
    /* sinon :) */
    else
    {
        /* pour chaque chemin nous allons récupérer le chemin qui a la plus grande largeur */
        for (uint i = 0; i < m_AvailableWays.size(); i++)
        {
            if (pBestWay)
            {
                if (pBestWay->size() < m_AvailableWays[i].size())
                    pBestWay = &m_AvailableWays[i];
            }
            else
                pBestWay = &m_AvailableWays[i];
        }

        /* on récupère la position moyenne du chemin (axe des ordonnées) */
        avr = average(*pBestWay, 0);

        /* si le chemin va vers l'avant */
        if (pBestWay->at(0)[1] > 0)
        {
            std::cout << "Go";
            /* la moyenne se situe dans le tiers gauche de l'image */
            if(avr <= (m_Analyzer.getImgWidth() / 3))
                std::cout << " to the left" << std::endl;
            /* la moyenne se situe dans le tiers central de l'image */
            else if (avr <= ((2 * m_Analyzer.getImgWidth()) / 3))
                std::cout << " ahead" << std::endl;
            /* la moyenne se situe dans le tiers droit de l'image */
            else if (avr <= m_Analyzer.getImgWidth())
                std::cout << " to the right" << std::endl;
        }
        /* sinon */
        else if (pBestWay->at(0)[1] < 0)
        {
            /* on fait comme vers l'avant sauf qu'on oriente les roues de la voiture dans la direction opposée. */
            std::cout << "Back";
            if(avr <= (m_Analyzer.getImgWidth() / 3))
                std::cout << " to the right" << std::endl;
            else if (avr <= ((2 * m_Analyzer.getImgWidth()) / 3))
                std::cout << " ahead" << std::endl;
            else if (avr <= m_Analyzer.getImgWidth())
                std::cout << " to the left" << std::endl;
        }
    }

    m_AvailableWays.clear();
}

bool CVCarDriver::findObjects()
{
    bool                                    bRet = false;
    std::vector<std::vector<cv::Point> >    contours;

    m_ObjectsFound.clear();
    m_Analyzer.findContours(contours);

    for (uint i = 0; i < contours.size(); i++)
    {
        cv::Rect    boundedRect;
        computeArea(contours[i], boundedRect);
        if(boundedRect.area() > 1000 && boundedRect.area() < (m_Analyzer.getImgWidth() * m_Analyzer.getImgHeight() * 9 / 10))
            m_ObjectsFound.push_back(boundedRect);
    }

    if (m_ObjectsFound.size())
        bRet = true;

    return bRet;
}

bool CVCarDriver::findLines()
{
    bool bRet = false;
    LINES  vTmpLines;

    m_LinesFound.clear();
    m_Analyzer.houghLines(vTmpLines);

    for (uint i = 0; i < vTmpLines.size(); i++)
    {
        if (getLineLength(vTmpLines[i]) > 100)
            m_LinesFound.push_back(vTmpLines[i]);
    }

    return bRet;
}

bool CVCarDriver::findObjectsFromFeatures()
{
    bool bRet = false;
    // todo
    return bRet;
}
