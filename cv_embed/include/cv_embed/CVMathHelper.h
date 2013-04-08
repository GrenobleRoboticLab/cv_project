#ifndef CVMATHHELPER_H
#define CVMATHHELPER_H

#include <opencv2/opencv.hpp>

/* Ce fichier ne contient que des méthodes de mathématique "pure" destiner à être utilisé dans de nombreux fichiers */

/** indique si la valeur est comprise entre min et max */
template<typename T>
inline bool inRange(T val, T min, T max) { return (val >= min && val <= max); }

/** retoune la moyenne du vector passé en paramètre */
template<typename T>
inline T average(const std::vector<T> & vValues)
{
    T sum = 0;

    for (uint i = 0; i < vValues.size(); i++)
        sum += vValues.at(i);

    return (sum / vValues.size());
}

/** retourne la moyenne du vecteur passé en paramètre ici on regrdera les valeurs pointées par l'indice (également en paramètre) */
template<typename T>
inline T average(const std::vector<cv::Vec<T, 2> > & vValues, uint nIndice)
{
    T sum = 0;

    if (nIndice > 1)
        return sum;

    for (uint i = 0; i < vValues.size(); i++)
        sum += vValues.at(i)[nIndice];

    return (sum / vValues.size());
}

/** retourne la longueur du segment identifié par (x1, y1) et (x2, y2) */
template<typename T>
inline double getLineLength(T x1, T y1, T x2, T y2)                         { return sqrt(pow((x2 - x1), 2) - pow((y2 - y1), 2));       }
/** methode de convivialité, reviens à appeler getLineLength(T x1, T y1, T x2, T y2) */
template<typename T>
inline double getLineLength(const cv::Vec<T, 4> & line)                     { return getLineLength(line[0], line[1], line[2], line[3]); }
/** methode de convivialité, reviens à appeler getLineLength(T x1, T y1, T x2, T y2) */
inline double getLineLength(const cv::Point & pt1, const cv::Point & pt2)   { return getLineLength(pt1.x, pt1.y, pt2.x, pt2.y);         }

/** retourne l'équation de la droite correspondant au segment identifié par (x1, y1) et (x2, y2) */
template<typename T>
inline cv::Vec<T, 2> getLineEq(T x1, T y1, T x2, T y2)
{
    cv::Vec<T, 2> vRet;

    if (x2 != x1)
        vRet[0] = (y2 - y1) / (x2 - x1);
    else vRet[0] = 1;
    vRet[1] = y1 - (vRet[0] * x1);

    return vRet;
}
/** methode de convivialité, reviens à appeler getLineEq(T x1, T y1, T x2, T y2) */
template<typename T>
inline cv::Vec<T, 2> getLineEq(const cv::Vec<T, 4> & line) { return getLineEq(line[0], line[1], line[2], line[3]); }
/** methode de convivialité, reviens à appeler getLineEq(T x1, T y1, T x2, T y2) */
template<typename T>
inline cv::Vec<T, 2> getLineEq(const cv::Point & pt1, const cv::Point & pt2) { return getLineEq(pt1.x, pt1.y, pt2.x, pt2.y); }

/** applique l'équation donnée au x donné : f(x) = eq[0] * x + eq[1] */
template<typename T>
inline T applyStraightEq(T x, const cv::Vec<T, 2> & eq) { return (x * eq[0]) + eq[1]; }

/** retourne true si les droite se coupe en un point (outPoint), retourne false si les droites sont parrallèles */
template<typename T>
inline bool getIntersectStraight(const cv::Vec<T, 2> & eq1, const cv::Vec<T, 2> & eq2, cv::Point & outPoint)
{
    bool bRet = false;

    // si les droites sont paralelle, on l'a dans le ... :(
    if (eq1[0] != eq2[0])
    {
        // sinon :)
        bRet = true;
        outPoint.x = (eq2[1] - eq1[1]) / (eq1[0] - eq2[0]);
        outPoint.y = applyStraightEq(outPoint.x, eq1);
    }

    return bRet;
}

/** génère le rectangle englobant du polygone passé en paramètre */
inline void computeArea(const std::vector<cv::Point> & vPoly, cv::Rect & pBoundedRect)
{
    int     nMinX,
            nMaxX,
            nMinY,
            nMaxY;

    nMinX = nMaxX = (vPoly.size()) ? vPoly[0].x : 0;
    nMinY = nMaxY = (vPoly.size()) ? vPoly[0].y : 0;

    for (size_t i = 0; i < vPoly.size() - 1; i++)
    {
        if (vPoly[i].x < nMinX)
            nMinX = vPoly[i].x;
        else if (vPoly[i].x > nMaxX)
            nMaxX = vPoly[i].x;

        if (vPoly[i].y < nMinY)
            nMinY = vPoly[i].y;
        else if (vPoly[i].y > nMaxY)
            nMaxY = vPoly[i].y;
    }

    pBoundedRect.x       = nMinX;
    pBoundedRect.y       = nMinY;
    pBoundedRect.width   = nMaxX - nMinX;
    pBoundedRect.height  = nMaxY - nMinY;
}

/** calcul la norme du vecteur passé en paramètre */
template<typename T>
inline T vectorNorme(T x, T y, T z = 0)             { return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2)); }
/** methode de convivialité, reviens à appeler vectorNorme(T x, T y, T z = 0) */
template<typename T>
inline T vectorNorme(const cv::Vec<T, 3> & v)  { return vectorNorme(v[0], v[1], v[2]); }

/** effectue la multiplication de deux vecteurs */
template<typename T>
inline cv::Vec<T, 3> vectorFactor(T x1, T y1, T z1, T x2, T y2, T z2)
{
    cv::Vec<T, 3>   v3Ret;

    v3Ret[0] = (y1 * z2) - (z1 * y2);
    v3Ret[0] = (z1 * x2) - (x1 * z2);
    v3Ret[0] = (x1 * y2) - (y1 * x2);

    return v3Ret;
}
/** methode de convivialité, reviens à appeler vectorFactor(T x, T y, T z = 0) */
template<typename T>
inline cv::Vec<T, 3> vectorFactor(const cv::Vec<T, 3> & v1, const cv::Vec<T, 3> & v2) { return vectorFactor(v1[0], v1[1], v1[2], v2[0], v2[1], v2[2]); }

/** retourne la distance entre un point et une ligne */
template<typename T>
inline T DPoin2tLine(const cv::Point & p, const cv::Vec<T, 4> & l)
{
    cv::Vec<T, 3> v, u;

    v[0] = p.x - l[0];
    v[1] = p.y - l[1];
    v[2] = 0;
    u[0] = l[0] - l[2];
    u[1] = l[1] - l[3];
    u[2] = 0;

    return (vectorNorme(vectorFactor(u, v)) / vectorNorme(u));
}

#endif // CVMATHHELPER_H
