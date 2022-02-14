#ifndef MULTIDIMENSIONALMINIMIZER_H
#define MULTIDIMENSIONALMINIMIZER_H

#include "wendlandRbf.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <gsl/gsl_multimin.h>

struct parameters {
    pcl::PointCloud<pcl::PointXYZI> piCloud_p;
    pcl::PointXYZ r_p;
    pcl::PointXYZ xprime_p;
    pcl::Normal nprime_p;
    double support_p;
    double coef_p;
};

class multiDimensionalMinimizer
{
public:
    multiDimensionalMinimizer(){}
    multiDimensionalMinimizer(pcl::PointCloud<pcl::PointXYZI>& ppiCloud, pcl::PointXYZ pr, pcl::PointXYZ pxprime, pcl::Normal pnprime, float psupport, float ratio);
    ~multiDimensionalMinimizer(){}

    float getT(){return t;}
    pcl::Normal& getN(){return n;}

private:
    pcl::PointCloud<pcl::PointXYZ> piCloud;
    pcl::PointXYZ r;
    pcl::PointXYZ xprime;
    pcl::Normal nprime;

    float support;

    float t;
    pcl::Normal n;
};

#endif // MULTIDIMENSIONALMINIMIZER_H
