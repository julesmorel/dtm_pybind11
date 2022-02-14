#ifndef SURFACEFRBFUTILS_H
#define SURFACEFRBFUTILS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "../core/surfacefrbf.h"

class surfaceFRBFUtils
{
public:
    static pcl::PointXYZ getPointOnSurfaceNewton(pcl::PointXYZ pt, pcl::Normal n, surfaceFrbf& psurface);
    static double computeImplicitFctValue(pcl::PointXYZ pt, surfaceFrbf& psurface);
    static bool computeImplicitFctValueAndPseudoGrad(pcl::PointXYZ pt, double& val, pcl::Normal& grad, surfaceFrbf& psurface);
    static double getSumRbfAtPoint(pcl::PointXYZ pt, surfaceFrbf& psurface);
};

#endif // SURFACEFRBFUTILS_H
