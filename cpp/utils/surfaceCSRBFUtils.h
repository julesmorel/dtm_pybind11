#ifndef SURFACECSRBFUTILS_H
#define SURFACECSRBFUTILS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "../core/surfaceCsrbf.h"

struct parametersNewton {
    pcl::PointCloud<csrbfPCL> listSph_r;
    pcl::PointXYZ po_r;
    pcl::Normal no_r;
};

class surfaceCSRBFUtils
{
public:
    static pcl::PointXYZ getPointOnSurfaceNewton(pcl::PointXYZ pt, pcl::Normal n, surfaceCsrbf& psurface);
    static double getValueImplicitFct(pcl::PointXYZ x, surfaceCsrbf& psurface, double radius);
    static pcl::Normal getValueImplicitFctGrad(pcl::PointXYZ x, surfaceCsrbf& psurface, double radius);
};

#endif // SURFACECSRBFUTILS_H
