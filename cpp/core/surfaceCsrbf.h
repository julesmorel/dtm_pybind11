#ifndef SURFACECSRBF_H
#define SURFACECSRBF_H

#include "surfacefrbf.h"
#include "csrbf.h"
#include "quadleafCsrbf.h"

#include "../plot/implicitfunction.h"

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_pointcloud_adjacency.h>

#include <Eigen/Sparse>

class surfaceCsrbf : public implicitFunction
{
public:

    surfaceCsrbf(){}
    surfaceCsrbf(surfaceFrbf& psurface, pcl::PointCloud<pcl::PointXYZI> &pCloud, float refinementValue=0.1f);

    pcl::PointCloud<csrbfPCL>& getCloudCsrbf(){return cloudCsrbf;}
    std::vector<quadleafCsrbf>& getListLeafs(){return listLeafs;}
    void updateListCsrbf();

    pcl::octree::OctreePointCloudSearch<csrbfPCL>* getOctree(){return octreeCsrbf;}

    virtual void fillScalarField(scalarField &field);

    void printCSRBF(std::string filename);

private:
    pcl::PointCloud<csrbfPCL> cloudCsrbf;
    pcl::octree::OctreePointCloudSearch<csrbfPCL>* octreeCsrbf;
    std::vector<quadleafCsrbf> listLeafs;
};

#endif // SURFACECSRBF_H
