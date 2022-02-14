#ifndef QUADLEAFCSRBF_H
#define QUADLEAFCSRBF_H

#include "surfacefrbf.h"
#include "csrbf.h"

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_pointcloud_adjacency.h>

#include <Eigen/Sparse>

class quadleafCsrbf
{
public:

    quadleafCsrbf(){}
    quadleafCsrbf(size_t pindexLeaf, surfaceFrbf& psurface,float refinementValue, pcl::PointCloud<pcl::PointXYZI> &pCloud);

    pcl::PointCloud<csrbfPCL>& getCloudCsrbf(){return cloudCsrbf;}

    Eigen::SparseMatrix<double>& getHSparse(){return HSparse;}
    Eigen::VectorXd& getAlpha(){return alpha;}
    void setAlpha(Eigen::VectorXd alphamod){alpha=alphamod;}

    void setAlpha(int k, double value){cloudCsrbf.at(k).alpha = value;}

    //void computeH(pcl::PointCloud<csrbfPCL>& fullCloud);
    //void solveSystem();

private:
    size_t indexLeaf;
    pcl::PointCloud<csrbfPCL> cloudCsrbf;

    pcl::octree::OctreePointCloudSearch<csrbfPCL>* octreeCsrbf;

    Eigen::SparseMatrix<double> HSparse;
    Eigen::VectorXd S,alpha;

    void buildListCsrbf(surfaceFrbf& psurface, float resolution, float valueThreshold);
    void decimateListCsrbf(pcl::PointCloud<pcl::PointXYZI> &pCloud, float valueThreshold);
    void addCsrbfToList(pcl::PointXYZ pCenter, float radius, surfaceFrbf& psurface, double threshold);
    void fillH(Eigen::SparseMatrix<double>& HSparse);
    void fillS(Eigen::VectorXd& S);
};

#endif // QUADLEAFCSRBF_H
