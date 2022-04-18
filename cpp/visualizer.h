#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "dtm.h"


class visualizer
{
public:
    visualizer(){}
    visualizer(pcl::PointCloud<pcl::PointXYZI> _pointsMin, pcl::PointCloud<pcl::PointXYZ> _pointsRectangleMin, surfaceFrbf _functionalRBFModel, surfaceCsrbf _CSRBFModel, bool _deformableModelApplied);
    void plot();
private:
    pcl::PointCloud<pcl::PointXYZI> pointsMin;
    pcl::PointCloud<pcl::PointXYZ> pointsRectangleMin;
    surfaceFrbf functionalRBFModel;
    surfaceCsrbf CSRBFModel;
    bool deformableModelApplied;
    float lowestPtsZ;
    pcl::PointCloud<pcl::PointXYZRGB> addColorToPointCloudwithI(pcl::PointCloud<pcl::PointXYZI> pts, uint8_t r, uint8_t g, uint8_t b,float deltaZ);
    void addSquare(pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointCloud<pcl::PointXYZ> &rect, float z, double r, double g, double b, std::string stringid, float alpha);
    void addQuadtreeAsMesh(pcl::visualization::PCLVisualizer::Ptr viewer, float z, double r, double g, double b, std::string stringid, float alpha);
    void addSurfaceMesh(pcl::visualization::PCLVisualizer::Ptr viewer ,const isoSurface & surface, float deltaZ, double r, double g, double b, std::string stringid);
    void addQuadsSurfaceMesh(pcl::visualization::PCLVisualizer::Ptr viewer, float z);
    void addBorderCells(pcl::visualization::PCLVisualizer::Ptr viewer, float z, double r, double g, double b, std::string stringid, float alpha);

};
