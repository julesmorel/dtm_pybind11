#ifndef DTM_H
#define DTM_H

#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "core/surfacefrbf.h"
#include "core/rectangle.h"

class dtm
{
public:
  dtm(){}
  dtm(std::string _filename, std::string _filenameRect, int _minNumberPointsPerLeaf, double _minSizeLeaf);
  void polygonize(int nx, int ny, int nz);
  void display();

private:
  pcl::PointCloud<pcl::PointXYZI> pointsMin;
  pcl::PointCloud<pcl::PointXYZ> pointsRectangleMin;

  surfaceFrbf functionalRBFModel;

  void readAsciiFile(std::string filename, pcl::PointCloud<pcl::PointXYZ>& points);
  pcl::PointCloud<pcl::PointXYZI> computeDensityAtEachPoints(pcl::PointCloud<pcl::PointXYZ> _inputPoints);
};

#endif // DTM_H
