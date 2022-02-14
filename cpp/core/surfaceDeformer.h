#ifndef SURFACEDEFORMER_H
#define SURFACEDEFORMER_H

#include "surfaceCsrbf.h"

#include <Eigen/Sparse>

class surfaceDeformer
{
public:
  surfaceDeformer(surfaceCsrbf& psurface, pcl::PointCloud<pcl::PointXYZI>& dataPoint, float pratio);
  void deform(pcl::PointCloud<pcl::PointXYZI>& dataPoint);
  surfaceCsrbf& getSurface(){return surface;}
private:
  surfaceCsrbf surface;
  //surface tension vs data points energie ratio
  float ratio;
  void computeDeformations(pcl::PointCloud<pcl::PointXYZI>& dataPoint);
  Eigen::VectorXd fillV(size_t index , surfaceCsrbf& surface);
  Eigen::VectorXd fillDirac(size_t index , surfaceCsrbf& surface);
  Eigen::VectorXd getAlphas(int index);
  void normalizingAlphas(float threshold);
};

#endif // SURFACEDEFORMER_H
