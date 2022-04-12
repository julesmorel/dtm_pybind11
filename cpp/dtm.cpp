#include "dtm.h"
#include "visualizer.h"
#include "core/surfaceDeformer.h"

#include<iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <pcl/common/common.h>
#include <pcl/io/obj_io.h>

dtm::dtm(std::string _filename, std::string _filenameRect, int _minNumberPointsPerLeaf, double _minSizeLeaf){
  pcl::PointCloud<pcl::PointXYZ> pointMinTemp;
  readAsciiFile(_filename,pointMinTemp);
  pointsMin = computeDensityAtEachPoints(pointMinTemp);
  readAsciiFile(_filenameRect,pointsRectangleMin);
  surfaceFrbf surface(pointsMin,_minNumberPointsPerLeaf,_minSizeLeaf,pointsRectangleMin);
  functionalRBFModel=surface;
  deformableModelApplied=false;
}

void dtm::polygonize(int nx, int ny, int nz){
  pcl::PointXYZI min,max;
  pcl::getMinMax3D (pointsMin, min, max);
  rectangle tile(pointsRectangleMin);
  functionalRBFModel.polygonizeLevelSet(tile,min.z-1.5,max.z+1.5,nx,ny,nz);
  if(deformableModelApplied)CSRBFModel.polygonizeLevelSet(tile,min.z-1.5,max.z+1.5,nx,ny,nz);
}

void dtm::display(){
  functionalRBFModel.getTree()->buildLeafsSurface();
  visualizer v(pointsMin,pointsRectangleMin, functionalRBFModel, CSRBFModel, deformableModelApplied);
  v.plot();
}

void dtm::applyDeformableModel(int numberIteration, double gamma){
  deformableModelApplied=true;
  surfaceCsrbf surface(functionalRBFModel, pointsMin);
  CSRBFModel=surface;
  for(int it=0;it<numberIteration;it++)
  {
    surfaceDeformer deformer(CSRBFModel,pointsMin,gamma);
    deformer.deform(pointsMin);
    surfaceCsrbf surfCsrbftmp = deformer.getSurface();
    CSRBFModel=surfCsrbftmp;
  }
}

void dtm::exportDTM(std::string filename){
  pcl::PolygonMesh mesh;
  if(deformableModelApplied){
    const isoSurface iso = CSRBFModel.getLevelSet();
    mesh = buildMeshFromSurface(iso);
  }else{
    const isoSurface iso = functionalRBFModel.getLevelSet();
    mesh = buildMeshFromSurface(iso);
  }
  std::cout<<"saving obj"<<std::endl;
  pcl::io::saveOBJFile (filename, mesh);
}

pcl::PolygonMesh dtm::buildMeshFromSurface(const isoSurface &iso){
  pcl::PolygonMesh mesh;
  pcl::PointCloud<pcl::PointXYZ> polygonsPts;
  std::vector<pcl::Vertices> polygonIds;

  for(std::size_t i=0; i<iso.getTriangles().size();i++)
  {
      pcl::Vertices face;
      face.vertices.push_back(iso.getTriangles().at(i).pointIndex[0]);
      face.vertices.push_back(iso.getTriangles().at(i).pointIndex[1]);
      face.vertices.push_back(iso.getTriangles().at(i).pointIndex[2]);
      polygonIds.push_back(face);
  }

  for(std::size_t i=0; i<iso.getMapPoints().size();i++)
  {
    pcl::PointXYZ p = iso.getMapPoints().at(i);
    polygonsPts.push_back(p);
  }

  pcl::toPCLPointCloud2 (polygonsPts, mesh.cloud);
  mesh.polygons = polygonIds;
  return mesh;
}

void dtm::readAsciiFile(std::string filename, pcl::PointCloud<pcl::PointXYZ>& points){
  std::ifstream file(filename);
  if (file.is_open()) {
    std::string line;
    while (getline(file, line)) {
      std::vector<std::string> results;
      boost::split(results, line, [](char c){return c == ' ';});

      pcl::PointXYZ p;
      p.x=std::stod (results.at(0));
      p.y=std::stod (results.at(1));
      p.z=std::stod (results.at(2));
      points.push_back(p);
    }
    file.close();
  }
}

pcl::PointCloud<pcl::PointXYZI> dtm::computeDensityAtEachPoints(pcl::PointCloud<pcl::PointXYZ> _inputPoints){
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (_inputPoints.makeShared());

    double maxDensity = 0.;
    pcl::PointCloud<pcl::PointXYZI> ptsMinEnriched;
    std::vector<double> densities;
    for(int i=0;i<_inputPoints.size();i++){
      pcl::PointXYZ currentPt = _inputPoints.at(i);
      std::vector<int> k_indices(20);
      std::vector<float> k_sqr_distances(20);
      kdtree.nearestKSearch(currentPt,20,k_indices,k_sqr_distances);
      double sum = std::accumulate(k_sqr_distances.begin(), k_sqr_distances.end(), 0.0);
      densities.push_back(sum);
      if(sum>maxDensity)maxDensity=sum;
    }
    for(int i=0;i<_inputPoints.size();i++){
        pcl::PointXYZ currentPt = _inputPoints.at(i);
        pcl::PointXYZI p;
        p.x=currentPt.x;
        p.y=currentPt.y;
        p.z=currentPt.z;
        double sum = densities.at(i);
        p.intensity=1.-sum/maxDensity;
        ptsMinEnriched.push_back(p);

    }
    return ptsMinEnriched;
}
