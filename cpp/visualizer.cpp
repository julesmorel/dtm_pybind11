#include "visualizer.h"
#include <iostream>
#include <thread>
#include <pcl/PCLPointCloud2.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/io/ply_io.h>

visualizer::visualizer(pcl::PointCloud<pcl::PointXYZI> _pointsMin, pcl::PointCloud<pcl::PointXYZ> _pointsRectangleMin, surfaceFrbf _functionalRBFModel)
{
  pointsMin=_pointsMin;
  pointsRectangleMin=_pointsRectangleMin;
  functionalRBFModel=_functionalRBFModel;
  float zmin=std::numeric_limits<float>::infinity();
  for(int i=0;i<pointsMin.size();i++){
    if(pointsMin.at(i).z<zmin){
        zmin = pointsMin.at(i).z;
    }
  }
  lowestPtsZ =zmin;
}

pcl::PointCloud<pcl::PointXYZRGB> visualizer::addColorToPointCloudwithI(pcl::PointCloud<pcl::PointXYZI> pts, uint8_t r, uint8_t g, uint8_t b, float deltaZ=0.)
{
  pcl::PointCloud<pcl::PointXYZRGB> ptsrgb;
  for(int i=0;i<pts.size();i++){
    pcl::PointXYZRGB prgb;
    prgb.x=pts.at(i).x;
    prgb.y=pts.at(i).y;
    prgb.z=pts.at(i).z+deltaZ;
    // pack r/g/b into rgb
    float it = 1.-pts.at(i).intensity;
    uint32_t rgb = ((uint32_t)((float)r*it) << 16 | (uint32_t)((float)g*it) << 8 | (uint32_t)((float)b*it));
    prgb.rgb = *reinterpret_cast<float*>(&rgb);
    ptsrgb.push_back(prgb);
  }
  return ptsrgb;
}

void visualizer::plot()
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

  float deltaZ=2.0;

  viewer->setBackgroundColor (0.9, 0.9, 0.9);

  pcl::PointCloud<pcl::PointXYZRGB> minRGB = addColorToPointCloudwithI(pointsMin,200,250,200,-0.*deltaZ);

  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr minptr = minRGB.makeShared();
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_min(minptr);
  viewer->addPointCloud<pcl::PointXYZRGB> (minptr, rgb_min, "min");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "min");

  // //plot the quadtree
   addQuadtreeAsMesh(viewer,lowestPtsZ-1*deltaZ,0.5,0.5,0.5,"quatree",0.98);
  // addSquare(viewer,pointsRectangleMin,lowestPtsZ-0.4*deltaZ,0.9,0.9,0.4,"bbs",1.0);
  // addMinPointsConcaveHull(viewer,lowestPtsZ-0.4*deltaZ);
  // addSquare(viewer,pointsRectangleMin,lowestPtsZ-0.8*deltaZ,0.7,0.7,0.9,"bbsm",1.0);
   addSurfaceMesh(viewer,0);
  //
  // viewer->addCoordinateSystem (1.0);
  // viewer->initCameraParameters ();
  //
  // viewer->setCameraPosition(25.6292, -183.596, 16.0492, -3.02173, -0.968898, -27.6203, 0.00256659, 0.415112, 0.909767);
  // viewer->setCameraClipDistances(55.3839, 192.787);
  // viewer->setPosition(372, 208);
  // viewer->setSize(1600, 2000);

  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void visualizer::addSquare(pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointCloud<pcl::PointXYZ> rect, float z, double r, double g, double b, std::string stringid, float alpha)
{
  pcl::PointXYZ p1;
  p1.x=alpha*rect.at(0).x+(1-alpha)*rect.at(2).x;
  p1.y=alpha*rect.at(0).y+(1-alpha)*rect.at(2).y;
  p1.z=z;
  pcl::PointXYZ p2;
  p2.x=alpha*rect.at(1).x+(1-alpha)*rect.at(3).x;
  p2.y=alpha*rect.at(1).y+(1-alpha)*rect.at(3).y;
  p2.z=z;
  pcl::PointXYZ p3;
  p3.x=alpha*rect.at(2).x+(1-alpha)*rect.at(0).x;
  p3.y=alpha*rect.at(2).y+(1-alpha)*rect.at(0).y;
  p3.z=z;
  pcl::PointXYZ p4;
  p4.x=alpha*rect.at(3).x+(1-alpha)*rect.at(1).x;
  p4.y=alpha*rect.at(3).y+(1-alpha)*rect.at(1).y;
  p4.z=z;
  viewer->addLine (p1, p2, r,g,b,stringid+std::to_string(0));
  viewer->addLine (p2, p3, r,g,b,stringid+std::to_string(1));
  viewer->addLine (p3, p4, r,g,b,stringid+std::to_string(2));
  viewer->addLine (p4, p1, r,g,b,stringid+std::to_string(3));
}

void visualizer::addQuadtreeAsMesh(pcl::visualization::PCLVisualizer::Ptr viewer, float z, double r, double g, double b, std::string stringid, float alpha)
{
  pcl::PolygonMesh mesh;
  pcl::PointCloud<pcl::PointXYZ> polygonsPts;
  std::vector<pcl::Vertices> polygonIds;

  std::deque<quadLeaf*> listLeaf = functionalRBFModel.getTree()->getListLeaf();
  for(int i=0;i<listLeaf.size();i++){
    pcl::PointCloud<pcl::PointXYZ> rect = listLeaf.at(i)->getQuadrant().getRect();

    pcl::PointXYZ p1;
    p1.x=alpha*rect.at(0).x+(1-alpha)*rect.at(2).x;
    p1.y=alpha*rect.at(0).y+(1-alpha)*rect.at(2).y;
    p1.z=z;
    pcl::PointXYZ p2;
    p2.x=alpha*rect.at(1).x+(1-alpha)*rect.at(3).x;
    p2.y=alpha*rect.at(1).y+(1-alpha)*rect.at(3).y;
    p2.z=z;
    pcl::PointXYZ p3;
    p3.x=alpha*rect.at(2).x+(1-alpha)*rect.at(0).x;
    p3.y=alpha*rect.at(2).y+(1-alpha)*rect.at(0).y;
    p3.z=z;
    pcl::PointXYZ p4;
    p4.x=alpha*rect.at(3).x+(1-alpha)*rect.at(1).x;
    p4.y=alpha*rect.at(3).y+(1-alpha)*rect.at(1).y;
    p4.z=z;

    polygonsPts.push_back(p1);
    polygonsPts.push_back(p2);
    polygonsPts.push_back(p3);
    polygonsPts.push_back(p4);

    pcl::Vertices faces1;
    faces1.vertices.push_back(4*i+0);
    faces1.vertices.push_back(4*i+1);
    faces1.vertices.push_back(4*i+2);
    polygonIds.push_back(faces1);
    pcl::Vertices faces2;
    faces2.vertices.push_back(4*i+2);
    faces2.vertices.push_back(4*i+3);
    faces2.vertices.push_back(4*i+0);
    polygonIds.push_back(faces2);
  }

  pcl::toPCLPointCloud2 (polygonsPts, mesh.cloud);
  mesh.polygons = polygonIds;

  viewer->addPolylineFromPolygonMesh(mesh,"Polymesh");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "Polymesh");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.3, 0.3, 0.3, "Polymesh");
}

void visualizer::addSurfaceMesh(pcl::visualization::PCLVisualizer::Ptr viewer, float deltaZ=0.)
{
  pcl::PolygonMesh mesh;
  pcl::PointCloud<pcl::PointXYZ> polygonsPts;
  std::vector<pcl::Vertices> polygonIds;
  const isoSurface iso = functionalRBFModel.getLevelSet();
  //std::ofstream fileStream("mesh.xyz", std::ios::out | std::ios::trunc);
  for(std::size_t i=0; i<iso.getTriangles().size();i++)
  {
      pcl::PointXYZ p1 = iso.getMapPoints().at(iso.getTriangles().at(i).pointIndex[0]);
      pcl::PointXYZ p2 = iso.getMapPoints().at(iso.getTriangles().at(i).pointIndex[1]);
      pcl::PointXYZ p3 = iso.getMapPoints().at(iso.getTriangles().at(i).pointIndex[2]);
      //fileStream<<p1.x<<" "<<p1.y<<" "<<p1.z<<std::endl;
      //fileStream<<p2.x<<" "<<p2.y<<" "<<p2.z<<std::endl;
      //fileStream<<p3.x<<" "<<p3.y<<" "<<p3.z<<std::endl;
      p1.z=p1.z+deltaZ;
      p2.z=p2.z+deltaZ;
      p3.z=p3.z+deltaZ;
      polygonsPts.push_back(p1);
      polygonsPts.push_back(p2);
      polygonsPts.push_back(p3);
      pcl::Vertices face;
      face.vertices.push_back(3*i+0);
      face.vertices.push_back(3*i+1);
      face.vertices.push_back(3*i+2);
      polygonIds.push_back(face);
  }
  pcl::toPCLPointCloud2 (polygonsPts, mesh.cloud);
  mesh.polygons = polygonIds;
  //pcl::io::savePLYFileBinary ("surface.ply", mesh);
  viewer->addPolygonMesh(mesh,"surfaceMesh");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.95, "surfaceMesh");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 0.95, "surfaceMesh");
}

void visualizer::addMinPointsConcaveHull(pcl::visualization::PCLVisualizer::Ptr viewer, float z)
{
  pcl::PolygonMesh mesh;
  pcl::PointCloud<pcl::PointXYZI> polygonsPts;
  std::vector<pcl::Vertices> polygonIds;

  pcl::PointCloud<pcl::PointXYZI> minPts = pointsMin;
  for(int i = 0; i < minPts.size(); i++)
        minPts.at(i).z = z;
  pcl::ConcaveHull<pcl::PointXYZI> chull;
  chull.setInputCloud (minPts.makeShared());
  chull.setAlpha (0.5);
  chull.reconstruct (polygonsPts, polygonIds);

  pcl::toPCLPointCloud2 (polygonsPts, mesh.cloud);
  mesh.polygons = polygonIds;
  viewer->addPolylineFromPolygonMesh(mesh,"ConcaveHull");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, "ConcaveHull");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.7, 0.8, 0.4, "ConcaveHull");
}
