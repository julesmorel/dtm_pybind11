#include "visualizer.h"
#include "core/quadleaf.h"
#include <iostream>
#include <thread>
#include <pcl/PCLPointCloud2.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/io/ply_io.h>
#include <sstream>

visualizer::visualizer(pcl::PointCloud<pcl::PointXYZI> _pointsMin, pcl::PointCloud<pcl::PointXYZ> _pointsRectangleMin, surfaceFrbf _functionalRBFModel, surfaceCsrbf _CSRBFModel, bool _deformableModelApplied)
{
  pointsMin=_pointsMin;
  pointsRectangleMin=_pointsRectangleMin;
  functionalRBFModel=_functionalRBFModel;
  CSRBFModel=_CSRBFModel;
  deformableModelApplied=_deformableModelApplied;
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
    uint32_t rgb = (static_cast<uint32_t>(static_cast<float>(r*it)) << 16 | static_cast<uint32_t>(static_cast<float>(g*it)) << 8 | static_cast<uint32_t>(static_cast<float>(b*it)));
    prgb.rgb = *reinterpret_cast<float*>(&rgb);
    ptsrgb.push_back(prgb);
  }
  return ptsrgb;
}

void visualizer::plot()
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

  float deltaZ=5.0;

  viewer->setBackgroundColor (0.9, 0.9, 0.9);

  pcl::PointCloud<pcl::PointXYZRGB> minRGB = addColorToPointCloudwithI(pointsMin,200,250,200,0.);

  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr minptr = minRGB.makeShared();
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_min(minptr);
  viewer->addPointCloud<pcl::PointXYZRGB> (minptr, rgb_min, "min");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "min");

  // //plot the quadtree
  if(!deformableModelApplied){
    addSquare(viewer,pointsRectangleMin,lowestPtsZ-2*deltaZ,0.9,0.9,0.4,"bbs",1.0);
    addQuadtreeAsMesh(viewer,lowestPtsZ-2*deltaZ,0.5,0.5,0.5,"quatree",0.98);
    //addBorderCells(viewer,lowestPtsZ-2.1*deltaZ,0.5,0.5,0.5,"borders",0.98);
    addSquare(viewer,pointsRectangleMin,lowestPtsZ-1*deltaZ,0.7,0.7,0.9,"bbsm",1.0);
    addQuadsSurfaceMesh(viewer,-1*deltaZ);
    addSurfaceMesh(viewer,functionalRBFModel.getLevelSet(),0.,0.5, 0.5, 0.95, "functionalRBFModel");
  }else{
    addSquare(viewer,pointsRectangleMin,lowestPtsZ-3*deltaZ,0.9,0.9,0.4,"bbs",1.0);
    addQuadtreeAsMesh(viewer,lowestPtsZ-3*deltaZ,0.5,0.5,0.5,"quatree",0.98);
    //addBorderCells(viewer,lowestPtsZ-3.1*deltaZ,0.5,0.5,0.5,"borders",0.98);
    addSquare(viewer,pointsRectangleMin,lowestPtsZ-2*deltaZ,0.7,0.7,0.9,"bbsm",1.0);
    addQuadsSurfaceMesh(viewer,-2*deltaZ);
    addSurfaceMesh(viewer,functionalRBFModel.getLevelSet(),-1*deltaZ,0.5, 0.5, 0.95, "functionalRBFModel");
    addSurfaceMesh(viewer,CSRBFModel.getLevelSet(),0.,0.5, 0.8, 0.9, "CSRBFModel");
  }

  viewer->initCameraParameters ();

  viewer->setCameraPosition(48.8185, -53.4724, 14.2673, 0.0441494, 0.2745, -5.89377, -0.173229, 0.20421, 0.963478);
  viewer->setCameraClipDistances(10.3839, 192.787);
  viewer->setPosition(20, 200);

   viewer->setSize(2000, 2000);

  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void visualizer::addSquare(pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointCloud<pcl::PointXYZ> &rect, float z, double r, double g, double b, std::string stringid, float alpha)
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

void visualizer::addBorderCells(pcl::visualization::PCLVisualizer::Ptr viewer, float z, double r, double g, double b, std::string stringid, float alpha)
{
  pcl::PolygonMesh mesh;
  pcl::PointCloud<pcl::PointXYZ> polygonsPts;
  std::vector<pcl::Vertices> polygonIds;

  std::deque<quadLeaf*> listLeaf = functionalRBFModel.getTree()->getListLeaf();
  for(int i=0;i<listLeaf.size();i++){
    quadLeaf* leaf = listLeaf.at(i);
    pcl::PointCloud<pcl::PointXYZ> rect = leaf->getQuadrant().getRect();

    if(leaf->isOnBorder())
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

      polygonsPts.push_back(p1);
      polygonsPts.push_back(p2);
      polygonsPts.push_back(p3);
      polygonsPts.push_back(p4);

      pcl::Vertices faces1;
      faces1.vertices.push_back(4*i+0);
      faces1.vertices.push_back(4*i+1);
      faces1.vertices.push_back(4*i+2);

      pcl::Vertices faces2;
      faces2.vertices.push_back(4*i+2);
      faces2.vertices.push_back(4*i+3);
      faces2.vertices.push_back(4*i+0);

      polygonIds.push_back(faces1);
      polygonIds.push_back(faces2);

      double orientation[]={90.,0.,0.};

      std::stringstream stream;
      stream << leaf->getLocationCode();
      std::string str;
      stream >> str;
      std::string text=str.substr(str.size() - 8);
      viewer->addText3D (text,p1,orientation,0.3,0.3,0.3,0.3,text);
    }
  }

  pcl::toPCLPointCloud2 (polygonsPts, mesh.cloud);
  mesh.polygons = polygonIds;

  //viewer->addPolygonMesh(mesh,"borders");
  //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "borders");
  //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.3, 0.3, 0.3, "borders");
}

void visualizer::addSurfaceMesh(pcl::visualization::PCLVisualizer::Ptr viewer, const isoSurface & surface, float deltaZ, double r, double g, double b, std::string stringid)
{
  pcl::PolygonMesh mesh;
  pcl::PointCloud<pcl::PointXYZ> polygonsPts;
  std::vector<pcl::Vertices> polygonIds;
  const isoSurface iso = surface;
  for(std::size_t i=0; i<iso.getTriangles().size();i++)
  {
      pcl::PointXYZ p1 = iso.getMapPoints().at(iso.getTriangles().at(i).pointIndex[0]);
      pcl::PointXYZ p2 = iso.getMapPoints().at(iso.getTriangles().at(i).pointIndex[1]);
      pcl::PointXYZ p3 = iso.getMapPoints().at(iso.getTriangles().at(i).pointIndex[2]);
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
  viewer->addPolygonMesh(mesh,stringid);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.85, stringid);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r,g,b,stringid);
}

void visualizer::addQuadsSurfaceMesh(pcl::visualization::PCLVisualizer::Ptr viewer, float z)
{
  pcl::PolygonMesh mesh;
  pcl::PointCloud<pcl::PointXYZ> polygonsPts;
  std::vector<pcl::Vertices> polygonIds;
  std::deque<quadLeaf*> listLeaf = functionalRBFModel.getTree()->getListLeaf();
  int idt=0;
  for(std::size_t j=0;j<listLeaf.size();j++)
  {if(!listLeaf.at(j)->isEmpty()){
      for(std::size_t i=0; i<listLeaf.at(j)->getLevelSet().getTriangles().size();i++)
      {
          pcl::PointXYZ p1 = listLeaf.at(j)->getLevelSet().getMapPoints().at(listLeaf.at(j)->getLevelSet().getTriangles().at(i).pointIndex[0]);
          pcl::PointXYZ p2 = listLeaf.at(j)->getLevelSet().getMapPoints().at(listLeaf.at(j)->getLevelSet().getTriangles().at(i).pointIndex[1]);
          pcl::PointXYZ p3 = listLeaf.at(j)->getLevelSet().getMapPoints().at(listLeaf.at(j)->getLevelSet().getTriangles().at(i).pointIndex[2]);
          p1.z=p1.z+z;
          p2.z=p2.z+z;
          p3.z=p3.z+z;
          polygonsPts.push_back(p1);
          polygonsPts.push_back(p2);
          polygonsPts.push_back(p3);
          pcl::Vertices face;
          face.vertices.push_back(3*idt+0);
          face.vertices.push_back(3*idt+1);
          face.vertices.push_back(3*idt+2);
          polygonIds.push_back(face);

          idt++;
      }
  }}
  pcl::toPCLPointCloud2 (polygonsPts, mesh.cloud);
  mesh.polygons = polygonIds;
  viewer->addPolygonMesh(mesh,"quadsSurfaceMesh");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.7, "quadsSurfaceMesh");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.95, 0.5, "quadsSurfaceMesh");
}
