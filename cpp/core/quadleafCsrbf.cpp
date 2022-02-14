#include "quadleafCsrbf.h"
#include "wendlandRbf.h"

#include "../utils/surfaceFRBFUtils.h"
#include "../utils/surfaceCSRBFUtils.h"

#include <fstream>

quadleafCsrbf::quadleafCsrbf(size_t id, surfaceFrbf& psurface,float resolution, pcl::PointCloud<pcl::PointXYZI> &pCloud):indexLeaf(id)
{
  buildListCsrbf(psurface,resolution, 3.0);
  //std::cout<<id<< " -> "<<cloudCsrbf.size()<<" centers ... "<<std::flush;
  decimateListCsrbf(pCloud,0.3);
  //std::cout<<cloudCsrbf.size()<<" centers after decimation "<<std::endl;

  //creating an octree for neighborhood search
  octreeCsrbf  = new pcl::octree::OctreePointCloudSearch<csrbfPCL>(0.1);
  octreeCsrbf->setInputCloud (cloudCsrbf.makeShared());
  octreeCsrbf->addPointsFromInputCloud();

  int nbColloc = cloudCsrbf.size();
  HSparse.resize(nbColloc,nbColloc);

  fillH(HSparse);

  //std::cout<<"H filled"<<std::endl;

  S.resize(nbColloc);
  S.setZero();
  fillS(S);

  //std::cout<<"S fillled"<<std::endl;

  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double> > ldlt;
  ldlt.compute(HSparse);

  if(ldlt.info()!=Eigen::Success) {
    std::cout<<"LDLT dense failed!"<<std::endl;
  }

  alpha = ldlt.solve(S);
  if(ldlt.info()!=Eigen::Success)std::cout<<"System not solved!"<<std::endl;

  for(int i=0;i<cloudCsrbf.size();i++)
  {
    cloudCsrbf.at(i).alpha = alpha(i);
  }
}

/*void quadleafCsrbf::solveSystem()
{
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double> > ldlt;
  ldlt.compute(HSparse);

  if(ldlt.info()!=Eigen::Success) {
    std::cout<<"LDLT dense failed!"<<std::endl;
  }

  alpha = ldlt.solve(S);
  if(ldlt.info()!=Eigen::Success)std::cout<<"System not solved!"<<std::endl;

  for(int i=0;i<cloudCsrbf.size();i++)
  {
    cloudCsrbf.at(i).alpha = alpha(i);
  }
}*/

void quadleafCsrbf::buildListCsrbf(surfaceFrbf& psurface, float resolution, float valueThreshold)
{


  pcl::PointXYZI BBMin, BBMax;
  pcl::getMinMax3D (psurface.getTree()->getListLeaf().at(indexLeaf)->getLocalCell()->getGlobalPts(), BBMin, BBMax);
  float zmin = BBMin.z-0.5;
  float zmax = BBMax.z+0.5;

  double radius = resolution*sqrt(3)*1.4;
  float lx=psurface.getTree()->getListLeaf().at(indexLeaf)->getQuadrant().getSizeXSide();
  int nbDivX = (int)(lx/resolution)+1;
  float ly=psurface.getTree()->getListLeaf().at(indexLeaf)->getQuadrant().getSizeYSide();
  int nbDivY = (int)(ly/resolution)+1;
  float lz=(int)((zmax-zmin)/resolution)+1;
  int nbDivZ = (int)(lz/resolution)+1;
  for(int i=0;i<nbDivX;i++)
  {
    for(int j=0;j<nbDivY;j++)
    {
      float dx = (float)i/(float)nbDivX;
      float dy = (float)j/(float)nbDivY;
      pcl::PointXYZ p = psurface.getTree()->getListLeaf().at(indexLeaf)->getQuadrant().getPointWithRatio(dx,dy);
      for(int k=0;k<nbDivZ;k++)
      {
          p.z = zmin+k*resolution;
          addCsrbfToList(p,radius,psurface,valueThreshold);
      }
    }
  }
}

void quadleafCsrbf::decimateListCsrbf(pcl::PointCloud<pcl::PointXYZI> &pCloud, float valueThreshold)
{
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> kdtreeCsrbf(0.1);
  kdtreeCsrbf.setInputCloud(pCloud.makeShared());
  kdtreeCsrbf.addPointsFromInputCloud();

  std::vector<int> listIndex_to_delete;

  for(int k=0;k<cloudCsrbf.size();k++){

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    pcl::PointXYZI p;
    p.x=cloudCsrbf.at(k).x;
    p.y=cloudCsrbf.at(k).y;
    p.z=cloudCsrbf.at(k).z;

    if ( kdtreeCsrbf.radiusSearch (p, 1.5*cloudCsrbf.at(k).radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) == 0 )
    {
      float tmpx = (cloudCsrbf.at(k).x-cloudCsrbf.at(k).projSurface_x);
      float tmpy = (cloudCsrbf.at(k).y-cloudCsrbf.at(k).projSurface_y);
      float tmpz = (cloudCsrbf.at(k).z-cloudCsrbf.at(k).projSurface_z);
      float distance = sqrt(tmpx*tmpx+tmpy*tmpy+tmpz*tmpz);
      if(distance>valueThreshold){
        listIndex_to_delete.push_back(k);
      }
      /*std::ofstream outfile;
      outfile.open("log/removed.xyz", std::ios_base::app);
      outfile<<p.x<<" "<<p.y<<" "<<p.z<<" "<<distance<<std::endl;*/
    }
  }

  pcl::PointCloud<csrbfPCL> cloudCsrbfCpy;
  cloudCsrbfCpy = cloudCsrbf;
  cloudCsrbf.clear();
  for(int i=0;i<cloudCsrbfCpy.size();i++)
  {
    auto it = std::find(listIndex_to_delete.begin(), listIndex_to_delete.end(), i);
    if(it == listIndex_to_delete.end())
    {
        cloudCsrbf.push_back(cloudCsrbfCpy.at(i));
    }
  }
  cloudCsrbfCpy.clear();
}

void quadleafCsrbf::addCsrbfToList(pcl::PointXYZ pCenter, float radius, surfaceFrbf& psurface, double threshold)
{
  double alpha=0.;
  pcl::Normal nor;
  bool isupdated = surfaceFRBFUtils::computeImplicitFctValueAndPseudoGrad(pCenter,alpha,nor,psurface);

  //check the distance to the surface
  if(isupdated)
  {
    //get the projection of each collocation on the surface
    pcl::PointXYZ proj = surfaceFRBFUtils::getPointOnSurfaceNewton(pCenter,nor,psurface);
    float tmpx = (pCenter.x-proj.x);
    float tmpy = (pCenter.y-proj.y);
    float tmpz = (pCenter.z-proj.z);
    float distance = sqrt(tmpx*tmpx+tmpy*tmpy+tmpz*tmpz);

    if(distance<threshold)
    {
      csrbfPCL center;
      center.x = pCenter.x;
      center.y = pCenter.y;
      center.z = pCenter.z;
      center.alpha = (float)alpha;
      center.radius = radius;
      center.gradFRBF_x = nor.normal_x;
      center.gradFRBF_y = nor.normal_y;
      center.gradFRBF_z = nor.normal_z;
      center.projSurface_x = proj.x;
      center.projSurface_y = proj.y;
      center.projSurface_z = proj.z;
      std::ofstream outfile;
      outfile.open("log/projection.xyz", std::ios_base::app);
      outfile<<proj.x<<" "<<proj.y<<" "<<proj.z<<std::endl;
      cloudCsrbf.push_back(center);
    }
  }
}

void quadleafCsrbf::fillH(Eigen::SparseMatrix<double>& HSparse)
{
  int nbColloc = cloudCsrbf.size();

  std::vector<Eigen::Triplet<float> > tripletList;
  tripletList.reserve(0.2*nbColloc*nbColloc);

  pcl::octree::OctreePointCloudSearch<csrbfPCL> * kdtreeCsrbf = new pcl::octree::OctreePointCloudSearch<csrbfPCL>(0.1);
  kdtreeCsrbf->setInputCloud(cloudCsrbf.makeShared());
  kdtreeCsrbf->addPointsFromInputCloud();

  for(int k=0;k<nbColloc;k++){

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    if ( kdtreeCsrbf->radiusSearch (cloudCsrbf.at(k), cloudCsrbf.at(k).radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
    {
      for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
        float rbfval = wendlandRbf::getValueRbfWendland(sqrt(pointRadiusSquaredDistance[i])/cloudCsrbf.at(k).radius);
        tripletList.push_back(Eigen::Triplet<float>(k,pointIdxRadiusSearch[i],rbfval));
      }
    }
    tripletList.push_back(Eigen::Triplet<float>(k,k,1.));
  }
  HSparse.setFromTriplets(tripletList.begin(), tripletList.end());
}
  /*for(int i=0;i<nbColloc;i++){
    for(int j=0;j<nbColloc;j++){
              csrbfPCL ci = cloudCsrbf.at(i);
              csrbfPCL cj = cloudCsrbf.at(j);
              float dx = ci.x-cj.x;
              float dy = ci.y-cj.y;
              float dz = ci.z-cj.z;
              float d = sqrt(dx*dx+dy*dy+dz*dz);
              float rbfval = wendlandRbf::getValueRbfWendland(d/ci.radius);
              HSparse.insert(i,j) = rbfval;
    }
  }
}

/*void quadleafCsrbf::computeH(pcl::PointCloud<csrbfPCL>& fullCloud)
{
  int nbColloc = cloudCsrbf.size();

  std::vector<Eigen::Triplet<float> > tripletList;
  tripletList.reserve(0.2*nbColloc*nbColloc);

  pcl::octree::OctreePointCloudSearch<csrbfPCL> * kdtreeCsrbf = new pcl::octree::OctreePointCloudSearch<csrbfPCL>(0.1);
  kdtreeCsrbf->setInputCloud(fullCloud.makeShared());
  kdtreeCsrbf->addPointsFromInputCloud();

  for(int k=0;k<nbColloc;k++){

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    if ( kdtreeCsrbf->radiusSearch (cloudCsrbf.at(k), cloudCsrbf.at(k).radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
    {
      for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
        float rbfval = wendlandRbf::getValueRbfWendland(sqrt(pointRadiusSquaredDistance[i])/cloudCsrbf.at(k).radius);
        tripletList.push_back(Eigen::Triplet<float>(k,pointIdxRadiusSearch[i],rbfval));
      }
    }
    tripletList.push_back(Eigen::Triplet<float>(k,k,1.));
  }
  HSparse.setFromTriplets(tripletList.begin(), tripletList.end());
}*/

void quadleafCsrbf::fillS(Eigen::VectorXd& S)
{
  int nbColloc = cloudCsrbf.size();

  for(int j=0;j<nbColloc;j++)
  {
    S(j)=cloudCsrbf.at(j).alpha;
  }
}
