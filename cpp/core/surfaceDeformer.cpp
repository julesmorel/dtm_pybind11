#include "surfaceDeformer.h"
#include "multidimensionalminimizer.h"

#include "../utils/surfaceCSRBFUtils.h"
#include "../utils/progress_bar.h"

#include <fstream>

#include <boost/thread.hpp>

#include<numeric>

surfaceDeformer::surfaceDeformer(surfaceCsrbf& psurface, pcl::PointCloud<pcl::PointXYZI>& dataPoint, float pratio):ratio(pratio)
{
  surface=psurface;
}

void surfaceDeformer::computeDeformations(pcl::PointCloud<pcl::PointXYZI>& dataPoint)
{
  //compute the deformation vector at each collocation
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZI>* octree = new pcl::octree::OctreePointCloudSearch<pcl::PointXYZI>(0.1);
  octree->setInputCloud (dataPoint.makeShared());
  octree->addPointsFromInputCloud ();

  for(int index=0;index<surface.getListLeafs().size();index++){

    for(int m=0;m<surface.getListLeafs().at(index).getCloudCsrbf().size();m++){

      csrbfPCL c = surface.getListLeafs().at(index).getCloudCsrbf().at(m);

      pcl::PointCloud<pcl::PointXYZI> neighbors;
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;

      pcl::PointXYZI center;
      center.x=c.x;
      center.y=c.y;
      center.z=c.z;

      if ( octree->radiusSearch (center, c.radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
      {
        for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
          pcl::PointXYZI pt;
          pt.x = dataPoint.at(pointIdxRadiusSearch[i]).x;
          pt.y = dataPoint.at(pointIdxRadiusSearch[i]).y;
          pt.z = dataPoint.at(pointIdxRadiusSearch[i]).z;
          pt.intensity = dataPoint.at(pointIdxRadiusSearch[i]).intensity;
          neighbors.push_back(pt);

          std::ofstream outfile;
          outfile.open("log/neighbors.xyz", std::ios_base::app);
          outfile<<center.x<<" "<<center.y<<" "<<center.z<<" "<<neighbors.size()<<std::endl;
        }
      }
      if(neighbors.size()==0){
        surface.getListLeafs().at(index).getCloudCsrbf().at(m).vectDef_x =0;//cloudCsrbf.at(m).gradFRBF_x;
        surface.getListLeafs().at(index).getCloudCsrbf().at(m).vectDef_y =0;//cloudCsrbf.at(m).gradFRBF_y;
        surface.getListLeafs().at(index).getCloudCsrbf().at(m).vectDef_z =0;//cloudCsrbf.at(m).gradFRBF_z;
        std::ofstream outfile;
        outfile.open("log/neighbors.xyz", std::ios_base::app);
        outfile<<center.x<<" "<<center.y<<" "<<center.z<<" "<<neighbors.size()<<std::endl;
      }else{

        pcl::PointXYZ p;
        p.x=c.x;
        p.y=c.y;
        p.z=c.z;
        /*pcl::Normal nprime = surfaceCSRBFUtils::getValueImplicitFctGrad(p,surface,c.radius);
        float norm=sqrt(nprime.normal_x*nprime.normal_x+nprime.normal_y*nprime.normal_y+nprime.normal_z*nprime.normal_z);
        nprime.normal_x=nprime.normal_x/norm;
        nprime.normal_y=nprime.normal_y/norm;
        nprime.normal_z=nprime.normal_z/norm;*/

        float norm=sqrt(c.gradFRBF_x*c.gradFRBF_x+c.gradFRBF_y*c.gradFRBF_y+c.gradFRBF_z*c.gradFRBF_z);
        pcl::Normal nprime;
        nprime.normal_x=c.gradFRBF_x/norm;
        nprime.normal_y=c.gradFRBF_y/norm;
        nprime.normal_z=c.gradFRBF_z/norm;

        pcl::PointXYZ proj;
        proj.x=c.projSurface_x;
        proj.y=c.projSurface_y;
        proj.z=c.projSurface_z;

        multiDimensionalMinimizer minimizer(neighbors, p,proj,nprime, c.radius,ratio);
        float t = minimizer.getT();
        pcl::Normal n = minimizer.getN();

        pcl::Normal defVector;
        defVector.normal_x = -t*n.normal_x;
        defVector.normal_y = -t*n.normal_y;
        defVector.normal_z = -t*n.normal_z;

        surface.getListLeafs().at(index).getCloudCsrbf().at(m).vectDef_x = defVector.normal_x;
        surface.getListLeafs().at(index).getCloudCsrbf().at(m).vectDef_y = defVector.normal_y;
        surface.getListLeafs().at(index).getCloudCsrbf().at(m).vectDef_z = defVector.normal_z;

        std::ofstream outfile;
        outfile.open("log/deformation.xyz", std::ios_base::app);
        outfile<<c.x<<" "<<c.y<<" "<<c.z<<" "<<surface.getListLeafs().at(index).getCloudCsrbf().at(m).vectDef_x<<" "<<surface.getListLeafs().at(index).getCloudCsrbf().at(m).vectDef_y<<" "<<surface.getListLeafs().at(index).getCloudCsrbf().at(m).vectDef_z<<std::endl;
      }
    }
  }
}

Eigen::VectorXd surfaceDeformer::fillV(size_t index , surfaceCsrbf& surface)
{
    Eigen::VectorXd V;
    int nbColloc = surface.getListLeafs().at(index).getCloudCsrbf().size();
    V.resize(nbColloc);

    for(int i=0;i<nbColloc;i++)
    {
        csrbfPCL c = surface.getListLeafs().at(index).getCloudCsrbf().at(i);
        pcl::Normal v;
        v.normal_x = c.vectDef_x;
        v.normal_y = c.vectDef_y;
        v.normal_z = c.vectDef_z;

        pcl::PointXYZ x;
        x.x=c.x;
        x.y=c.y;
        x.z=c.z;
        pcl::Normal grad = surfaceCSRBFUtils::getValueImplicitFctGrad(x,surface,surface.getListLeafs().at(index).getCloudCsrbf().at(i).radius);
        float norm=sqrt(grad.normal_x*grad.normal_x+grad.normal_y*grad.normal_y+grad.normal_z*grad.normal_z);
        /*grad.normal_x=grad.normal_x/norm;
        grad.normal_y=grad.normal_y/norm;
        grad.normal_z=grad.normal_z/norm;*/

        V(i)=v.normal_x*grad.normal_x+v.normal_y*grad.normal_y+v.normal_z*grad.normal_z;

        std::ofstream outfile;
        outfile.open("log/V.xyz", std::ios_base::app);
        outfile<<c.x<<" "<<c.y<<" "<<c.z<<" "<<V(i)<<" "<<norm<<std::endl;
    }
    return V;
}

Eigen::VectorXd surfaceDeformer::fillDirac(size_t index , surfaceCsrbf& surface)
{
    Eigen::VectorXd dirac;
    int nbColloc = surface.getListLeafs().at(index).getCloudCsrbf().size();
    dirac.resize(nbColloc);

    for(int i=0;i<nbColloc;i++)
    {
        double eps = surface.getCloudCsrbf().at(i).radius*2.;
        pcl::PointXYZ p;
        p.x = surface.getListLeafs().at(index).getCloudCsrbf().at(i).x;
        p.y = surface.getListLeafs().at(index).getCloudCsrbf().at(i).y;
        p.z = surface.getListLeafs().at(index).getCloudCsrbf().at(i).z;
        double val = surfaceCSRBFUtils::getValueImplicitFct(p,surface,surface.getCloudCsrbf().at(i).radius);
        //dirac(i) = 1./(M_PI*eps*(1.+((val/eps)*(val/eps))));
        dirac(i) = 1./(1.+500*val*val);

        std::ofstream outfile;
        outfile.open("log/dirac.xyz", std::ios_base::app);
        outfile<<p.x<<" "<<p.y<<" "<<p.z<<" "<<dirac(i)<<std::endl;
    }
    return dirac;
}

Eigen::VectorXd surfaceDeformer::getAlphas(int index)
{
  Eigen::VectorXd a;
  int nbColloc = surface.getListLeafs().at(index).getCloudCsrbf().size();
  //std::cout<<"   ncconlloc "<<nbColloc<<std::endl;
  if(nbColloc>0){
  a.resize(nbColloc);
  for(int j=0;j<nbColloc;j++)
  {
    a(j)=surface.getListLeafs().at(index).getCloudCsrbf().at(j).alpha;
  }}
  return a;
}

void surfaceDeformer::normalizingAlphas(float threshold)
{
    std::vector<float> listMaxAlphasPerLeaf;
    for(int index=0;index<surface.getListLeafs().size();index++){
      Eigen::VectorXd a = getAlphas(index);
      float anorm1 = a.cwiseAbs().maxCoeff();
      //std::cout<<index<<" / "<<surface.getListLeafs().size()<<" : "<<anorm1<<std::endl;
      listMaxAlphasPerLeaf.push_back(anorm1);
    }
    //float norm1Alphas = std::accumulate(listMaxAlphasPerLeaf.begin(),listMaxAlphasPerLeaf.end(),0);
    float norm1Alphas = *max_element(listMaxAlphasPerLeaf.begin(), listMaxAlphasPerLeaf.end());
    //std::cout<<"norm1Alphas: "<<norm1Alphas<<std::endl;
    if(norm1Alphas>threshold)
    {
        float correctionFactor=threshold/norm1Alphas;
        for(int index=0;index<surface.getListLeafs().size();index++){
          for(int j=0;j<surface.getListLeafs().at(index).getCloudCsrbf().size();j++)
          {
            surface.getListLeafs().at(index).getCloudCsrbf().at(j).alpha = correctionFactor*surface.getListLeafs().at(index).getCloudCsrbf().at(j).alpha;
          }
        }
    }
}

void surfaceDeformer::deform(pcl::PointCloud<pcl::PointXYZI>& dataPoint)
{
  progress_bar progress{std::clog, 70u, "    Applying the deformation"};
  //std::cout<<"       Normalizing alphas ..."<<std::flush;
  normalizingAlphas(0.05);
  //std::cout<<" Computing deformation vectors ..."<<std::flush;
  computeDeformations(dataPoint);
  //std::cout<<" Applying deformations ..."<<std::flush;
  //clock_t t = clock();
  float sumNorm=0.;

  for(int index=0;index<surface.getListLeafs().size();index++){

    float numLeaf = (float)surface.getListLeafs().size();
    progress.write((float)index/numLeaf);

    int nbColloc = surface.getListLeafs().at(index).getCloudCsrbf().size();
    //apply the deformation
    Eigen::VectorXd V,dirac;
    dirac.resize(nbColloc);
    V.resize(nbColloc);

    double tau = 1.;

    dirac = fillDirac(index,surface);
    V = fillV(index,surface);

    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double> > ldltdeformation;
    ldltdeformation.compute(surface.getListLeafs().at(index).getHSparse());
    if(ldltdeformation.info()!=Eigen::Success) {
      std::cout<<"LDLT dense failed!"<<std::endl;
    }
    Eigen::VectorXd b = V.cwiseProduct(dirac);
    Eigen::VectorXd res = ldltdeformation.solve(b);
    sumNorm+=res.norm()/(float)nbColloc;
    Eigen::VectorXd alpha = getAlphas(index);
    alpha = alpha - tau*res;
    surface.getListLeafs().at(index).setAlpha(alpha);

    for(int j=0;j<surface.getListLeafs().at(index).getCloudCsrbf().size();j++)
    {
      //surface.getListLeafs().at(index).setAlpha(j,alpha(j));
      surface.getListLeafs().at(index).getCloudCsrbf().at(j).alphaUpdated = alpha(j);
    }
  }
  //std::cout<<"      Norm deformation : "<<sumNorm<<std::endl;
  /*t = clock() - t;
  std::cout<<" Solved in "<<((float)t)/CLOCKS_PER_SEC<<" s"<<std::endl;*/

  surface.getCloudCsrbf().clear();
  for(int index=0;index<surface.getListLeafs().size();index++){
    for(int j=0;j<surface.getListLeafs().at(index).getCloudCsrbf().size();j++)
    {
      surface.getListLeafs().at(index).getCloudCsrbf().at(j).alpha = surface.getListLeafs().at(index).getCloudCsrbf().at(j).alphaUpdated;
    }
    surface.getCloudCsrbf().insert(surface.getCloudCsrbf().end(), surface.getListLeafs().at(index).getCloudCsrbf().begin(), surface.getListLeafs().at(index).getCloudCsrbf().end());
  }
}
