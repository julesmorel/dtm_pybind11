#include "surfaceCsrbf.h"
#include "wendlandRbf.h"
#include "quadleafCsrbf.h"

#include "../utils/surfaceFRBFUtils.h"
#include "../utils/surfaceCSRBFUtils.h"

#include "../utils/progress_bar.h"

#include <fstream>

#include <boost/thread.hpp>

surfaceCsrbf::surfaceCsrbf(surfaceFrbf& psurface, pcl::PointCloud<pcl::PointXYZI> &pCloud, float refinementValue)
{
    progress_bar progress{std::clog, 70u, "    Transforming PU model into CSRBF model"};

    for(int i=0;i<psurface.getTree()->getListLeaf().size();i++)
    {
          float numLeaf = (float)psurface.getTree()->getListLeaf().size();
          progress.write((float)i/numLeaf);

          quadleafCsrbf leafCsrbf(i,psurface,refinementValue, pCloud);
          if(leafCsrbf.getCloudCsrbf().size()>0){
            listLeafs.push_back(leafCsrbf);
            cloudCsrbf.insert(cloudCsrbf.end(), listLeafs.at(i).getCloudCsrbf().begin(), listLeafs.at(i).getCloudCsrbf().end());
          }
    }

  //creating an octree for neighborhood search
  octreeCsrbf  = new pcl::octree::OctreePointCloudSearch<csrbfPCL>(0.1);
  octreeCsrbf->setInputCloud (cloudCsrbf.makeShared());
  octreeCsrbf->addPointsFromInputCloud();
}

void surfaceCsrbf::updateListCsrbf()
{
  cloudCsrbf.clear();
  for(int i=0;i<listLeafs.size();i++)
  {
        cloudCsrbf.insert(cloudCsrbf.end(), listLeafs.at(i).getCloudCsrbf().begin(), listLeafs.at(i).getCloudCsrbf().end());
  }
}

void surfaceCsrbf::printCSRBF(std::string filename)
{
  std::ofstream fileStream(filename.c_str(), std::ios::out | std::ios::trunc);
  for(int k=0;k<cloudCsrbf.size();k++){
    csrbfPCL c = cloudCsrbf.at(k);
    fileStream<<c.x<<" "<<c.y<<" "<<c.z<<" "<<c.radius<<" "<<c.alpha<<std::endl;
  }
  fileStream.close();
}

void surfaceCsrbf::fillScalarField(scalarField &field)
{
  pcl::PointXYZ min = field.getRectangle().getP0();

  int nbDivX = field.getXNode();
  int nbDivY = field.getYNode();
  int nbDivZ = field.getZNode();

  float sizeCellX = field.getSizeCellX();
  float sizeCellY = field.getSizeCellY();
  float sizeCellZ = field.getSizeCellZ();

  for(std::size_t m=0;m<cloudCsrbf.size();m++)
  {
    float dx,dy,dz;
    int coordCenterX, coordCenterY, coordCenterZ;
    pcl::PointXYZ c;
    c.x = cloudCsrbf.at(m).x;
    c.y = cloudCsrbf.at(m).y;
    c.z = cloudCsrbf.at(m).z;
    field.getRectangle().getRatioWithPoint(c, dx, dy);
    dz = c.z-field.getZmin();
    coordCenterX = (int)(dx/sizeCellX);
    coordCenterY = (int)(dy/sizeCellY);
    coordCenterZ = (int)(dz/sizeCellZ);
    int stepX = (int)(cloudCsrbf.at(m).radius/sizeCellX)+1;
    int stepY = (int)(cloudCsrbf.at(m).radius/sizeCellY)+1;
    int stepZ = (int)(cloudCsrbf.at(m).radius/sizeCellZ)+1;

    for(int k=coordCenterZ-stepZ;k<coordCenterZ+stepZ;k++)
    {
      for(int j=coordCenterY-stepY;j<coordCenterY+stepY;j++)
      {
        for(int i=coordCenterX-stepX;i<coordCenterX+stepX;i++)
        {
          if(i>=0 && i<nbDivX+1 && j>=0 && j<nbDivY+1 && k>=0 && k<nbDivZ+1){

            pcl::PointXYZ p = field.getPointFromIndex(i,j,k);

            double sum = 0.;
            if(field.getValue(i,j,k) != scalarField::NODATA)
            {
              sum = field.getValue(i,j,k);
            }

            bool hasBeenUpdated = false;

            float tmpx = (cloudCsrbf.at(m).x-p.x);
            float tmpy = (cloudCsrbf.at(m).y-p.y);
            float tmpz = (cloudCsrbf.at(m).z-p.z);
            float distR = sqrt(tmpx*tmpx+tmpy*tmpy+tmpz*tmpz);

            double r = (double)distR/(double)cloudCsrbf.at(m).radius;

            if(r<1.)
            {
              double res = cloudCsrbf.at(m).alpha;
              //double sumAtp=getSumRbfAtPoint(p);
              //std::cout<<sumAtp<<std::endl;
              double val = (res)*wendlandRbf::getValueRbfWendland(r);///sumAtp;

              if(val!=0.0){
                sum += val;
                hasBeenUpdated = true;
              }

            }

            if(hasBeenUpdated)field.setValue(i,j,k,sum);
          }
        }
      }
    }
  }
}
