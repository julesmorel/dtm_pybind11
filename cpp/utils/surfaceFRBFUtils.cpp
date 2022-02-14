#include "surfaceFRBFUtils.h"
#include "../core/wendlandRbf.h"

#include <gsl/gsl_roots.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_multimin.h>

#include <fstream>

double gNewtonFRBF(double x, void *params)
{
    parametersNewtonFRBF *p = (parametersNewtonFRBF *)params;

    pcl::PointXYZ p0;
    p0.x = p->po_r.x+ x * p->no_r.normal_x;
    p0.y = p->po_r.y+ x * p->no_r.normal_y;
    p0.z = p->po_r.z+ x * p->no_r.normal_z;

    double sum = 0.;

    for(std::size_t m=0;m<p->listSph_r.size();m++)
    {
        float tmpx = (p->listSph_r.at(m)->getCenter().x-p0.x);
        float tmpy = (p->listSph_r.at(m)->getCenter().y-p0.y);
        float tmpz = (p->listSph_r.at(m)->getCenter().z-p0.z);
        float distR = sqrt(tmpx*tmpx+tmpy*tmpy+tmpz*tmpz);

        float r = distR/p->listSph_r.at(m)->getRadius();

        if(r<1.)
        {
            double res = p->listSph_r.at(m)->getLocalCell()->getValueOfImplicitFunction(p0.x,p0.y,p0.z);
            sum += (res)*wendlandRbf::getValueRbfWendland(r);
        }
    }
    return sum;
}

double dgNewtonFRBF(double x, void *params)
{
    parametersNewtonFRBF *p = (parametersNewtonFRBF *)params;

    pcl::PointXYZ p0;
    p0.x = p->po_r.x+ x * p->no_r.normal_x;
    p0.y = p->po_r.y+ x * p->no_r.normal_y;
    p0.z = p->po_r.z+ x * p->no_r.normal_z;

    pcl::PointXYZ grad;
    grad.x=0;
    grad.y=0;
    grad.z=0;

    for(std::size_t m=0;m<p->listSph_r.size();m++)
    {
        float tmpx = (p->listSph_r.at(m)->getCenter().x-p0.x);
        float tmpy = (p->listSph_r.at(m)->getCenter().y-p0.y);
        float tmpz = (p->listSph_r.at(m)->getCenter().z-p0.z);
        float distR = sqrt(tmpx*tmpx+tmpy*tmpy+tmpz*tmpz);

        float r = distR/p->listSph_r.at(m)->getRadius();

        if(r<1.)
        {
            Eigen::Vector4f X(p0.x,p0.y,p0.z,1);
            Eigen::Vector4f res = ((2*p->listSph_r.at(m)->getLocalCell()->getM1().matrix())*X)+(p->listSph_r.at(m)->getLocalCell()->getM2());

            grad.x+=(res[0])*std::pow(1-r,4.0)*(1.+4.*r);
            grad.y+=(res[1])*std::pow(1-r,4.0)*(1.+4.*r);
            grad.z+=(res[2])*std::pow(1-r,4.0)*(1.+4.*r);
        }
    }

    double prod = grad.x*p->no_r.normal_x+grad.y*p->no_r.normal_y+grad.z*p->no_r.normal_z;
    return prod;
}

void gdgNewtonFRBF(double x, void *params, double *y, double *dy)
{
    parametersNewtonFRBF *p = (parametersNewtonFRBF *)params;

    pcl::PointXYZ p0;
    p0.x = p->po_r.x+ x * p->no_r.normal_x;
    p0.y = p->po_r.y+ x * p->no_r.normal_y;
    p0.z = p->po_r.z+ x * p->no_r.normal_z;

    double sum = 0.;
    pcl::PointXYZ grad;
    grad.x=0;
    grad.y=0;
    grad.z=0;

    for(std::size_t m=0;m<p->listSph_r.size();m++)
    {
        float tmpx = (p->listSph_r.at(m)->getCenter().x-p0.x);
        float tmpy = (p->listSph_r.at(m)->getCenter().y-p0.y);
        float tmpz = (p->listSph_r.at(m)->getCenter().z-p0.z);
        float distR = sqrt(tmpx*tmpx+tmpy*tmpy+tmpz*tmpz);

        float r = distR/p->listSph_r.at(m)->getRadius();

        if(r<1.)
        {
            Eigen::Vector4f X(p0.x,p0.y,p0.z,1);
            Eigen::Vector4f res = ((2*p->listSph_r.at(m)->getLocalCell()->getM1().matrix())*X)+(p->listSph_r.at(m)->getLocalCell()->getM2());

            grad.x+=(res[0])*std::pow(1-r,4.0)*(1.+4.*r);
            grad.y+=(res[1])*std::pow(1-r,4.0)*(1.+4.*r);
            grad.z+=(res[2])*std::pow(1-r,4.0)*(1.+4.*r);

            double res2 = p->listSph_r.at(m)->getLocalCell()->getValueOfImplicitFunction(p0.x,p0.y,p0.z);
            sum += (res2)*wendlandRbf::getValueRbfWendland(r);
        }
    }

    *y = sum;
    *dy = grad.x*p->no_r.normal_x+grad.y*p->no_r.normal_y+grad.z*p->no_r.normal_z;
}

pcl::PointXYZ surfaceFRBFUtils::getPointOnSurfaceNewton(pcl::PointXYZ pt, pcl::Normal n, surfaceFrbf& psurface)
{
    parametersNewtonFRBF param;

    param.listSph_r = psurface.getTree()->getListLeaf();
    param.po_r = pt;
    param.no_r = n;

    int status;
    int iter = 0, max_iter = 100;
    const gsl_root_fdfsolver_type *T;
    gsl_root_fdfsolver *s;
    double x0, x = 0.;
    gsl_function_fdf FDF;

    FDF.f = &gNewtonFRBF;
    FDF.df = &dgNewtonFRBF;
    FDF.fdf = &gdgNewtonFRBF;
    FDF.params = &param;

    gsl_set_error_handler_off();

    T = gsl_root_fdfsolver_newton;
    s = gsl_root_fdfsolver_alloc (T);
    gsl_root_fdfsolver_set (s, &FDF, x);

    do
      {
        iter++;
        status = gsl_root_fdfsolver_iterate (s);
        x0 = x;
        x = gsl_root_fdfsolver_root (s);
        status = gsl_root_test_delta (x, x0, 0, 1e-3);
      }
    while (status == GSL_CONTINUE && iter < max_iter);

    if(iter == max_iter)
    {   //std::cout<<"minimizer has not converged"<<std::endl;
        x=0;
        std::ofstream outfile;
        outfile.open("log/projection.xyz", std::ios_base::app);
        outfile<<pt.x<<" "<<pt.y<<" "<<pt.z<<" "<<x<<std::endl;
    }

    pcl::PointXYZ p0;
    p0.x = pt.x+ x * param.no_r.normal_x;
    p0.y = pt.y+ x * param.no_r.normal_y;
    p0.z = pt.z+ x * param.no_r.normal_z;

    gsl_root_fdfsolver_free (s);

    return p0;
}

double surfaceFRBFUtils::computeImplicitFctValue(pcl::PointXYZ pt, surfaceFrbf& psurface)
{
  double val=0.;
  for(std::size_t m=0;m<psurface.getTree()->getListLeaf().size();m++)
  {
    quadLeaf* leaf = psurface.getTree()->getListLeaf().at(m);
    if(!leaf->isEmpty()){
      float tmpx = (leaf->getCenter().x-pt.x);
      float tmpy = (leaf->getCenter().y-pt.y);
      float tmpz = (leaf->getCenter().z-pt.z);
      float distR = sqrt(tmpx*tmpx+tmpy*tmpy+tmpz*tmpz);
      float r = distR/leaf->getRadius();
      if(r<1.)
      {
        Eigen::Vector4f X(pt.x,pt.y,pt.z,1);
        double coefrbf = wendlandRbf::getValueRbfWendland(r)/surfaceFRBFUtils::getSumRbfAtPoint(pt,psurface);
        double res = leaf->getLocalCell()->getValueOfImplicitFunction(pt.x,pt.y,pt.z);
        val += (res)*coefrbf;
      }
    }
  }
  return val;
}

bool surfaceFRBFUtils::computeImplicitFctValueAndPseudoGrad(pcl::PointXYZ pt, double& val, pcl::Normal& grad, surfaceFrbf& psurface)
{
  bool updated=false;
  float sumRBF = 0.;
  for(std::size_t m=0;m<psurface.getTree()->getListLeaf().size();m++)
  {
    quadLeaf* leaf = psurface.getTree()->getListLeaf().at(m);
    if(!leaf->isEmpty()){
      float tmpx = (leaf->getCenter().x-pt.x);
      float tmpy = (leaf->getCenter().y-pt.y);
      float tmpz = (leaf->getCenter().z-pt.z);
      float distR = sqrt(tmpx*tmpx+tmpy*tmpy+tmpz*tmpz);
      float r = distR/leaf->getRadius();

      if(r<1.)
      {
        Eigen::Vector4f X(pt.x,pt.y,pt.z,1);
        Eigen::Vector4f resv = ((2*leaf->getLocalCell()->getM1().matrix())*X)+(leaf->getLocalCell()->getM2());
        double coefrbf = wendlandRbf::getValueRbfWendland(r);
        sumRBF+=coefrbf;

        double res = leaf->getLocalCell()->getValueOfImplicitFunction(pt.x,pt.y,pt.z);

        val += (res)*coefrbf;

        grad.normal_x+=(resv[0])*coefrbf;
        grad.normal_y+=(resv[1])*coefrbf;
        grad.normal_z+=(resv[2])*coefrbf;

        updated=true;
      }
    }
  }
  val=val/sumRBF;
  grad.normal_x=grad.normal_x/sumRBF;
  grad.normal_y=grad.normal_y/sumRBF;
  grad.normal_z=grad.normal_z/sumRBF;
  return updated;
}

double surfaceFRBFUtils::getSumRbfAtPoint(pcl::PointXYZ pt, surfaceFrbf& psurface)
{
  double sum=0.;
  for(std::size_t m=0;m<psurface.getTree()->getListLeaf().size();m++)
  {
    quadLeaf* leaf = psurface.getTree()->getListLeaf().at(m);
    float tmpx = (leaf->getCenter().x-pt.x);
    float tmpy = (leaf->getCenter().y-pt.y);
    float tmpz = (leaf->getCenter().z-pt.z);
    float distR = sqrt(tmpx*tmpx+tmpy*tmpy+tmpz*tmpz);
    float r = distR/leaf->getRadius();
    sum += wendlandRbf::getValueRbfWendland(r);
  }
  return sum;
}
