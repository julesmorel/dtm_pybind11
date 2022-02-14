#include "surfaceCSRBFUtils.h"
#include "../core/wendlandRbf.h"

#include <gsl/gsl_roots.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_multimin.h>

#include <fstream>

double gNewton (double x, void *params)
{
    parametersNewton *p = (parametersNewton *)params;

    pcl::PointXYZ p0;
    p0.x = p->po_r.x+ x * p->no_r.normal_x;
    p0.y = p->po_r.y+ x * p->no_r.normal_y;
    p0.z = p->po_r.z+ x * p->no_r.normal_z;

    double sum = 0.;

    for(std::size_t m=0;m<p->listSph_r.size();m++)
    {
        float tmpx = (p0.x-p->listSph_r.at(m).x);
        float tmpy = (p0.y-p->listSph_r.at(m).y);
        float tmpz = (p0.z-p->listSph_r.at(m).z);
        float distR = sqrt(tmpx*tmpx+tmpy*tmpy+tmpz*tmpz);
        float r = distR/p->listSph_r.at(m).radius;

        if(r<1.)
        {
            sum += p->listSph_r.at(m).alpha*wendlandRbf::getValueRbfWendland(r);
        }
    }
    return sum;
}

double dgNewton (double x, void *params)
{
    parametersNewton *p = (parametersNewton *)params;

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
        float tmpx = (p0.x-p->listSph_r.at(m).x);
        float tmpy = (p0.y-p->listSph_r.at(m).y);
        float tmpz = (p0.z-p->listSph_r.at(m).z);
        float distR = sqrt(tmpx*tmpx+tmpy*tmpy+tmpz*tmpz);
        float r = distR/p->listSph_r.at(m).radius;

        if(r<1. && distR!=0.)
        {
            float lambda =  p->listSph_r.at(m).alpha*wendlandRbf::getValueRbfWendlandDeriv(r)/distR;
            grad.x+=tmpx*lambda;
            grad.y+=tmpy*lambda;
            grad.z+=tmpz*lambda;
        }
    }

    double prod = grad.x*p->no_r.normal_x+grad.y*p->no_r.normal_y+grad.z*p->no_r.normal_z;
    return prod;
}

void gdgNewton(double x, void *params, double *y, double *dy)
{
    parametersNewton *p = (parametersNewton *)params;

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
        float tmpx = (p0.x-p->listSph_r.at(m).x);
        float tmpy = (p0.y-p->listSph_r.at(m).y);
        float tmpz = (p0.z-p->listSph_r.at(m).z);
        float distR = sqrt(tmpx*tmpx+tmpy*tmpy+tmpz*tmpz);

        float r = distR/p->listSph_r.at(m).radius;
        if(r<1)
        {
            sum += p->listSph_r.at(m).alpha*wendlandRbf::getValueRbfWendland(r);
        }
        if(r<1. && distR!=0.)
        {
            float lambda =  p->listSph_r.at(m).alpha*wendlandRbf::getValueRbfWendlandDeriv(r)/distR;
            grad.x+=tmpx*lambda;
            grad.y+=tmpy*lambda;
            grad.z+=tmpz*lambda;
        }
    }

    *y = sum;
    *dy = grad.x*p->no_r.normal_x+grad.y*p->no_r.normal_y+grad.z*p->no_r.normal_z;
}

pcl::PointXYZ surfaceCSRBFUtils::getPointOnSurfaceNewton(pcl::PointXYZ pt, pcl::Normal n , surfaceCsrbf& psurface)
{
    parametersNewton param;

    param.listSph_r = psurface.getCloudCsrbf();
    param.po_r = pt;
    param.no_r = n;

    int status;
    int iter = 0, max_iter = 1000;
    const gsl_root_fdfsolver_type *T;
    gsl_root_fdfsolver *s;
    double x0, x = 0.;
    gsl_function_fdf FDF;

    FDF.f = &gNewton;
    FDF.df = &dgNewton;
    FDF.fdf = &gdgNewton;
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
        status = gsl_root_test_delta (x, x0, 1e-2, 1e-3);
      }
    while (status == GSL_CONTINUE && iter < max_iter);

    if(iter == max_iter)
    {   //std::cout<<"minimizer has not converged"<<std::endl;
        x=0;
    }

    pcl::PointXYZ p0;
    p0.x = pt.x+ x * param.no_r.normal_x;
    p0.y = pt.y+ x * param.no_r.normal_y;
    p0.z = pt.z+ x * param.no_r.normal_z;

    gsl_root_fdfsolver_free (s);

    return p0;
  }

  pcl::Normal surfaceCSRBFUtils::getValueImplicitFctGrad(pcl::PointXYZ x, surfaceCsrbf& psurface, double radius)
  {
    pcl::Normal gradient;

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    csrbfPCL c;
    c.x=x.x;
    c.y=x.y;
    c.z=x.z;

    if ( psurface.getOctree()->radiusSearch (c, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
    {
      for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
        csrbfPCL c = psurface.getCloudCsrbf().at(pointIdxRadiusSearch[i]);
        float tmpx = x.x-c.x;
        float tmpy = x.y-c.y;
        float tmpz = x.z-c.z;
        float dist = sqrt(tmpx*tmpx+tmpy*tmpy+tmpz*tmpz);

        float r = dist/c.radius;
        if(r<1. && dist!=0.)
        {
          float lambda =  c.alpha*wendlandRbf::getValueRbfWendlandDeriv(r)/dist;
          gradient.normal_x+=tmpx*lambda;
          gradient.normal_y+=tmpy*lambda;
          gradient.normal_z+=tmpz*lambda;
        }
      }
    }
    return gradient;
  }

  double surfaceCSRBFUtils::getValueImplicitFct(pcl::PointXYZ x, surfaceCsrbf& psurface, double radius)
  {
    double sum = 0.;

    csrbfPCL c;
    c.x=x.x;
    c.y=x.y;
    c.z=x.z;

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    if ( psurface.getOctree()->radiusSearch (c, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
    {
      for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
        csrbfPCL c = psurface.getCloudCsrbf().at(pointIdxRadiusSearch[i]);
        float tmpx = x.x-c.x;
        float tmpy = x.y-c.y;
        float tmpz = x.z-c.z;
        float dist = sqrt(tmpx*tmpx+tmpy*tmpy+tmpz*tmpz);

        float r = dist/c.radius;
        if(r<1. && dist!=0.)
        {
          sum += c.alpha*wendlandRbf::getValueRbfWendland(r);
        }
      }
    }
    return sum;
  }
