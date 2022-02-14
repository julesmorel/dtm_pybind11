#include "multidimensionalminimizer.h"

#include <fstream>

double E(const gsl_vector *v, void *params)
{
  double n1, n2, n3, t;
  parameters *p = (parameters *)params;

  n1 = gsl_vector_get(v, 0);
  n2 = gsl_vector_get(v, 1);
  n3 = gsl_vector_get(v, 2);
  t = gsl_vector_get(v, 3);

  double normN = sqrt(n1*n1+n2*n2+n3*n3);

  double energie=0.;
  if(p->piCloud_p.size()>0){
    float numNeighborsInv = 1./p->piCloud_p.size();

    for(int i=0;i<p->piCloud_p.size();i++)
    {
      double x = (p->piCloud_p.at(i).x-p->r_p.x+t*n1);
      double y = (p->piCloud_p.at(i).y-p->r_p.y+t*n2);
      double z = (p->piCloud_p.at(i).z-p->r_p.z+t*n3);
      double w = p->piCloud_p.at(i).intensity;
      double prodscal = (x*n1+y*n2+z*n3)/normN;
      double theta = wendlandRbf::getValueRbfWendland(sqrt(x*x+y*y+z*z)/p->support_p);

      //energie += numNeighborsInv*p->coef_p*prodscal*prodscal*theta;
      energie += p->coef_p*w*prodscal*prodscal*theta;
    }
  }

  double xprime = (p->xprime_p.x-p->r_p.x+t*n1);
  double yprime = (p->xprime_p.y-p->r_p.y+t*n2);
  double zprime = (p->xprime_p.z-p->r_p.z+t*n3);
  double prodscalprime = xprime*p->nprime_p.normal_x+yprime*p->nprime_p.normal_y+zprime*p->nprime_p.normal_z;

  energie += (1.-p->coef_p)*prodscalprime*prodscalprime;

  return energie;
}

void gradE(const gsl_vector *v, void *params, gsl_vector *df)
{
  double n1, n2, n3, t;
  parameters *p = (parameters *)params;

  double dfn1=0.;
  double dfn2=0.;
  double dfn3=0.;
  double dft=0.;

  n1 = gsl_vector_get(v, 0);
  n2 = gsl_vector_get(v, 1);
  n3 = gsl_vector_get(v, 2);
  t = gsl_vector_get(v, 3);

  Eigen::MatrixXd tin(3,4);
  tin(0,0)=t;
  tin(1,1)=t;
  tin(2,2)=t;
  tin(0,3)=n1;
  tin(1,3)=n2;
  tin(2,3)=n3;

  double normN = sqrt(n1*n1+n2*n2+n3*n3);

  if(p->piCloud_p.size()>0){
    float numNeighborsInv = 1./p->piCloud_p.size();

    for(int i=0;i<p->piCloud_p.size();i++)
    {
      //1st term
      double x = (p->piCloud_p.at(i).x-p->r_p.x+t*n1);
      double y = (p->piCloud_p.at(i).y-p->r_p.y+t*n2);
      double z = (p->piCloud_p.at(i).z-p->r_p.z+t*n3);
      double w = p->piCloud_p.at(i).intensity;
      double prodscal = (x*n1+y*n2+z*n3)/normN;
      double norm = sqrt(x*x+y*y+z*z);
      double theta = wendlandRbf::getValueRbfWendland(norm/p->support_p);

      Eigen::Vector3d m;
      Eigen::Matrix3d sub;
      Eigen::Vector3d nvect;
      nvect(0)=n1;
      nvect(1)=n2;
      nvect(2)=n3;
      sub = Eigen::Matrix3d::Identity() - (nvect*nvect.transpose())*(1./(normN*normN));

      Eigen::Vector3d pimoinsr;
      pimoinsr(0)=p->piCloud_p.at(i).x-p->r_p.x;
      pimoinsr(1)=p->piCloud_p.at(i).y-p->r_p.y;
      pimoinsr(2)=p->piCloud_p.at(i).z-p->r_p.z;
      m=sub*pimoinsr;

      double dfn11=0.;
      double dfn12=0.;
      double dfn21=0.;
      double dfn22=0.;
      double dfn31=0.;
      double dfn32=0.;

      dfn11=2.*prodscal*theta*(m(0)/normN+t*n1/normN);
      dfn21=2.*prodscal*theta*(m(1)/normN+t*n2/normN);
      dfn31=2.*prodscal*theta*(m(2)/normN+t*n3/normN);

      //2nd term
      Eigen::Vector3d gradThetha;
      double drbf = wendlandRbf::getValueRbfWendlandDeriv(norm/p->support_p);
      gradThetha(0) = drbf * x / norm;
      gradThetha(1) = drbf * y / norm;
      gradThetha(2) = drbf * z / norm;

      Eigen::Vector4d tmp;
      tmp = tin.transpose() * gradThetha;
      dfn12 = prodscal*prodscal*tmp(0);
      dfn22 = prodscal*prodscal*tmp(1);
      dfn32 = prodscal*prodscal*tmp(2);

      /*dfn1 += numNeighborsInv*p->coef_p*(dfn11 + dfn12);
      dfn2 += numNeighborsInv*p->coef_p*(dfn21 + dfn22);
      dfn3 += numNeighborsInv*p->coef_p*(dfn31 + dfn32);

      dft += numNeighborsInv*p->coef_p*(2.*prodscal*normN*theta + prodscal*prodscal*tmp(3));*/

      dfn1 += p->coef_p*w*(dfn11 + dfn12);
      dfn2 += p->coef_p*w*(dfn21 + dfn22);
      dfn3 += p->coef_p*w*(dfn31 + dfn32);

      dft += p->coef_p*w*(2.*prodscal*normN*theta + prodscal*prodscal*tmp(3));
    }
  }
  //3nd term
  Eigen::VectorXd nprimeVect(3);
  double psx = p->nprime_p.normal_x*(p->xprime_p.x-p->r_p.x+t*n1);
  double psy = p->nprime_p.normal_y*(p->xprime_p.y-p->r_p.y+t*n2);
  double psz = p->nprime_p.normal_z*(p->xprime_p.z-p->r_p.z+t*n3);
  double ps = psx+psy+psz;
  nprimeVect(0)=2.*ps*p->nprime_p.normal_x;
  nprimeVect(1)=2.*ps*p->nprime_p.normal_y;
  nprimeVect(2)=2.*ps*p->nprime_p.normal_z;
  Eigen:: VectorXd tmp2(4);

  Eigen::MatrixXd tin2(3,4);
  tin2(0,0)=t;
  tin2(1,1)=t;
  tin2(2,2)=t;
  tin2(0,3)=n1;
  tin2(1,3)=n2;
  tin2(2,3)=n3;

  tmp2 = tin2.transpose() * nprimeVect;
  dfn1 += (1-p->coef_p)*tmp2(0);
  dfn2 += (1-p->coef_p)*tmp2(1);
  dfn3 += (1-p->coef_p)*tmp2(2);
  dft += (1-p->coef_p)*tmp2(3);

  gsl_vector_set(df, 0, dfn1);
  gsl_vector_set(df, 1, dfn2);
  gsl_vector_set(df, 2, dfn3);
  gsl_vector_set(df, 3, dft);
}

void EandGradE(const gsl_vector *x, void *params, double *f, gsl_vector *df)
{
    *f = E(x, params);
    gradE(x, params, df);
}

multiDimensionalMinimizer::multiDimensionalMinimizer(pcl::PointCloud<pcl::PointXYZI>& ppiCloud, pcl::PointXYZ pr, pcl::PointXYZ pxprime, pcl::Normal pnprime, float psupport, float ratio)
{
    parameters param;
    param.piCloud_p = ppiCloud;
    param.r_p = pr;
    param.xprime_p = pxprime;
    param.nprime_p = pnprime;
    param.support_p = psupport;
    param.coef_p = ratio;

    //std::cout<<"xprime : "<<param.xprime_p.x<<" "<<param.xprime_p.y<<" "<<param.xprime_p.z<<std::endl;

    size_t iterMax = 1000;
    size_t iter = 0;
    int status;

    const gsl_multimin_fdfminimizer_type *T;
    gsl_multimin_fdfminimizer *s;

    gsl_vector *x;
    gsl_multimin_function_fdf func;

    func.n = 4;
    func.f = &E;
    func.df = &gradE;
    func.fdf = &EandGradE;
    func.params = &param;

    /* Starting point*/
    x = gsl_vector_alloc (4);
    gsl_vector_set (x, 0, pnprime.normal_x);
    gsl_vector_set (x, 1, pnprime.normal_y);
    gsl_vector_set (x, 2, pnprime.normal_z);
    double tprime = 0;//sqrt((pr.x-pxprime.x)*(pr.x-pxprime.x)+(pr.y-pxprime.y)*(pr.y-pxprime.y)+(pr.z-pxprime.z)*(pr.z-pxprime.z));
    pcl::PointXYZ pxprimevec;
    pxprimevec.x=pr.x-pxprime.x;
    pxprimevec.y=pr.y-pxprime.y;
    pxprimevec.z=pr.z-pxprime.z;
    tprime = pxprimevec.x*pnprime.normal_x+pxprimevec.y*pnprime.normal_y+pxprimevec.z*pnprime.normal_z;
    gsl_vector_set (x, 3, tprime);

    T = gsl_multimin_fdfminimizer_conjugate_fr;
    s = gsl_multimin_fdfminimizer_alloc (T, 4);

    gsl_multimin_fdfminimizer_set (s, &func, x, 0.01, 0.1);

    do{
        iter++;
        status = gsl_multimin_fdfminimizer_iterate (s);

        //std::cout<<status<<" iter :"<<iter<<" n1 "<<gsl_vector_get (s->x, 0)<<" n2 "<<gsl_vector_get (s->x, 1)<<" n3 "<<gsl_vector_get (s->x, 2)<<" t "<<gsl_vector_get (s->x, 3)<<" E "<<s->f<<std::endl;

        if (status)break;

        status = gsl_multimin_test_gradient (s->gradient, 1e-3);

    }while (status == GSL_CONTINUE && iter < iterMax);

    if(iter == iterMax)
    {   //std::cout<<"minimizer has not converged"<<std::endl;
        double norm2 = gsl_vector_get (s->x, 0)*gsl_vector_get (s->x, 0)+gsl_vector_get (s->x, 1)*gsl_vector_get (s->x, 1)+gsl_vector_get (s->x, 2)*gsl_vector_get (s->x, 2);
        std::cout<<status<<" iter :"<<iter<<" norm2 : "<<norm2<<" n1 "<<gsl_vector_get (s->x, 0)<<" n2 "<<gsl_vector_get (s->x, 1)<<" n3 "<<gsl_vector_get (s->x, 2)<<" t "<<gsl_vector_get (s->x, 3)<<" E "<<s->f<<" NN "<<ppiCloud.size()<<std::endl;
        std::ofstream outfile;
        outfile.open("log/errorMultiDim.xyz", std::ios_base::app);
        outfile<<pr.x<<" "<<pr.y<<" "<<pr.z<<" "<<norm2<<" "<<t<<" "<<E<<std::endl;
        n.normal_x=0.;
        n.normal_y=0.;
        n.normal_z=0.;
        t=0.;
    }else{
        n.normal_x=gsl_vector_get (s->x, 0);
        n.normal_y=gsl_vector_get (s->x, 1);
        n.normal_z=gsl_vector_get (s->x, 2);
        t=gsl_vector_get (s->x, 3);
    }
    //std::cout<<status<<" "<<iter<<std::endl;

    gsl_multimin_fdfminimizer_free (s);
    gsl_vector_free (x);

    /*std::string filename= "/home/jules/Desktop/status.txt";
    std::ofstream fileStream(filename.c_str(), std::ios::out | std::ios::trunc);

    fileStream<<status<<std::endl;

    fileStream.close();*/
}
