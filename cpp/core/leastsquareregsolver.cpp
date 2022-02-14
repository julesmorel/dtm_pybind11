/****************************************************************************
 Copyright (C) 2017 Jules Morel

 Contact : jules.morel@ifpindia.org

 Developers : Jules MOREL (IFP LSIS)

 This file is part of PluginIFPLSIS library.

 PluginIFPLSIS is free library: you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 PluginIFPLSIS is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU Lesser General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/lgpl.html>.
*****************************************************************************/

#include "leastsquareregsolver.h"

#include <vector>

#include <gsl/gsl_multifit.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_multifit_nlin.h>
#include <gsl/gsl_matrix.h>

//error = weighted sums of squared residuals
void leastSquareRegSolver::solveQuadric(pcl::PointXYZ centroid, float radius, bool useWeight, pcl::PointCloud<pcl::PointXYZI>  pts, std::vector<double>* coeff, double* error)
{
    int status;
    gsl_matrix *X, *cov;
    gsl_vector *y, *c, *w ;

     gsl_set_error_handler_off();

    int n = pts.size();
    X = gsl_matrix_alloc (n, NBR_COEFS);
    y = gsl_vector_alloc (n);

    if(useWeight) w = gsl_vector_alloc (n);

    c = gsl_vector_alloc (NBR_COEFS);
    cov = gsl_matrix_alloc (NBR_COEFS, NBR_COEFS);

    for (int i = 0; i < n; i++)
    {
        gsl_matrix_set (X, i, 0, pts.at(i).x*pts.at(i).x);
        gsl_matrix_set (X, i, 1, pts.at(i).x*pts.at(i).y);
        gsl_matrix_set (X, i, 2, pts.at(i).y*pts.at(i).y);
        gsl_matrix_set (X, i, 3, pts.at(i).x);
        gsl_matrix_set (X, i, 4, pts.at(i).y);
        //gsl_matrix_set (X, i, 0, 0);
        //gsl_matrix_set (X, i, 1, 0);
        //gsl_matrix_set (X, i, 2, 0);
        //gsl_matrix_set (X, i, 3, 0.);
        //gsl_matrix_set (X, i, 4, 0.);
        gsl_matrix_set (X, i, 5, 1.0);
        //gsl_matrix_set (X, i, 5, 0.);

        if(useWeight){
            /*float tmpx = (centroid.x-pts.at(i).x);
            float tmpy = (centroid.y-pts.at(i).y);
            float tmpz = (centroid.z-pts.at(i).z);
            float distR = sqrt(tmpx*tmpx+tmpy*tmpy+tmpz*tmpz);

            float r = distR/radius;
            gsl_vector_set (w, i, std::pow(1-r,4.0)*(1.+4.*r));*/
            gsl_vector_set (w, i, pts.at(i).intensity);
        }

        gsl_vector_set (y, i, pts.at(i).z);
    }

    gsl_multifit_linear_workspace * work = gsl_multifit_linear_alloc (n, NBR_COEFS);

    if(useWeight){
        status = gsl_multifit_wlinear(X, w, y, c, cov,error, work);
        if (status) {
              std::cout<<" GSL error quadric | pts "<<pts.size()<<std::endl;
        }
    }else{
        gsl_multifit_linear (X, y, c, cov,error, work);
    }

    //error normalisation
    (*error) = sqrt((*error)/(double)n);

    gsl_multifit_linear_free (work);

    for(int i=0;i<NBR_COEFS;i++)
    {
      coeff->push_back(gsl_vector_get (c,i));
    }

    gsl_matrix_free (X);
    gsl_vector_free (y);
    gsl_vector_free (c);
    gsl_matrix_free (cov);
  }

  void leastSquareRegSolver::solvePlaneLocal(pcl::PointXYZ centroid, pcl::PointCloud<pcl::PointXYZI>  pts, std::vector<double>* coeff, double* error)
  {
      int status;
      gsl_matrix *X, *cov;
      gsl_vector *y, *c, *w ;

       gsl_set_error_handler_off();

      int n = pts.size();
      X = gsl_matrix_alloc (n, NBR_COEFS);
      y = gsl_vector_alloc (n);
      w = gsl_vector_alloc (n);

      c = gsl_vector_alloc (NBR_COEFS);
      cov = gsl_matrix_alloc (NBR_COEFS, NBR_COEFS);

      for (int i = 0; i < n; i++)
      {
          gsl_matrix_set (X, i, 0, 0);
          gsl_matrix_set (X, i, 1, 0);
          gsl_matrix_set (X, i, 2, 0);
          gsl_matrix_set (X, i, 3, pts.at(i).x);
          gsl_matrix_set (X, i, 4, pts.at(i).y);
          gsl_matrix_set (X, i, 5, 1.0);

          gsl_vector_set (w, i, pts.at(i).intensity);

          gsl_vector_set (y, i, pts.at(i).z);
      }

      gsl_multifit_linear_workspace * work = gsl_multifit_linear_alloc (n, NBR_COEFS);
      gsl_multifit_wlinear(X, w, y, c, cov,error, work);

      //error normalisation
      (*error) = sqrt((*error)/(double)n);

      gsl_multifit_linear_free (work);

      for(int i=0;i<NBR_COEFS;i++)
      {
        coeff->push_back(gsl_vector_get (c,i));
      }

      gsl_matrix_free (X);
      gsl_vector_free (y);
      gsl_vector_free (c);
      gsl_matrix_free (cov);
    }

  void leastSquareRegSolver::solvePlane(pcl::PointXYZ centroid, pcl::PointCloud<pcl::PointXYZI>  pts, std::vector<double>* coeff, double* error)
  {
    int status;
    gsl_matrix *X, *cov;
    gsl_vector *y, *c, *w;

    gsl_set_error_handler_off();

    int n = pts.size();
    int nbCoefPlane = 2;
    X = gsl_matrix_alloc (n, nbCoefPlane);
    y = gsl_vector_alloc (n);
    w = gsl_vector_alloc (n);

    c = gsl_vector_alloc (nbCoefPlane);
    cov = gsl_matrix_alloc (nbCoefPlane, nbCoefPlane);

    for (int i = 0; i < n; i++)
    {
      gsl_matrix_set (X, i, 0, pts.at(i).x-centroid.x);
      gsl_matrix_set (X, i, 1, pts.at(i).y-centroid.y);
      gsl_vector_set (w, i, pts.at(i).intensity);
      gsl_vector_set (y, i, pts.at(i).z-centroid.z);
    }

    gsl_multifit_linear_workspace * work = gsl_multifit_linear_alloc (n, nbCoefPlane);

    status = gsl_multifit_wlinear (X, w, y, c, cov, error, work);

    if (status) {
        std::cout<<" GSL error plane | pts "<<pts.size()<<std::endl;
    }

    gsl_multifit_linear_free (work);

    for(int i=0;i<nbCoefPlane;i++)
    {
      coeff->push_back(gsl_vector_get (c,i));
    }

    gsl_matrix_free (X);
    gsl_vector_free (y);
    gsl_vector_free (c);
    gsl_matrix_free (cov);
  }
