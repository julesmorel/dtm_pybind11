#ifndef CSRBF_H
#define CSRBF_H

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

struct csrbfPCL
{
  PCL_ADD_POINT4D
  float radius;
  float alpha;
  float alphaUpdated;
  float gradCSRBF_x;
  float gradCSRBF_y;
  float gradCSRBF_z;
  float gradFRBF_x;
  float gradFRBF_y;
  float gradFRBF_z;
  float vectDef_x;
  float vectDef_y;
  float vectDef_z;
  float projSurface_x;
  float projSurface_y;
  float projSurface_z;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

 POINT_CLOUD_REGISTER_POINT_STRUCT (csrbfPCL,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, radius, radius)
                                   (float, alpha,alpha)
                                    (float, alphaUpdated,alphaUpdated)
                                    (float, gradCSRBF_x,gradCSRBF_x)
                                    (float, gradCSRBF_y,gradCSRBF_y)
                                    (float, gradCSRBF_z,gradCSRBF_z)
                                    (float, gradFRBF_x,gradFRBF_x)
                                    (float, gradFRBF_y,gradFRBF_y)
                                    (float, gradFRBF_z,gradFRBF_z)
                                    (float, vectDef_x,vectDef_x)
                                    (float, vectDef_y,vectDef_y)
                                    (float, vectDef_z,vectDef_z)
                                    (float, projSurface_x,projSurface_x)
                                    (float, projSurface_y,projSurface_y)
                                    (float, projSurface_z,projSurface_z))

#endif // CSRBF_H
