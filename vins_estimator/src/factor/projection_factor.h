#pragma once

#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../parameters.h"

/*
If the size of the parameter blocks and the size of the residual vector is known at compile time (this is the common case), SizeCostFunction can be used where these values can be specified as template parameters and the user only needs to implement CostFunction::Evaluate().
*/
class ProjectionFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 1>//kNumResiduals,parameter1 sizes,parameter2 sizes.....
{
  public:
    ProjectionFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector3d pts_i, pts_j;
    Eigen::Matrix<double, 2, 3> tangent_base;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};
