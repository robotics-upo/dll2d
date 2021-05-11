#ifndef __DLL2DSOLVER_HPP__
#define __DLL2DSOLVER_HPP__

#include <vector>
#include <utility>
#include "ceres/ceres.h"
#include "glog/logging.h"
#include <dll2d/df2d.hpp>

using ceres::CostFunction;
using ceres::SizedCostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using ceres::HuberLoss;
using ceres::Covariance;

class DLL2DCostFunction
  : public SizedCostFunction<1 /* number of residuals */,
                             3 /* size of first parameter */> 
{
 public:
    DLL2DCostFunction(double px, double py, DF2D &grid)
      : _px(px), _py(py), _grid(grid)
    {

    }

    virtual ~DLL2DCostFunction(void) 
    {

    }

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const 
    {
        double tx = parameters[0][0];
        double ty = parameters[0][1];
        double a  = parameters[0][2];

        // Compute the residual
        BilinearParams p;
        double sa, ca, nx, ny, dxa, dya;
        double c0, c1, c2, c3;
        sa = sin(a);
        ca = cos(a);
        nx = tx + _px*ca - _py*sa;
        ny = ty + _px*sa + _py*ca;
        p = _grid.getDistInterpolation(nx, ny);
        c0 = p.a0; c1 = p.a1; c2 = p.a2; c3 = p.a3;

        residuals[0] =  c0 + c1*nx + c2*ny + c3*nx*ny;

        if (jacobians != NULL && jacobians[0] != NULL) 
        {
            double dxa, dya;
            dxa = _py*ca + _px*sa;
            dya = _px*ca - _py*sa;
            jacobians[0][0] = c1 + c3*ny;
            jacobians[0][1] = c2 + c3*nx;
            jacobians[0][2] = c2*dya - c1*dxa + c3*dya*nx - c3*dxa*ny;
        }

        return true;
    }

  private:

    // Point to be evaluated
    double _px; 
    double _py; 

    // Distance grid
    DF2D &_grid;
};

class DLL2DSolver
{
  private:

    // Distance grid
    DF2D &_grid;

    // Optimizer parameters
    int _max_num_iterations;
    int _max_threads;

  public:

    DLL2DSolver(DF2D *grid) : _grid(*grid)
    {
        google::InitGoogleLogging("DLL2DSolver");
        _max_num_iterations = 50;
        _max_threads = 1;
    }

    ~DLL2DSolver(void)
    {

    } 

    bool setMaxIterations(int n)
    {
        if(n>0)
        {
            _max_num_iterations = n;
            return true;
        } 
        else
            return false;
    }

    bool setMaxThreads(int n)
    {
        if(n>0)
        {
            _max_threads = n;
            return true;
        } 
        else
            return false;
    }

    bool solve(std::vector<Point2D> &p, double &tx, double &ty, double &yaw, double *covMatrix = NULL)
    {
        // Initial solution
        bool converged = false;
        double x[3];
        x[0] = tx; x[1] = ty; x[2] = yaw; 

        // Build the problem.
        Problem problem;

        // Set up a cost funtion per point into the cloud
        for(unsigned int i=0; i<p.size(); i++)
        {
            CostFunction* cost_function = new DLL2DCostFunction(p[i].x, p[i].y, _grid);
            problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.1), x); 
        }

        // Run the solver!
        Solver::Options options;
        options.minimizer_progress_to_stdout = false;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = _max_num_iterations;
        options.num_threads = _max_threads; 
        Solver::Summary summary;
        Solve(options, &problem, &summary);
        if(summary.termination_type == ceres::CONVERGENCE)
            converged = true;

        // Compute covariance matrix
        if(covMatrix != NULL && converged)
        {
            Covariance::Options covOptions;
            Covariance covariance(covOptions);
            std::vector<std::pair<const double*, const double*> > covariance_blocks;
            covariance_blocks.push_back(std::make_pair(x, x));
            CHECK(covariance.Compute(covariance_blocks, &problem));
            covariance.GetCovarianceBlock(x, x, covMatrix);
        }

        // Get the solution
        if(converged)
        {
            tx = x[0]; ty = x[1]; yaw = x[2]; 
        }

        return converged; 
    }
};




#endif
