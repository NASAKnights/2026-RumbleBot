#include "utils/BallisticsInterpolator.h"
#include <algorithm>
#include <cmath>

int BallisticsInterpolator::searchsorted(double first, double last, int n, double value) const {
    if (value <= first) return 0;
    if (value >= last) return n - 1;
    
    double step = (last - first) / (n - 1);
    int idx = static_cast<int>((value - first) / step);
    return std::min(idx, n - 2);
}

double BallisticsInterpolator::trilinear_helper(
    double c000, double c001, double c010, double c011,
    double c100, double c101, double c110, double c111,
    double xd, double yd, double zd) const {
    
    // First interpolate along x-axis (4 interpolations)
    double c00 = c000 * (1 - xd) + c100 * xd;
    double c01 = c001 * (1 - xd) + c101 * xd;
    double c10 = c010 * (1 - xd) + c110 * xd;
    double c11 = c011 * (1 - xd) + c111 * xd;
    
    // Then interpolate along y-axis (2 interpolations)
    double c0 = c00 * (1 - yd) + c10 * yd;
    double c1 = c01 * (1 - yd) + c11 * yd;
    
    // Finally interpolate along z-axis (1 interpolation)
    double c = c0 * (1 - zd) + c1 * zd;
    
    return c;
}

void BallisticsInterpolator::interpolate(
    double vx, double vy, double distance, 
    double& rel_vx_out, double& rel_vy_out, double& rel_vz_out) const {
    
    // Find grid cell indices
    int i = searchsorted(first_vx, last_vx, dim_vx, vx);
    int j = searchsorted(first_vy, last_vy, dim_vy, vy);
    int k = searchsorted(first_x, last_x, dim_x, distance);
    
    // Clamp to valid range
    i = std::max(0, std::min(i, dim_vx - 2));
    j = std::max(0, std::min(j, dim_vy - 2));
    k = std::max(0, std::min(k, dim_x - 2));
    
    // Compute grid coordinates
    double vx_step = (last_vx - first_vx) / (dim_vx - 1);
    double vy_step = (last_vy - first_vy) / (dim_vy - 1);
    double x_step = (last_x - first_x) / (dim_x - 1);
    
    double vx0 = first_vx + i * vx_step;
    double vx1 = first_vx + (i + 1) * vx_step;
    double vy0 = first_vy + j * vy_step;
    double vy1 = first_vy + (j + 1) * vy_step;
    double x0 = first_x + k * x_step;
    double x1 = first_x + (k + 1) * x_step;
    
    // Calculate normalized distances within the cube (0 to 1)
    double xd = (vx - vx0) / (vx1 - vx0);
    double yd = (vy - vy0) / (vy1 - vy0);
    double zd = (distance - x0) / (x1 - x0);
    
    // Interpolate rel_vx
    rel_vx_out = trilinear_helper(
        get(rel_vx_data, i,   j,   k  ), get(rel_vx_data, i,   j,   k+1),
        get(rel_vx_data, i,   j+1, k  ), get(rel_vx_data, i,   j+1, k+1),
        get(rel_vx_data, i+1, j,   k  ), get(rel_vx_data, i+1, j,   k+1),
        get(rel_vx_data, i+1, j+1, k  ), get(rel_vx_data, i+1, j+1, k+1),
        xd, yd, zd
    );
    
    // Interpolate rel_vy
    rel_vy_out = trilinear_helper(
        get(rel_vy_data, i,   j,   k  ), get(rel_vy_data, i,   j,   k+1),
        get(rel_vy_data, i,   j+1, k  ), get(rel_vy_data, i,   j+1, k+1),
        get(rel_vy_data, i+1, j,   k  ), get(rel_vy_data, i+1, j,   k+1),
        get(rel_vy_data, i+1, j+1, k  ), get(rel_vy_data, i+1, j+1, k+1),
        xd, yd, zd
    );
    
    // Interpolate rel_vz
    rel_vz_out = trilinear_helper(
        get(rel_vz_data, i,   j,   k  ), get(rel_vz_data, i,   j,   k+1),
        get(rel_vz_data, i,   j+1, k  ), get(rel_vz_data, i,   j+1, k+1),
        get(rel_vz_data, i+1, j,   k  ), get(rel_vz_data, i+1, j,   k+1),
        get(rel_vz_data, i+1, j+1, k  ), get(rel_vz_data, i+1, j+1, k+1),
        xd, yd, zd
    );
}