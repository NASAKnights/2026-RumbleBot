#pragma once

#include <cstddef>

class BallisticsInterpolator {
private:
    // Grid dimensions
    const int dim_vx;
    const int dim_vy;
    const int dim_x;
    
    // Grid bounds
    const double first_vx;
    const double last_vx;
    const double first_vy;
    const double last_vy;
    const double first_x;
    const double last_x;
    
    // Pointers to the data arrays (stored as 1D pointers)
    const float* rel_vx_data;
    const float* rel_vy_data;
    const float* rel_vz_data;
    
    // Helper to access 3D array stored as 1D
    inline double get(const float* data, int i, int j, int k) const {
        return static_cast<double>(data[i * dim_vy * dim_x + j * dim_x + k]);
    }
    
    // Helper function to find index where value would be inserted
    int searchsorted(double first, double last, int n, double value) const;
    
    // Core trilinear interpolation on 8 corner values
    double trilinear_helper(double c000, double c001, double c010, double c011,
                           double c100, double c101, double c110, double c111,
                           double xd, double yd, double zd) const;
    
public:
    // Constructor - takes pointers to first element of 3D arrays
    template<size_t N1, size_t N2, size_t N3>
    BallisticsInterpolator(
        int dim_vx, int dim_vy, int dim_x,
        double first_vx, double last_vx,
        double first_vy, double last_vy,
        double first_x, double last_x,
        const float (&rel_vx)[N1][N2][N3],
        const float (&rel_vy)[N1][N2][N3],
        const float (&rel_vz)[N1][N2][N3]
    ) : dim_vx(dim_vx), dim_vy(dim_vy), dim_x(dim_x),
        first_vx(first_vx), last_vx(last_vx),
        first_vy(first_vy), last_vy(last_vy),
        first_x(first_x), last_x(last_x),
        rel_vx_data(&rel_vx[0][0][0]),
        rel_vy_data(&rel_vy[0][0][0]),
        rel_vz_data(&rel_vz[0][0][0])
    {
    }
    
    // Interpolate relative velocity components at given (vx, vy, distance)
    void interpolate(double vx, double vy, double distance, 
                    double& rel_vx_out, double& rel_vy_out, double& rel_vz_out) const;
};