#pragma once
#include <Eigen/Dense>

using namespace Eigen;

namespace optim
{
    struct probData
    {
        int num_goal, maxiter, num, nvar, num_obs;
        
        float weight_smoothness, weight_psi, weight_v, rho_obs, t_fin, t, rho_ineq, rho_psi, rho_nonhol, weight_smoothness_psi;
        float a_obs, b_obs, a_max, v_max;
        ArrayXXf x_init, y_init, vx_init, vy_init, ax_init, ay_init, vx_fin, vy_fin, ax_fin, ay_fin;
        ArrayXXf cost_smoothness, cost_smoothness_psi, lincost_smoothness_psi;
        ArrayXXf x_guess, y_guess, x_fin, y_fin;
        ArrayXXf alpha_obs, d_obs, x_obs, y_obs;
        ArrayXXf A_eq, B_x_eq, B_y_eq, lamda_x, lamda_y, A_vel, A_acc, d_v, d_a, lamda_psi;
        ArrayXXf x, y, xdot, ydot, xddot, yddot, alpha_v, alpha_a;
        ArrayXXf res_obs, res_bounds, res_nonhol, res_eq;
        ArrayXXf psi_init, psi_fin, psidot_init, psidot_fin, v_init, v, psi, psidot, psiddot;
        ArrayXXf A_psi, A_nonhol, A_eq_psi, A_obs, A_ineq;

        ArrayXXf Pdot_upsample, v_controls, w_controls;
    };
    struct four_var
    {
        ArrayXXf a, b, c, d;
    };
    struct three_var
    {
        ArrayXXf a, b, c;
    };
     struct three_varr
    {
        ArrayXXf a, b;
        probData c;
    };
    struct two_var
    {
        ArrayXXf a, b;
    };
    void shape(ArrayXXf arr);
    float euclidean_dist(float x1, float y1, float x2, float y2);
    ArrayXXf ones(int row, int col);
    ArrayXXf reshape(ArrayXXf x, uint32_t r, uint32_t c);
    ArrayXXf clip2(ArrayXXf min, ArrayXXf max, ArrayXXf arr);
    ArrayXXf clip(float min, float max, ArrayXXf arr);
    ArrayXXf diff(ArrayXXf arr);
    ArrayXXf maximum(float val, ArrayXXf arr2);
    ArrayXXf minimum(float val, ArrayXXf arr2);
    ArrayXXf linspace(float t_init, float t_end, float steps);
    float binomialCoeff(float n, float k);
    ArrayXXf arctan2(ArrayXXf arr1, ArrayXXf arr2);
    ArrayXXf cumsum(ArrayXXf arr1, ArrayXXf arr2);
    ArrayXXf stack(ArrayXXf arr1, ArrayXXf arr2, char ch);
    three_var bernstein_coeff_order10_new(float n, float tmin, float tmax, ArrayXXf t_actual, int num);
    
    probData solve(probData prob_data, ArrayXXf P, ArrayXXf Pdot, ArrayXXf Pddot, ArrayXXf x_init, ArrayXXf x_fin, ArrayXXf y_init, ArrayXXf y_fin, ArrayXXf v_init,  ArrayXXf psi_init, ArrayXXf psidot_init, ArrayXXf psi_fin, ArrayXXf psidot_fin, ArrayXXf x_obs, ArrayXXf y_obs, bool warm );
    three_varr compute_x(probData prob_data, ArrayXXf P, ArrayXXf Pdot, ArrayXXf Pddot, ArrayXXf b_eq_x, ArrayXXf b_eq_y, ArrayXXf x_obs, ArrayXXf y_obs);
    three_varr compute_psi(probData prob_data, ArrayXXf P, ArrayXXf Pdot, ArrayXXf Pddot, ArrayXXf psi_temp, ArrayXXf b_eq_psi);
    probData mpc(probData prob_data, three_var PPP, ArrayXXf x_g, ArrayXXf y_g, float x_init, float y_init, float v_init, float psi_init, float psidot_init, ArrayXXf x_obs_temp, ArrayXXf y_obs_temp, ArrayXXf vx_obs, ArrayXXf vy_obs, bool warm);
    probData initialize_guess_alpha(probData prob_data);
    three_var compute_bernstein(ArrayXXf tot_time, float t_fin, int num);
    probData initializeArrays(probData prob_data);
}