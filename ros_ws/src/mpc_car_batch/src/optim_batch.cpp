#include <iostream>
#include "mpc_car_batch/optim_batch.h"
#include <fstream>
using namespace std;


namespace optim
{
	ArrayXXf ones(int row, int col)
	{
		ArrayXXf temp(row, col);
		temp = 1.0;
		return temp;
	}
	ArrayXXf stack(ArrayXXf arr1, ArrayXXf arr2, char ch)
	{
		if (ch == 'v')
		{
			ArrayXXf temp(arr1.rows() + arr2.rows(), arr1.cols());
			temp << arr1, arr2;
			return temp;
		}
		else
		{
			ArrayXXf temp(arr1.rows(), arr1.cols() + arr2.cols());
			temp << arr1, arr2;
			return temp;
		}

	}//np.shape(a)== (3,) a = [1,2,3] 
	ArrayXXf clip2(ArrayXXf min, ArrayXXf max, ArrayXXf arr)
	{
		for (int k = 0; k < arr.cols() * arr.rows(); k++)
		{
			if (arr(k) > max(k))
				arr(k) = max(k);
			if (arr(k) < min(k))
				arr(k) = min(k);
		}
		return arr;
	}
	ArrayXXf clip(float min, float max, ArrayXXf arr)
	{
		for (int k = 0; k < arr.cols() * arr.rows(); k++)
		{
			if (arr(k) > max)
				arr(k) = max;
			if (arr(k) < min)
				arr(k) = min;
		}
		return arr;
	}
	ArrayXXf diff(ArrayXXf arr)
	{
		ArrayXXf temp;
		
		if(arr.cols() == 1)
			temp = ArrayXXf(arr.rows() - 1, 1);
		else
			temp = ArrayXXf(1, arr.cols() - 1);

		for (int i = 0; i < temp.rows() * temp.cols(); i++)
		{
			temp(i) = arr(i + 1) - arr(i);
		}
		return temp;
	}
	ArrayXXf maximum(float val, ArrayXXf arr2)
	{
		ArrayXXf temp(arr2.rows(), arr2.cols());
		temp = val;

		int k = 0;
		for (int i = 0; i < arr2.cols(); i++)
		{
			for (int j = 0; j < arr2.rows(); j++)
			{
				if (arr2(k) > val)
					temp(k) = arr2(k);
				k++;
			}
		}
		return temp;
	}
	ArrayXXf minimum(float val, ArrayXXf arr2)
	{
		ArrayXXf temp(arr2.rows(), arr2.cols());
		temp = val;

		int k = 0;
		for (int i = 0; i < arr2.cols(); i++)
		{
			for (int j = 0; j < arr2.rows(); j++)
			{
				if (arr2(k) < val)
					temp(k) = arr2(k);
				k++;
			}
		}
		return temp;
	}
	ArrayXXf reshape(ArrayXXf x, uint32_t r, uint32_t c)
	{
		Map<ArrayXXf> rx(x.data(), r, c);
		return rx;
	}

	float binomialCoeff(float n, float k)
	{
		if (k == 0 || k == n)
			return 1;

		return binomialCoeff(n - 1, k - 1) +
			binomialCoeff(n - 1, k);
	}

	ArrayXXf arctan2(ArrayXXf arr1, ArrayXXf arr2)
	{
		ArrayXXf temp(arr1.rows(), arr1.cols());

		int k = 0;
		for (int i = 0; i < arr1.cols(); i++)
		{
			for (int j = 0; j < arr1.rows(); j++)
			{
				temp(k) = atan2(arr1(k), arr2(k));
				k++;
			}
		}
		return temp;
	}

	ArrayXXf cumsum(ArrayXXf arr1, ArrayXXf arr2)
	{

		float init = arr1(0);

		for (int i = 0; i < arr1.cols(); i++)
			arr1.col(i) = arr1.col(i) * arr2;

		int k = 1;
		for (int j = 0; j < arr1.cols(); j++)
		{
			for (int i = 1; i < arr1.rows(); i++)
			{
				arr1(k) = arr1(k) + arr1(k - 1);
				k++;
			}
			k++;
		}
		return arr1;
	}
	void shape(ArrayXXf arr)
	{
		cout << arr.rows() << " " << arr.cols();
	}
	float euclidean_dist(float x1, float y1, float x2, float y2)
	{
		return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
	}
	//___________________________________________________________Optim Functions____________________________________________________________________
	three_var bernstein_coeff_order10_new(float n, float tmin, float tmax, ArrayXXf t_actual, int num)
	{
		three_var s;
		float l = tmax - tmin;
		ArrayXXf t = (t_actual - tmin) / l;

		ArrayXXf P(num, (int)n + 1), Pdot(num, (int)n + 1), Pddot(num, (int)n + 1);


		P.col(0) = binomialCoeff(n, 0) * pow(1 - t, n - 0) * pow(t, 0);

		P.col(1) = binomialCoeff(n, 1) * pow(1 - t, n - 1) * pow(t, 1);

		P.col(2) = binomialCoeff(n, 2) * pow(1 - t, n - 2) * pow(t, 2);

		P.col(3) = binomialCoeff(n, 3) * pow(1 - t, n - 3) * pow(t, 3);

		P.col(4) = binomialCoeff(n, 4) * pow(1 - t, n - 4) * pow(t, 4);

		P.col(5) = binomialCoeff(n, 5) * pow(1 - t, n - 5) * pow(t, 5);

		P.col(6) = binomialCoeff(n, 6) * pow(1 - t, n - 6) * pow(t, 6);

		P.col(7) = binomialCoeff(n, 7) * pow(1 - t, n - 7) * pow(t, 7);

		P.col(8) = binomialCoeff(n, 8) * pow(1 - t, n - 8) * pow(t, 8);

		P.col(9) = binomialCoeff(n, 9) * pow(1 - t, n - 9) * pow(t, 9);

		P.col(10) = binomialCoeff(n, 10) * pow(1 - t, n - 10) * pow(t, 10);

		Pdot.col(0) = -10.0 * pow(-t + 1, 9);

		Pdot.col(1) = -90.0 * t * pow(-t + 1, 8) + 10.0 * pow(-t + 1, 9);

		Pdot.col(2) = -360.0 * pow(t, 2) * pow(-t + 1, 7) + 90.0 * t * pow(-t + 1, 8);

		Pdot.col(3) = -840.0 * pow(t, 3) * pow(-t + 1, 6) + 360.0 * pow(t, 2) * pow(-t + 1, 7);

		Pdot.col(4) = -1260.0 * pow(t, 4) * pow(-t + 1, 5) + 840.0 * pow(t, 3) * pow(-t + 1, 6);

		Pdot.col(5) = -1260.0 * pow(t, 5) * pow(-t + 1, 4) + 1260.0 * pow(t, 4) * pow(-t + 1, 5);

		Pdot.col(6) = -840.0 * pow(t, 6) * pow(-t + 1, 3) + 1260.0 * pow(t, 5) * pow(-t + 1, 4);

		Pdot.col(7) = -360.0 * pow(t, 7) * pow(-t + 1, 2) + 840.0 * pow(t, 6) * pow(-t + 1, 3);

		Pdot.col(8) = 45.0 * pow(t, 8) * (2 * t - 2) + 360.0 * pow(t, 7) * pow(-t + 1, 2);

		Pdot.col(9) = -10.0 * pow(t, 9) + 9 * pow(t, 8) * (-10.0 * t + 10.0);

		Pdot.col(10) = 10.0 * pow(t, 9);


		Pddot.col(0) = 90.0 * pow(-t + 1, 8.0);

		Pddot.col(1) = 720.0 * t * pow(-t + 1, 7) - 180.0 * pow(-t + 1, 8);

		Pddot.col(2) = 2520.0 * pow(t, 2) * pow(-t + 1, 6) - 1440.0 * t * pow(-t + 1, 7) + 90.0 * pow(-t + 1, 8);

		Pddot.col(3) = 5040.0 * pow(t, 3) * pow(-t + 1, 5) - 5040.0 * pow(t, 2) * pow(-t + 1, 6) + 720.0 * t * pow(-t + 1, 7);

		Pddot.col(4) = 6300.0 * pow(t, 4) * pow(-t + 1, 4) - 10080.0 * pow(t, 3) * pow(-t + 1, 5) + 2520.0 * pow(t, 2) * pow(-t + 1, 6);

		Pddot.col(5) = 5040.0 * pow(t, 5) * pow(-t + 1, 3) - 12600.0 * pow(t, 4) * pow(-t + 1, 4) + 5040.0 * pow(t, 3) * pow(-t + 1, 5);

		Pddot.col(6) = 2520.0 * pow(t, 6) * pow(-t + 1, 2) - 10080.0 * pow(t, 5) * pow(-t + 1, 3) + 6300.0 * pow(t, 4) * pow(-t + 1, 4);

		Pddot.col(7) = -360.0 * pow(t, 7) * (2 * t - 2) - 5040.0 * pow(t, 6) * pow(-t + 1, 2) + 5040.0 * pow(t, 5) * pow(-t + 1, 3);

		Pddot.col(8) = 90.0 * pow(t, 8) + 720.0 * pow(t, 7) * (2 * t - 2) + 2520.0 * pow(t, 6) * pow(-t + 1, 2);

		Pddot.col(9) = -180.0 * pow(t, 8) + 72 * pow(t, 7) * (-10.0 * t + 10.0);

		Pddot.col(10) = 90.0 * pow(t, 8);

		s.a = P;
		s.b = Pdot / l;
		s.c = Pddot / (l * l);

		return s;
	}
	probData solve(probData prob_data, ArrayXXf P, ArrayXXf Pdot, ArrayXXf Pddot, ArrayXXf x_init, ArrayXXf x_fin, ArrayXXf y_init, ArrayXXf y_fin, ArrayXXf v_init,  ArrayXXf psi_init, ArrayXXf psidot_init, ArrayXXf psi_fin, ArrayXXf psidot_fin, ArrayXXf x_obs, ArrayXXf y_obs, bool warm )
	{
		prob_data.rho_ineq = 1.0;
        prob_data.rho_psi = 1.0;
        prob_data.rho_nonhol = 1.0;
        prob_data.rho_obs = 1.0;

		ArrayXXf vx_init, vy_init, b_eq_x, b_eq_y, b_eq_psi,
		res_psi, res_nonhol, res_obs, res_bounds,
		res_eq_x, res_nonhol_x, res_eq_y, res_nonhol_y, res_x_obs_vec, res_y_obs_vec,
		wc_alpha_ax, ws_alpha_ay, res_ax_vec, res_ay_vec, psi_temp, c_psi, c_x, c_y; ;
		 
		
		vx_init = v_init * cos(psi_init);
		vy_init = v_init * sin(psi_init);

		
		b_eq_x = ArrayXXf(prob_data.num_goal, 3);
		b_eq_x << x_init, vx_init, x_fin;

		b_eq_y = ArrayXXf(prob_data.num_goal, 3);
		b_eq_y << y_init, vy_init, y_fin;

		b_eq_psi = ArrayXXf(prob_data.num_goal, 4);
		b_eq_psi << psi_init, psidot_init, psi_fin, psidot_fin;
		
		res_psi = ones(prob_data.maxiter, 1);
		res_nonhol = ones(prob_data.maxiter, 1);
		res_obs = ones(prob_data.maxiter, 1);
		res_bounds = ones(prob_data.maxiter, 1);

		
		prob_data.v = ones(prob_data.num_goal, prob_data.num).colwise() * v_init.col(0) ;
		prob_data.psi = ones(prob_data.num_goal, prob_data.num).colwise() * psi_init.col(0);
		
		prob_data.xdot = prob_data.v * cos(prob_data.psi);
		prob_data.ydot = prob_data.v * sin(prob_data.psi);
		
		int i = 0;
		// for(int i = 0; i < prob_data.maxiter; i++)
		do
		{
			if(i >= prob_data.maxiter)
				break;
			psi_temp = arctan2(prob_data.ydot, prob_data.xdot);
			three_varr s_xy, s_psi;

			s_psi = compute_psi(prob_data, P, Pdot, Pddot, psi_temp, b_eq_psi);
			c_psi = s_psi.a;
			res_psi(i) = s_psi.b.matrix().lpNorm<2>();
			prob_data = s_psi.c;

			// break;
			s_xy = compute_x(prob_data, P, Pdot, Pddot, b_eq_x, b_eq_y, x_obs, y_obs);	
			c_x = s_xy.a;
			c_y = s_xy.b;
			prob_data = s_xy.c;


			prob_data.v = sqrt(pow(prob_data.xdot, 2) + pow(prob_data.ydot, 2));
			prob_data.v = clip(0.0, 24.0, prob_data.v);
			prob_data.v.col(0) = prob_data.v_init.col(0);

			res_nonhol_x = prob_data.xdot - prob_data.v * cos(prob_data.psi); 
			res_nonhol_y = prob_data.ydot - prob_data.v * sin(prob_data.psi);

			// obs
			ArrayXXf wc_alpha(prob_data.num_goal, prob_data.num_obs*prob_data.num);
			ArrayXXf ws_alpha(prob_data.num_goal, prob_data.num_obs*prob_data.num);
			ArrayXXf randm, randm2(1, prob_data.num_obs*prob_data.num), randm3(1, prob_data.num_obs*prob_data.num);
			for(int i = 0; i < prob_data.num_goal; i++)
			{
				randm = (-prob_data.x_obs).rowwise() + prob_data.x.row(i);
				randm2 = reshape(randm.transpose(), 1, prob_data.num_obs*prob_data.num);  
				wc_alpha.row(i) = randm2;

				randm = (-prob_data.y_obs).rowwise() + prob_data.y.row(i);
				randm3 = reshape(randm.transpose(), 1, prob_data.num_obs*prob_data.num);
				ws_alpha.row(i) = randm3;
			}
			
			prob_data.alpha_obs = arctan2(ws_alpha * prob_data.a_obs, wc_alpha * prob_data.b_obs);

			ArrayXXf c1_d, c2_d, d_temp;
			c1_d = 1.0*prob_data.rho_obs*(pow(prob_data.a_obs, 2) * pow(cos(prob_data.alpha_obs), 2) + pow(prob_data.b_obs, 2)*pow(sin(prob_data.alpha_obs), 2));
			c2_d = 1.0*prob_data.rho_obs*(prob_data.a_obs*wc_alpha*cos(prob_data.alpha_obs) + prob_data.b_obs*ws_alpha*sin(prob_data.alpha_obs));
			d_temp = c2_d/c1_d;
			prob_data.d_obs = maximum(1, d_temp);

			
			res_x_obs_vec = wc_alpha-prob_data.a_obs*prob_data.d_obs*cos(prob_data.alpha_obs);
        	res_y_obs_vec = ws_alpha-prob_data.b_obs*prob_data.d_obs*sin(prob_data.alpha_obs);


			// acc
			wc_alpha_ax = prob_data.xddot;
        	ws_alpha_ay = prob_data.yddot;
        	prob_data.alpha_a = arctan2( ws_alpha_ay, wc_alpha_ax);
			
			ArrayXXf c1_d_a, c2_d_a, d_temp_a;
			c1_d_a = 1.0*prob_data.rho_ineq*(pow(cos(prob_data.alpha_a), 2) + pow(sin(prob_data.alpha_a), 2));
        	c2_d_a = 1.0*prob_data.rho_ineq*(wc_alpha_ax*cos(prob_data.alpha_a) + ws_alpha_ay*sin(prob_data.alpha_a));
			
			d_temp_a = c2_d_a/c1_d_a;
			prob_data.d_a = minimum(prob_data.a_max, d_temp_a);

			res_nonhol_x = prob_data.xdot - prob_data.v * cos(prob_data.psi); 
			res_nonhol_y = prob_data.ydot - prob_data.v * sin(prob_data.psi);

        	res_ax_vec = prob_data.xddot-prob_data.d_a*cos(prob_data.alpha_a);
        	res_ay_vec = prob_data.yddot-prob_data.d_a*sin(prob_data.alpha_a);


			prob_data.lamda_x = prob_data.lamda_x-prob_data.rho_nonhol *(prob_data.A_nonhol.transpose().matrix() * res_nonhol_x.transpose().matrix()).transpose().array()
								-prob_data.rho_obs * (prob_data.A_obs.transpose().matrix() * res_x_obs_vec.transpose().matrix()).transpose().array()
								-prob_data.rho_ineq * (prob_data.A_acc.transpose().matrix() * res_ax_vec.transpose().matrix()).transpose().array();

			prob_data.lamda_y = prob_data.lamda_y-prob_data.rho_nonhol* (prob_data.A_nonhol.transpose().matrix() * res_nonhol_y.transpose().matrix()).transpose().array()
								-prob_data.rho_obs * (prob_data.A_obs.transpose().matrix() * res_y_obs_vec.transpose().matrix()).transpose().array()
								-prob_data.rho_ineq * (prob_data.A_acc.transpose().matrix() * res_ay_vec.transpose().matrix()).transpose().array();

			res_obs(i) = stack(res_x_obs_vec, res_y_obs_vec, 'v').matrix().lpNorm<2>();
			res_bounds(i) = stack(res_ax_vec, res_ay_vec, 'v').matrix().lpNorm<2>();
			res_nonhol(i) = stack(res_nonhol_x, res_nonhol_y, 'v').matrix().lpNorm<2>();
			
			prob_data.rho_obs = prob_data.rho_obs*1.06;
			prob_data.rho_nonhol = prob_data.rho_nonhol*1.06;
			prob_data.rho_psi = prob_data.rho_psi*1.06;
			prob_data.rho_ineq = prob_data.rho_ineq*1.06;

			i++;
		} while(1);//res_obs(i-1) >= 0.1 || res_bounds(i - 1) >= 0.001 || res_nonhol(i - 1) >= 0.1);
		// cout << res_obs(i-1) <<  " " << res_bounds(i-1) << " " << res_nonhol(i-1) << endl;
		prob_data.res_bounds = res_bounds;
		prob_data.res_nonhol = res_nonhol;
		prob_data.res_obs = stack(res_x_obs_vec, res_y_obs_vec, 'h');
		// shape(res_obs);

		prob_data.v_controls = sqrt(pow((prob_data.Pdot_upsample.matrix() * c_x.matrix().transpose()).transpose().array(), 2)
								+ pow((prob_data.Pdot_upsample.matrix() * c_y.matrix().transpose()).transpose().array(), 2));
		prob_data.w_controls = (prob_data.Pdot_upsample.matrix() * c_psi.matrix().transpose()).transpose();
		return prob_data;
	}
	three_varr compute_x(probData prob_data, ArrayXXf P, ArrayXXf Pdot, ArrayXXf Pddot, ArrayXXf b_eq_x, ArrayXXf b_eq_y, ArrayXXf x_obs, ArrayXXf y_obs)
	{
		three_varr s;

		ArrayXXf temp_x_obs, temp_y_obs, b_obs_x, b_obs_y, b_nonhol_x, b_nonhol_y, b_ax_ineq, b_ay_ineq,
		cost, cost_mat_inv, lincost_x, lincost_y, sol_x, sol_y, primal_sol_x, primal_sol_y;
		MatrixXf cost_mat;


		temp_x_obs = prob_data.d_obs * cos(prob_data.alpha_obs) * prob_data.a_obs;
		temp_y_obs = prob_data.d_obs * sin(prob_data.alpha_obs) * prob_data.b_obs;

		ArrayXXf randm1(prob_data.num*prob_data.num_obs, 1), randm2(prob_data.num*prob_data.num_obs, 1);
		
		randm1 = reshape(prob_data.x_obs.transpose(), prob_data.num*prob_data.num_obs, 1);
		randm2 = reshape(prob_data.y_obs.transpose(), prob_data.num*prob_data.num_obs, 1);

		b_obs_x = temp_x_obs.rowwise() + randm1.col(0).transpose();  
		b_obs_y = temp_y_obs.rowwise() + randm2.col(0).transpose();

		b_nonhol_x = prob_data.v * cos(prob_data.psi);
		b_nonhol_y = prob_data.v * sin(prob_data.psi);

		b_ax_ineq = prob_data.d_a * cos(prob_data.alpha_a);
		b_ay_ineq = prob_data.d_a * sin(prob_data.alpha_a);

		
		cost = prob_data.cost_smoothness+prob_data.rho_nonhol*(prob_data.A_nonhol.transpose().matrix() * prob_data.A_nonhol.matrix()).array()
				+ prob_data.rho_obs*(prob_data.A_obs.transpose().matrix() * prob_data.A_obs.matrix()).array()
				+ prob_data.rho_ineq*(prob_data.A_acc.transpose().matrix() * prob_data.A_acc.matrix()).array();

		// cout << "cost=" << cost.matrix().lpNorm<2>() << endl;
		cost_mat = stack(stack(cost, prob_data.A_eq.transpose(), 'h'), stack(prob_data.A_eq, 0.0*ones(prob_data.A_eq.rows(), prob_data.A_eq.rows()), 'h'), 'v');
		MatrixXf I(cost_mat.rows(), cost_mat.cols());
	    I.setIdentity();
	    cost_mat_inv = (cost_mat).householderQr().solve(I);
		// Ax = I; x = A-1
		
		lincost_x = -prob_data.lamda_x-prob_data.rho_nonhol * (prob_data.A_nonhol.transpose().matrix() * b_nonhol_x.transpose().matrix()).transpose().array()
					-prob_data.rho_obs * (prob_data.A_obs.transpose().matrix() * b_obs_x.transpose().matrix()).transpose().array()
					-prob_data.rho_ineq * (prob_data.A_acc.transpose().matrix() * b_ax_ineq.transpose().matrix()).transpose().array();

		lincost_y = -prob_data.lamda_y-prob_data.rho_nonhol * (prob_data.A_nonhol.transpose().matrix() * b_nonhol_y.transpose().matrix()).transpose().array()
					-prob_data.rho_obs * (prob_data.A_obs.transpose().matrix() * b_obs_y.transpose().matrix()).transpose().array()
					-prob_data.rho_ineq * (prob_data.A_acc.transpose().matrix() * b_ay_ineq.transpose().matrix()).transpose().array();

		
		sol_x = (cost_mat_inv.matrix() * stack(-lincost_x, b_eq_x, 'h').matrix().transpose()).transpose();
		primal_sol_x = sol_x.leftCols(prob_data.nvar);
		prob_data.x = (P.matrix() * primal_sol_x.matrix().transpose()).transpose();
    	prob_data.xdot = (Pdot.matrix() * primal_sol_x.matrix().transpose()).transpose();
		prob_data.xddot = (Pddot.matrix() * primal_sol_x.matrix().transpose()).transpose();

		sol_y = (cost_mat_inv.matrix() * stack(-lincost_y, b_eq_y, 'h').matrix().transpose()).transpose();
		primal_sol_y = sol_y.leftCols(prob_data.nvar);
		prob_data.y = (P.matrix() * primal_sol_y.matrix().transpose()).transpose();
    	prob_data.ydot = (Pdot.matrix() * primal_sol_y.matrix().transpose()).transpose();
		prob_data.yddot = (Pddot.matrix() * primal_sol_y.matrix().transpose()).transpose();

		s.a = primal_sol_x;
		s.b = primal_sol_y;
		s.c = prob_data;
		return s;
	}
	three_varr compute_psi(probData prob_data, ArrayXXf P, ArrayXXf Pdot, ArrayXXf Pddot, ArrayXXf psi_temp, ArrayXXf b_eq_psi)
	{
		three_varr s;
		ArrayXXf cost, cost_mat_inv, lincost_psi, sol_psi, primal_sol_psi, res_psi;
		MatrixXf cost_mat;
	

		cost = prob_data.cost_smoothness_psi + prob_data.rho_psi * (prob_data.A_psi.transpose().matrix() * prob_data.A_psi.matrix()).array();	
		cost_mat = stack(stack(cost, prob_data.A_eq_psi.transpose(), 'h'), stack(prob_data.A_eq_psi, 0.0*ones(prob_data.A_eq_psi.rows(), prob_data.A_eq_psi.rows()), 'h'), 'v');
		MatrixXf I(cost_mat.rows(), cost_mat.cols());
	    I.setIdentity();
	    cost_mat_inv = (cost_mat).householderQr().solve(I);

		lincost_psi = -prob_data.lamda_psi - prob_data.rho_psi * (prob_data.A_psi.transpose().matrix() * psi_temp.transpose().matrix()).transpose().array();

		sol_psi = (cost_mat_inv.matrix() * stack(-lincost_psi, b_eq_psi, 'h').matrix().transpose()).transpose();
		primal_sol_psi = sol_psi.leftCols(prob_data.nvar);

		prob_data.psi = (P.matrix() * primal_sol_psi.transpose().matrix()).transpose();
		prob_data.psidot = (Pdot.matrix() * primal_sol_psi.transpose().matrix()).transpose();
		prob_data.psiddot = (Pddot.matrix() * primal_sol_psi.transpose().matrix()).transpose();

		res_psi = (prob_data.A_psi.matrix() * primal_sol_psi.transpose().matrix()).transpose().array() - psi_temp;
		
		prob_data.lamda_psi = prob_data.lamda_psi - prob_data.rho_psi * (prob_data.A_psi.transpose().matrix() * res_psi.transpose().matrix()).transpose().array();

		s.a = primal_sol_psi;
		s.b = res_psi;
		s.c = prob_data;
		return s;
		
	}
	
	probData mpc(probData prob_data, three_var PPP, ArrayXXf x_g, ArrayXXf y_g, float x_init, float y_init, float v_init, float psi_init, float psidot_init, ArrayXXf x_obs_temp, ArrayXXf y_obs_temp, ArrayXXf vx_obs, ArrayXXf vy_obs, bool warm)
	{
		four_var s;
		// if(warm)
		{
			prob_data = initializeArrays(prob_data);
			prob_data.lamda_x = 0 * ones(prob_data.num_goal, prob_data.nvar);
			prob_data.lamda_y = 0 * ones(prob_data.num_goal, prob_data.nvar);
			prob_data.lamda_psi = 0 * ones(prob_data.num_goal, prob_data.nvar);
			prob_data.d_a = prob_data.a_max * ones(prob_data.num_goal, prob_data.num);
			prob_data.alpha_a = 0.0 * ones(prob_data.num_goal, prob_data.num);
		}
		prob_data.x_init = x_init;
		prob_data.y_init = y_init;
		prob_data.x_fin = x_g;
		prob_data.y_fin = y_g;

		prob_data.psi_init = psi_init * ones(prob_data.num_goal, 1);
		prob_data.psi_fin = 0.0*(91.0 * M_PI / 180.0) * ones(prob_data.num_goal, 1);
		prob_data.psidot_init = psidot_init * ones(prob_data.num_goal, 1);
		prob_data.psidot_fin = 0.0 * ones(prob_data.num_goal, 1);
		prob_data.v_init = v_init * ones(prob_data.num_goal, 1);


		ArrayXXf numbers(prob_data.num_obs, prob_data.num);
		for(int i = 0; i < prob_data.num_obs; i++)
			numbers.row(i).setLinSpaced(prob_data.num, 0, prob_data.num);

		prob_data.x_obs = (ones(prob_data.num_obs, prob_data.num)).colwise() * (x_obs_temp).col(0) + (numbers.colwise() * vx_obs.col(0) * prob_data.t);
		prob_data.y_obs = (ones(prob_data.num_obs, prob_data.num)).colwise() * (y_obs_temp).col(0) + (numbers.colwise() * vy_obs.col(0) * prob_data.t);
		

		// if(warm)
		{
			prob_data.x_guess = ArrayXXf(prob_data.num_goal, prob_data.num);
			prob_data.y_guess = ArrayXXf(prob_data.num_goal, prob_data.num);
			for(int i = 0; i < prob_data.x_guess.rows(); i++)
			{
				prob_data.x_guess.row(i).setLinSpaced(prob_data.num, x_init, x_g(i));
				prob_data.y_guess.row(i).setLinSpaced(prob_data.num, y_init, y_g(i));
			}
		}
		// else
		// {
		// 	prob_data.x_guess = prob_data.x;
		// 	prob_data.y_guess = prob_data.y;
		// }
		// cout << "Good till here" << endl;	
		// if(warm)	
			prob_data = initialize_guess_alpha(prob_data);
		prob_data = solve(prob_data, PPP.a, PPP.b, PPP.c, prob_data.x_init, prob_data.x_fin, prob_data.y_init, prob_data.y_fin, prob_data.v_init,  prob_data.psi_init, prob_data.psidot_init, prob_data.psi_fin, 
					prob_data.psidot_fin, prob_data.x_obs, prob_data.y_obs, warm);		

		return prob_data;
	}
	probData initialize_guess_alpha(probData prob_data)
	{
		ArrayXXf c1_d, c2_d, d_temp;
		ArrayXXf wc_alpha(prob_data.num_goal, prob_data.num_obs*prob_data.num);
		ArrayXXf ws_alpha(prob_data.num_goal, prob_data.num_obs*prob_data.num);
		ArrayXXf randm, randm2(1, prob_data.num_obs*prob_data.num), randm3(1, prob_data.num_obs*prob_data.num);

		for(int i = 0; i < prob_data.num_goal; i++)
		{	
			randm = (-prob_data.x_obs).rowwise() + prob_data.x_guess.row(i);
			randm2 = reshape(randm.transpose(), 1, prob_data.num_obs*prob_data.num);
			wc_alpha.row(i) = randm2;

			randm = (-prob_data.y_obs).rowwise() + prob_data.y_guess.row(i);
			randm3 = reshape(randm.transpose(), 1, prob_data.num_obs*prob_data.num);
			ws_alpha.row(i) = randm3;
		}

		prob_data.alpha_obs = arctan2(ws_alpha * prob_data.a_obs, wc_alpha * prob_data.b_obs);

		// cout << maximum(1, (wc_alpha + ws_alpha)/(prob_data.a_obs*cos(prob_data.alpha_obs) + prob_data.b_obs*sin(prob_data.alpha_obs))).matrix().lpNorm<2>();
		
		c1_d = 1.0*prob_data.rho_obs*(pow(prob_data.a_obs, 2) * pow(cos(prob_data.alpha_obs), 2) + pow(prob_data.b_obs, 2)*pow(sin(prob_data.alpha_obs), 2));
		c2_d = 1.0*prob_data.rho_obs*(prob_data.a_obs*wc_alpha*cos(prob_data.alpha_obs) + prob_data.b_obs*ws_alpha*sin(prob_data.alpha_obs));
		d_temp = c2_d/c1_d;
		
		
		prob_data.d_obs = maximum(1.000000000001, d_temp);
		// shape(prob_data.d_obs);
		// cout << "d_obs=" << prob_data.d_obs.matrix().lpNorm<2>() << endl;

		return prob_data;
	}
	three_var compute_bernstein(ArrayXXf tot_time, float t_fin, int num)
	{
		three_var PPP;
		PPP = bernstein_coeff_order10_new(10.0, tot_time(0), t_fin, tot_time, num);
		return PPP;
	}
	probData initializeArrays(probData prob_data)
	{

		prob_data.alpha_obs = ArrayXXf(prob_data.num_goal, prob_data.num_obs * prob_data.num);
		prob_data.d_obs = ArrayXXf(prob_data.num_goal, prob_data.num_obs * prob_data.num);
		prob_data.lamda_x = ArrayXXf(prob_data.num_goal, prob_data.nvar);
		prob_data.lamda_y = ArrayXXf(prob_data.num_goal, prob_data.nvar);
		prob_data.lamda_psi = ArrayXXf(prob_data.num_goal, prob_data.nvar);
		prob_data.d_a = ArrayXXf(prob_data.num_goal, prob_data.num);
		prob_data.alpha_a = ArrayXXf(prob_data.num_goal, prob_data.num);

		prob_data.x_init = ArrayXXf(prob_data.num_goal, 1);
		prob_data.y_init = ArrayXXf(prob_data.num_goal, 1);
		prob_data.x_fin = ArrayXXf(prob_data.num_goal, 1);
		prob_data.y_fin = ArrayXXf(prob_data.num_goal, 1);
		prob_data.psi_init = ArrayXXf(prob_data.num_goal, 1);
		prob_data.psi_fin = ArrayXXf(prob_data.num_goal, 1);
		prob_data.psidot_init = ArrayXXf(prob_data.num_goal, 1);
		prob_data.psidot_fin = ArrayXXf(prob_data.num_goal, 1);
		prob_data.v_init = ArrayXXf(prob_data.num_goal, 1);

		return prob_data;
	}
}
