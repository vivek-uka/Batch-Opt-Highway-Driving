#define _USE_MATH_DEFINES
#include <memory>
#include <iostream>
#include <time.h>
#include <unistd.h>
#include <fstream>
#include "yaml-cpp/yaml.h"

#include "acado_common.h"
#include "acado_auxiliary_functions.h"
#include <thread>
#include <Eigen/Dense>

#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/rclcpp.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "geometry_msgs/msg/vector3.hpp"
#include "msgs_car/msg/controls.hpp"
#include "msgs_car/msg/states.hpp"	

// interface.cpp
// solver.c
// common.h

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */
#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */
#define N           ACADO_N   /* Number of intervals in the horizon. */
#define VERBOSE		0
#define pi M_PI

using namespace std;
using namespace Eigen;
using std::placeholders::_1;

extern "C"{
__thread ACADOvariables acadoVariables;
__thread ACADOworkspace acadoWorkspace;
}


class MinimalPublisher : public rclcpp::Node
{
	public: 

	int num_goal, num_obs, num;
	float dt, x_init, y_init, v_init, theta_init, thetadot_init, total_time, speed, avg_time, avg_speed;
	float prev_v_send, prev_w_send, a_obs, b_obs, w0, w1, v_des;

	int cnt = 1, loop = -1, setting;
	ofstream outdata, outdata2;
	ArrayXXf lane;
	ArrayXXf x_obs_temp, y_obs_temp, vx_obs, vy_obs, warm, Gotit, x_g, y_g, meta_cost;
	ArrayXXf batch_x, batch_y, batch_v, batch_w, batch_time, batch_theta, 
				batch_optimal, batch_dist, batch_a, batch_j, old, batch_res_obs; 
	MinimalPublisher(): Node("minimal_publisher"), count_(0)
    {
		speed = 0.0;
		total_time = 0.0;
		avg_speed = 0.0;
		avg_time = 0.0;
		prev_v_send = 0.0;
		prev_w_send = 0.0;

		num_obs = 6;
		num = 100;
		num_goal = 1;

		
		dt = 0.08;
		x_init = 0.0;
		y_init = 10.0;
		theta_init = 0.0;

		warm = ArrayXXf(num_goal, 1);
		Gotit = ArrayXXf(num_goal, 1);	
		warm = 0;
		Gotit = 0;

		x_obs_temp = ArrayXXf(num_obs, 1);
        y_obs_temp = ArrayXXf(num_obs, 1);
        vx_obs = ArrayXXf(num_obs, 1);
        vy_obs = ArrayXXf(num_obs, 1);
		old = ArrayXXf(num_goal, 1);

		batch_dist = ArrayXXf(num_goal, 1);
		batch_optimal = ArrayXXf(num_goal, 1);
		batch_time = ArrayXXf(num_goal, 1);
		batch_x = ArrayXXf(num_goal, 100);
		batch_y = ArrayXXf(num_goal, 100);
		batch_v = ArrayXXf(num_goal, 100);
		batch_w = ArrayXXf(num_goal, 100);	
		batch_theta = ArrayXXf(num_goal, 100);
		batch_a = ArrayXXf(num_goal, 100);
		batch_j = ArrayXXf(num_goal, 100);
		batch_res_obs = ArrayXXf(num_goal, 1);
		

		batch_time = 10000000000000;

        x_obs_temp = 0;
        y_obs_temp = 0;
		vx_obs = 0;
        vy_obs = 0.0;
		a_obs = 5.6;
		b_obs = 3.0;

		x_g = ArrayXXf(num_goal, 1);
        y_g = ArrayXXf(num_goal, 1);

		meta_cost = ArrayXXf(num_goal, 9);
        meta_cost = -1;

		

		x_g = 120;
		y_g = -10;
		old = x_g;
		subscription_ = this->create_subscription<msgs_car::msg::States>(
        "ego_vehicle_obs", 10, std::bind(&MinimalPublisher::topic_callback, this, _1));

        publisher_ = this->create_publisher<msgs_car::msg::Controls>("ego_vehicle_cmds", 10);
        timer_ = this->create_wall_timer(10ms, bind(&MinimalPublisher::timer_callback, this));

		YAML::Node map = YAML::LoadFile("src/mpc_car_acado_single/config.yaml");
		string setting = map["setting"].as<string>();
		w0 = map["configuration"][setting]["weights"][0].as<float>();
        w1 = map["configuration"][setting]["weights"][1].as<float>();
		v_des = map["configuration"][setting]["v_des"].as<float>();

		outdata.open(map["configuration"][setting]["file"].as<string>());
		thread Thread[num_goal];	
		for(int i = 0; i < num_goal; i++)
			Thread[i] = thread(&MinimalPublisher::optimize, this, i);
		for(int i = 0; i < num_goal; i++)
			Thread[i].detach();
		

	}
	private:
    void topic_callback(const msgs_car::msg::States::SharedPtr msg)
    {   
		if(Gotit.maxCoeff() == 0)
		{
			x_init = msg->x[0];
			y_init = msg->y[0];
			v_init = sqrt(msg->vx[0] * msg->vx[0] + msg->vy[0] * msg->vy[0]);
			theta_init = msg->psi[0];
			thetadot_init = msg->psidot; 
			
			x_obs_temp << msg->x[1], msg->x[2], msg->x[3], msg->x[4], msg->x[5], msg->x[6];
			y_obs_temp << msg->y[1], msg->y[2], msg->y[3], msg->y[4], msg->y[5], msg->y[6];
			vx_obs << msg->vx[1], msg->vx[2], msg->vx[3], msg->vx[4], msg->vx[5], msg->vx[6];
			vy_obs << msg->vy[1], msg->vy[2], msg->vy[3], msg->vy[4], msg->vy[5], msg->vy[6];
			RCLCPP_INFO(this->get_logger(),"Got feedback");
			Gotit = 1;

			if(loop == -1)
				prev_v_send = v_init;	
		}
	}
	void timer_callback();
	void optimize(int info);
	rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<msgs_car::msg::States>::SharedPtr subscription_;
    rclcpp::Publisher<msgs_car::msg::Controls>::SharedPtr publisher_;
    size_t count_;
};


void MinimalPublisher :: timer_callback()
{
    auto message = msgs_car::msg::Controls();
	
	int done = 1;
	for(int i = 0; i < num_goal; i++)
	{
		if(warm(i) == 0)
		{	
			done = 0;
			break;
		}
		
	}
	if(done)
	{
		loop++;
		if(loop > 0)
		{	
			total_time +=  batch_time.maxCoeff();
			avg_time = total_time/loop;
		}
		// if(x_init > 5*cnt) // 5 for rightmost + max
		{
			// x_g << 100, 110, 120, 130, 140, 150, 160, 170, 180, 190;
			// x_g << 120, 120, 120, 120, 120, 120, 130, 130, 130, 130;
			x_g = old + x_init;
			cnt+=1;
		}
		
		// 50 5 1 for 11 goals hsrl

		int index = 0;
		
		message.w = batch_w.row(index).leftCols(3).mean();
		message.v = batch_v.row(index).leftCols(3).mean();
		message.index = index;
		message.goals = num_goal;
		
		outdata << x_init << " " << y_init << " " << theta_init << " " <<message.v << " " << message.w
                << " " << (message.v - prev_v_send)/dt << " " << (message.w - prev_w_send)/dt
                << " " << batch_time.maxCoeff() << " " << loop+1 << " " << index << endl;
		// outdata2 << batch_x << " " << batch_y << endl;
		prev_v_send = message.v;
		prev_w_send = message.w;
		speed += message.v;
		avg_speed = speed/(loop + 2);

		for(int i = 0; i < num_goal; i++)
		{
			for(int j = 0; j < num; j++)
			{
				geometry_msgs::msg::Pose pose;
				pose.position.x = batch_x(i, j);
				pose.position.y = batch_y(i, j);
				message.batch.poses.push_back(pose);    
			}
		}
		publisher_->publish(message);
		done = 0;
		warm = 0;
	}
	// if(loop > 0)
	// {
	// 	RCLCPP_INFO(this->get_logger(),"Time taken = %f Average speed = %f", avg_time, avg_speed);
	// }	
	
}

void MinimalPublisher :: optimize(int info)
{
	// float x_g = info -> x_g, y_g = info -> y_g, theta_g = info -> theta_g;
	int id = info, NUM_STEPS = 15, first = 0;
	if(1 - first)
		acado_initializeSolver();
	while(1)
	{	
		if(!Gotit(id)){
			sleep(0.001);
			continue;
		}
		acado_timer t, t2;
		for (int i = 0; i < N * (1 - first); ++i)  
		{
			acadoVariables.u[ i*NU + 0 ] = 0.0;		
			acadoVariables.u[ i*NU + 1 ] = 0.0;		
		}	
		for (int i = 0; i < N * (1 - first); ++i)  
		{
			acadoVariables.y[ i*NY + 0 ] = x_g(id);
			acadoVariables.y[ i*NY + 1 ] = y_g(id);
			acadoVariables.y[ i*NY + 2 ] = v_des;

			acadoVariables.y[ i*NY + 3 ] = 0.0;		
			acadoVariables.y[ i*NY + 4 ] = 0.0;
			
			acadoVariables.y[ i*NY + 5 ] = 0.0;
			acadoVariables.y[ i*NY + 6 ] = 0.0;
			acadoVariables.y[ i*NY + 7 ] = 0.0;
			acadoVariables.y[ i*NY + 8 ] = 0.0;
			acadoVariables.y[ i*NY + 9 ] = 0.0;	
			acadoVariables.y[ NY*i + 10] = 0.0;
		}
		acadoVariables.yN[ 0 ] = x_g(id);
		acadoVariables.yN[ 1 ] = y_g(id);
		acadoVariables.yN[ 2 ] = 0.0;					
		acadoVariables.yN[ 3 ] = 0.0;	

		acadoVariables.x0[ 0 ] = x_init;
		acadoVariables.x0[ 1 ] = y_init;
		acadoVariables.x0[ 2 ] = theta_init;		// theta
		acadoVariables.x0[ 3 ] = v_init;			// v
		acadoVariables.x0[ 4 ] = thetadot_init;		// w
			
		for (int i = 0; i < (N + 1)*(1 - first); i++)  
		{
			acadoVariables.x[ i*NX + 0 ] = acadoVariables.x0[ 0 ];	//x
			acadoVariables.x[ i*NX + 1 ] = acadoVariables.x0[ 1 ];	//y
			acadoVariables.x[ i*NX + 2 ] = acadoVariables.x0[ 2 ];	//theta
			acadoVariables.x[ i*NX + 3 ] = acadoVariables.x0[ 3 ];	//v
			acadoVariables.x[ i*NX + 4 ] = acadoVariables.x0[ 4 ];	//w
		} 
		

		for (int i = 0; i < (N + 1); ++i)
		{
			acadoVariables.od[i * NOD + 0] = x_obs_temp(0) + vx_obs(0) * dt * i;
			acadoVariables.od[i * NOD + 1] = y_obs_temp(0) + vy_obs(0) * dt * i;
			acadoVariables.od[i * NOD + 2] = x_obs_temp(1) + vx_obs(1) * dt * i;
			acadoVariables.od[i * NOD + 3] = y_obs_temp(1) + vy_obs(1) * dt * i;
			acadoVariables.od[i * NOD + 4] = x_obs_temp(2) + vx_obs(2) * dt * i;
			acadoVariables.od[i * NOD + 5] = y_obs_temp(2) + vy_obs(2) * dt * i;
			acadoVariables.od[i * NOD + 6] = x_obs_temp(3) + vx_obs(3) * dt * i;
			acadoVariables.od[i * NOD + 7] = y_obs_temp(3) + vy_obs(3) * dt * i;
			acadoVariables.od[i * NOD + 8] = x_obs_temp(4) + vx_obs(4) * dt * i;
			acadoVariables.od[i * NOD + 9] = y_obs_temp(4) + vy_obs(4) * dt * i;
			acadoVariables.od[i * NOD + 10] = x_obs_temp(5) + vx_obs(5) * dt * i;
			acadoVariables.od[i * NOD + 11] = y_obs_temp(5) + vy_obs(5) * dt * i;
		}
		
		for (int i = 0; i < N * (1 - first); i++)
		{
			acadoVariables.W[NY*NY*i + (NY+1)*0] = 0.0;
			acadoVariables.W[NY*NY*i + (NY+1)*1] = w0;
			acadoVariables.W[NY*NY*i + (NY+1)*2] = w1;				// cruise 6.5*1e3, hsrl=5e1,1e2, rl 5e1 pn sythetic
																				// ngsim hrsl 5e1, 1e3 for ngsim2

			acadoVariables.W[NY*NY*i + (NY+1)*3] = 7*1e3;	// a
			acadoVariables.W[NY*NY*i + (NY+1)*4] = 7*1e6;	// j
			acadoVariables.W[NY*NY*i + (NY+1)*5] = 3*2.8*1e4;
			acadoVariables.W[NY*NY*i + (NY+1)*6] = 3*2.8*1e4;
			acadoVariables.W[NY*NY*i + (NY+1)*7] = 3*2.8*1e4;
			acadoVariables.W[NY*NY*i + (NY+1)*8] = 3*2.8*1e4;
			acadoVariables.W[NY*NY*i + (NY+1)*9] = 3*2.8*1e4;
			acadoVariables.W[NY*NY*i + (NY+1)*10] = 3*2.8*1e4;
		}

		acadoVariables.WN[(NYN+1)*0] = 0*1e9;		// x terminal
		acadoVariables.WN[(NYN+1)*1] = 0*1e9;		// y terminal
		acadoVariables.WN[(NYN+1)*2] = 1e9;		// theta terminal
		acadoVariables.WN[(NYN+1)*3] = 1e6;	

		// acado_preparationStep();
		

		acado_tic( &t );
		for(int iter = 0; iter < NUM_STEPS; ++iter)
		{
			acado_preparationStep();
			acado_feedbackStep( );
			// if(NUM_STEPS == 1)
			// {
			// 	acado_shiftStates(2, 0, 0);
        	// 	acado_shiftControls( 0 );
			// }				
		}
		real_t te = acado_toc( &t );
		batch_time(id) = te;
		
		for(int i = 0; i < N; i++)
		{
			batch_x(id, i) = acadoVariables.x[i*NX + 0];
			batch_y(id, i) = acadoVariables.x[i*NX + 1];
			
			batch_theta(id, i) = 0.0;
			if(abs(batch_theta(id, i)) > 13.0 * M_PI/180.0)
				batch_theta(id, i) = acadoVariables.x[i*NX + 2];
			batch_v(id, i) = acadoVariables.x[i*NX + 3];
			batch_w(id, i) = acadoVariables.x[i*NX + 4]; 

			batch_dist(id) += sqrt(pow((x_obs_temp - batch_x(id, i)), 2) + pow((y_obs_temp - batch_y(id, i)), 2)).matrix().lpNorm<2>(); 
			batch_res_obs(id) += 1/(pow((x_obs_temp - batch_x(id, i))/a_obs, 2) + pow((y_obs_temp - batch_y(id, i))/b_obs, 2) - 1.0).matrix().lpNorm<2>();
			batch_a(id, i) = acadoVariables.u[i*NU + 0];
			batch_j(id, i) = acadoVariables.u[i*NU + 1];
		}
		
		warm(id) = 1;
		Gotit(id) = 0;
		first = 0;
		batch_optimal(id) = acado_getKKT();
		NUM_STEPS = 6;
		// if(NUM_STEPS != 5)
		// 	cout << "Thread complete= "<< id << " KKT Tol= " << acado_getKKT() << endl;
		
	}
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}