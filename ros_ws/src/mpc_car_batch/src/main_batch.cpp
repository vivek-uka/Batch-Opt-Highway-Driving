#define _USE_MATH_DEFINES
#include <memory>
#include <iostream>
#include <time.h>
#include <fstream>
#include <cstdlib>
#include <iomanip> 
#include <cmath>
#include <Eigen/Dense>
#include "mpc_car_batch/optim_batch.h"


#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/rclcpp.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "geometry_msgs/msg/vector3.hpp"
#include "msgs_car/msg/controls.hpp"
#include "msgs_car/msg/states.hpp"

#define MPC 0
using namespace std;
using namespace optim;
using std::placeholders::_1;

class MinimalPublisher : public rclcpp::Node
{
  public:
        
        ArrayXXf lane, tot_time, x_g, y_g, tot_time_up;
        ArrayXXf x_obs_temp, y_obs_temp, vx_obs, vy_obs, old, meta_cost;

        three_var PPP, PPP_up;
        probData prob_data;
        float x_init, y_init, v_init, psi_init, psidot_init, total_time, speed, avg_time, avg_speed;
        float prev_v_send, prev_w_send;

        bool Gotit, warm;

        float min = 1000000;
        int index, cnt, loop;
        clock_t start, end;
        ofstream outdata, outdata2, outdata3, outdata4;

    MinimalPublisher(): Node("minimal_publisher"), count_(0)
    {
        Gotit = false;
        warm = true;

        avg_speed = 0.0;
        speed = 0.0;
        total_time = 0.0;
        avg_time = 0.0;
        index = 0;
        loop = 0;
        prev_v_send = 0.0;
        prev_w_send = 0.0;

        x_init = 0.0;
        y_init = 10.0;
        v_init = 15.0;
        psi_init = 0.0;
        psidot_init = 0.0;

        prob_data.t_fin = 8.0;
        prob_data.num = 100;
        prob_data.t = prob_data.t_fin/prob_data.num;
        prob_data.weight_smoothness = 2.5;
        prob_data.weight_psi = 5;
        prob_data.maxiter = 100;
        prob_data.num_obs = 6;
        prob_data.v_max = 24.0;
        prob_data.a_max = 4.0;
        prob_data.a_obs = 5.6;
        prob_data.b_obs = 3.0;
        prob_data.rho_ineq = 1.0;
        prob_data.rho_psi = 1.0;
        prob_data.rho_nonhol = 1.0;
        prob_data.rho_obs = 1.0;
        prob_data.weight_smoothness_psi = 5.0;

        tot_time = ArrayXXf(prob_data.num, 1);
        tot_time.col(0).setLinSpaced(prob_data.num, 0.0, prob_data.t_fin);

        PPP = compute_bernstein(tot_time, prob_data.t_fin, prob_data.num);
        prob_data.nvar = PPP.a.cols();

        //__________________________________________________________________________
        tot_time_up = ArrayXXf((int)(prob_data.t_fin/0.008), 1);
        tot_time_up.col(0).setLinSpaced(prob_data.t_fin/0.008, 0.0, prob_data.t_fin);
        
        PPP_up = compute_bernstein(tot_time_up, prob_data.t_fin, prob_data.t_fin/0.008);
        prob_data.Pdot_upsample = PPP_up.b;
        //__________________________________________________________________________
        
        prob_data.cost_smoothness = prob_data.weight_smoothness * PPP.c.transpose().matrix() * PPP.c.matrix();
        prob_data.cost_smoothness_psi = prob_data.weight_smoothness_psi * PPP.c.transpose().matrix() * PPP.c.matrix();
        prob_data.lincost_smoothness_psi = 0 * ones(prob_data.nvar, 1);

        prob_data.A_eq = ArrayXXf(3, prob_data.nvar);
        prob_data.A_eq_psi = ArrayXXf(4, prob_data.nvar);
        
        prob_data.A_eq << PPP.a.row(0), PPP.b.row(0), PPP.a.row(PPP.a.rows() - 1);
        prob_data.A_eq_psi << PPP.a.row(0), PPP.b.row(0), PPP.a.row(PPP.a.rows() - 1), PPP.b.row(PPP.b.rows() - 1);

        prob_data.A_nonhol = PPP.b;
        prob_data.A_psi = PPP.a;
        prob_data.A_ineq = PPP.c;
        prob_data.A_acc = PPP.c;
        
        prob_data.A_obs = stack(PPP.a, PPP.a, 'v');
        for (int i = 0; i < prob_data.num_obs - 2; i++)
            prob_data.A_obs = stack(prob_data.A_obs, PPP.a, 'v');
        

        x_obs_temp = ArrayXXf(prob_data.num_obs, 1);
        y_obs_temp = ArrayXXf(prob_data.num_obs, 1);
        vx_obs = ArrayXXf(prob_data.num_obs, 1);
        vy_obs = ArrayXXf(prob_data.num_obs, 1);

        x_obs_temp = 0;
        y_obs_temp = 0;
        vx_obs = 0;
        vy_obs = 0.0;

    
        prob_data.num_goal = 11;
        x_g = ArrayXXf(prob_data.num_goal, 1);
        y_g = ArrayXXf(prob_data.num_goal, 1);

        meta_cost = ArrayXXf(prob_data.num_goal, 9);
        meta_cost = -1;

        lane = ArrayXXf(11, 1);
		lane << -10, -8, -6, -4, -2, 0, 2, 4, 6, 8, 10;
		// lane << -10, -9.5, -9, -6, -4, -2, 2, 4, 6, 8, 10;
        // x_g = 120;//85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 135;
        // y_g = -10;  
		for(int i = 0, m = -1; i < prob_data.num_goal; i++)
		{
			if(i%lane.rows() == 0)
				m++;

			y_g(i) = lane(i%lane.rows());
			x_g(i) = 120;// + m*10;

            meta_cost(i, 0) = i+1;
		}
        // x_g << 110, 115, 120, 125, 130, 135, 120, 120, 120, 120, 120;
        x_g << 110, 115, 120, 125, 130, 135, 120, 120, 120, 120, 120;
        x_g += 40;
        y_g << -10, -10, -10, -10, -10, -10, -6, -2, 2 ,6, 10; 
		old = x_g;

        
        subscription_ = this->create_subscription<msgs_car::msg::States>(
        "ego_vehicle_obs", 10, std::bind(&MinimalPublisher::topic_callback, this, _1));

        publisher_ = this->create_publisher<msgs_car::msg::Controls>("ego_vehicle_cmds", 10);
        timer_ = this->create_wall_timer(10ms, bind(&MinimalPublisher::timer_callback, this));
        cnt = 1;

        RCLCPP_INFO(this->get_logger(),"NODES ARE UP");
        // outdata.open("stats/New/Cruise/No-Time-Budget/mpc_car_batch_data_11_goals.txt");
        // outdata.open("stats/New/RightLane/No-Time-Budget/mpc_car_batch_data_11_goals.txt");
        // outdata.open("stats/New/HighSpeed_RightLane/No-Time-Budget/mpc_car_batch_data_11_goals.txt");
        outdata.open("/home/vivek/On-Codes/Backup/Batch_traj_opt/ros_ws/stats/New/NGSIM/No-Time-Budget/mpc_car_batch_data_11_goals_0_ngsim.txt");
        outdata2.open("/home/vivek/On-Codes/Backup/Batch_traj_opt/ros_ws/stats/New/NGSIM/No-Time-Budget/mpc_car_batch_data_11_goals_0_all_info.txt");
    }
    
  private:
    void topic_callback(const msgs_car::msg::States::SharedPtr msg)
    {
       
        x_init = msg->x[0];
        y_init = msg->y[0];
        v_init = sqrt(msg->vx[0] * msg->vx[0] + msg->vy[0] * msg->vy[0]);
        psi_init = msg->psi[0];
        psidot_init = msg->psidot; 
        
        x_obs_temp << msg->x[1], msg->x[2], msg->x[3], msg->x[4], msg->x[5], msg->x[6];
        y_obs_temp << msg->y[1], msg->y[2], msg->y[3], msg->y[4], msg->y[5], msg->y[6];
        vx_obs << msg->vx[1], msg->vx[2], msg->vx[3], msg->vx[4], msg->vx[5], msg->vx[6];
        vy_obs << msg->vy[1], msg->vy[2], msg->vy[3], msg->vy[4], msg->vy[5], msg->vy[6];

        if(loop == 0)
            prev_v_send = v_init;
        
        Gotit = 1;
    }
    void timer_callback();
    void get_ranks();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<msgs_car::msg::States>::SharedPtr subscription_;
    rclcpp::Publisher<msgs_car::msg::Controls>::SharedPtr publisher_;
    size_t count_;
};

void MinimalPublisher :: get_ranks()
{
    float v_cruise = 15.0;
    for(int i = 0; i < prob_data.num_goal; i++)
    {
        meta_cost(i, 1) = (prob_data.v.row(i) - v_cruise).matrix().lpNorm<2>(); // 5
        meta_cost(i, 2) = prob_data.res_obs.row(i).matrix().lpNorm<2>();        // 6
        meta_cost(i, 3) = (prob_data.y.row(i) -  (-10)).matrix().lpNorm<2>(); // 7
        meta_cost(i, 4) = (prob_data.v.row(i) - 24).matrix().lpNorm<2>();                       // 8
        meta_cost(i, 5) = -1;
        meta_cost(i, 6) = -1;
        meta_cost(i, 7) = -1;
        meta_cost(i, 8) = -1;
    }
    float inf = std::numeric_limits<float>::infinity();
    for(int i = 0; i < prob_data.num_goal; i++)
    {
        float min0 = inf, min1 = inf, min2 = inf, min3 = inf; 
        int index0 = -1, index1 = -1, index2 = -1, index3 = -1;
        for(int j = 0; j < prob_data.num_goal; j++)
        {
            if(meta_cost(j, 1) < min0 && meta_cost(j, 5) < 0)
            {
                min0 = meta_cost(j, 1);
                index0 = j;
            }
            if(meta_cost(j, 2) < min1 && meta_cost(j, 6) < 0)
            {
                min1 = meta_cost(j, 2);
                index1 = j;
            }
            if(meta_cost(j, 3) < min2 && meta_cost(j, 7) < 0)
            {
                min2 = meta_cost(j, 3);
                index2 = j;
            }
            if(meta_cost(j, 4) < min3 && meta_cost(j, 8) < 0)
            {
                min3 = meta_cost(j, 4);
                index3 = j;
            }
        }
        meta_cost(index0, 5) = i+1; // cruise     
        meta_cost(index1, 6) = i+1; // optimal
        meta_cost(index2, 7) = i+1; // rightmost lane
        meta_cost(index3, 8) = i+1; // max average velocity
    }
}
void MinimalPublisher :: timer_callback()
{
    auto message = msgs_car::msg::Controls();
    
    if(Gotit)
    {
        
        x_g = old + x_init;
        cnt+=1;
       
        start = clock();
        prob_data = mpc(prob_data, PPP, x_g, y_g, x_init, y_init, v_init, psi_init, psidot_init, x_obs_temp, y_obs_temp, 
                            vx_obs, vy_obs, warm);
        end = clock();
        warm = false;
        
        get_ranks();
        min = 100000000;
        index = 5;
        // cruise = 50, 50, 0, 0
        // righlane = 0, 70, 30, 0
        // highspeed rightlane = 0, 50, 25, 25

        //ngsim2 hsrl 0, 60, 19, 21
        for(int i = 0; i < prob_data.num_goal; i++)
        {
            float cost = 0.0 * meta_cost(i, 5) + 60.0 * meta_cost(i, 6) + 19.0* meta_cost(i, 7) + 21.0 * meta_cost(i, 8); 
                                // cruise                  optimal                   rightlane             max avg velocity
            if( cost < min)
            {
                min = cost; 
                index = i;
            }    
        }
        // message.w = prob_data.w_controls.row(index).leftCols(18).mean();
        // message.v = prob_data.v_controls.row(index).leftCols(18).mean();
        message.w = prob_data.psidot.row(index).leftCols(3).mean();
        message.v = prob_data.v.row(index).leftCols(3).mean();

        message.index = index;
        message.goals = prob_data.num_goal;
        loop++;
        
        speed += message.v;
        total_time += double(end - start) / double(CLOCKS_PER_SEC);
        avg_speed = speed/loop;
        avg_time = total_time/loop;

        outdata << x_init << " " << y_init << " " << psi_init << " " << message.v << " " << message.w 
                << " " << (message.v - prev_v_send)/prob_data.t << " " << (message.w - prev_w_send)/prob_data.t
                << " " << double(end - start) / double(CLOCKS_PER_SEC) << " " << loop << " " << index << endl;
        outdata2 << prob_data.x << " " << prob_data.y << endl;
        prev_v_send = message.v;
		prev_w_send = message.w;

        for(int i = 0; i < prob_data.num_goal; i++)
        {
            for(int j = 0; j < prob_data.num; j++)
            {
                geometry_msgs::msg::Pose pose;
                pose.position.x = prob_data.x(i, j);
                pose.position.y = prob_data.y(i, j);
                message.batch.poses.push_back(pose);    
            }
        }
        // RCLCPP_INFO(this->get_logger(),"Time taken = %f index = %d res_obs = %f", time_taken, index, prob_data.res_obs.matrix().lpNorm<2>());
        // RCLCPP_INFO(this->get_logger(),"Average time = %f Average speed = %f", avg_time, avg_speed);
        publisher_->publish(message);
        Gotit = false;
    }
    
}

int main(int argc, char * argv[])
{
    
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}