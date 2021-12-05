#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include"frenet_cpp/cubic_spline.h"
#include"frenet_cpp/frenet_path.h"
#include"frenet_cpp/quintic_polynomial.h"
#include"frenet_cpp/quartic_polynomial.h"
#include "frenet_optimal_trajectory.cpp"

using namespace cpprobotics;
using namespace std;

#define SIM_LOOP 500
#define MAX_SPEED  50.0 / 3.6  // maximum speed [m/s]
#define MAX_ACCEL  2.0  // maximum acceleration [m/ss]
#define MAX_CURVATURE  1.0  // maximum curvature [1/m]
#define MAX_ROAD_WIDTH  7.0  // maximum road width [m]
#define D_ROAD_W  0.5  // road width sampling length [m]
#define DT  0.1  // time tick [s]
#define MAXT  5.0  // max prediction time [m]
#define MINT  4.0  // min prediction time [m]
#define TARGET_SPEED  30.0 / 3.6  // target speed [m/s]
#define D_T_S  5.0 / 3.6  // target speed sampling length [m/s]
#define N_S_SAMPLE  4  // sampling number of target speed
#define ROBOT_RADIUS  1.5  // robot radius [m]
#define OBSTACLE_RADIUS  1.5
#define _USE_MATH_DEFINES

#define KJ  0.1
#define KT  0.1
#define KD  1.0
#define KLAT  1.0
#define KLON  1.0

using std::placeholders::_1;

class FrenetPub : public rclcpp::Node
{
   public:
    size_t count_;
    Vec_f wx;
    Vec_f wy;
    Vec_f agent_p;
    Vec_f agent_v;
    double agent_head;
    double csp_discretization;
    std::vector<Poi_f> obstcles;
    std::vector<Poi_f> obstcles_vel;

    Spline2D *csp_obj;
    Vec_f r_x;
    Vec_f r_y;
    Vec_f ryaw;
    Vec_f rcurvature;
    Vec_f rs;
    double vx, vy, v_s, v_c, temp, s_cal;
    
    clock_t start, end;
    float c_speed;
    float c_d;
    float c_d_d;
    float c_d_dd;
    float s0;
    double a;
    double b;
    double v_x;
    double v_y;
    double w_x;
    double w_y;
    int count;
    double total_time;

    rclcpp::TimerBase::SharedPtr timer_;

    FrenetPub();
    void timer_callback();
};

FrenetPub::FrenetPub() : Node("frenet_publisher"), count_(0)
{
    wx = Vec_f({0.0, 50.0, 100.0});
    wy = Vec_f({0.0, 0.0, 0.0});
    agent_p = Vec_f({5, 2.0});
    agent_v = Vec_f({1.0, 0.0});
    agent_head=0.00;;
    csp_discretization=0.01;
    obstcles = {
                {{20.0, -2.0}},
                {{40.0, 0.0}},
                {{65.0, 4.0}},
                {{70.0, 4.0}},
                {{80.0, -4.0}}
                };
    obstcles_vel = {
                {{-0.5, 0.0}},
                {{-0.5, 0.0}},
                {{-0.5, 0.0}},
                {{-0.5, 0.0}},
                {{-0.5, 0.0}}
                };
    csp_obj = new Spline2D(wx, wy);
    vx=std::cos(agent_head)*agent_v[0];
    vy=std::sin(agent_head)*agent_v[0];
    float min=std::numeric_limits<float>::max();

    for(double i=0; i<csp_obj->s.back(); i+=csp_discretization)
    {
        std::array<float, 2> point_ = csp_obj->calc_postion(i);
        r_x.push_back(point_[0]);
        r_y.push_back(point_[1]);
        temp=std::sqrt(std::pow(point_[0]-agent_p[0],2)+std::pow(point_[1]-agent_p[1],2));
        if(temp<min)
        { 
            min=temp;
            s_cal=i;
        }
        ryaw.push_back(csp_obj->calc_yaw(i));
        rcurvature.push_back(csp_obj->calc_curvature(i));
        rs.push_back(i);
    }

    v_s=(vx*(r_x[1]-r_x[0]) + vy*(r_y[1]-r_y[0]))/std::sqrt(std::pow(r_x[1]-r_x[0],2)+std::pow(r_y[1]-r_y[0],2)); 
    v_c=(vx*(r_y[0]-r_y[1]) + vy*(r_x[1]-r_x[0]))/std::sqrt(std::pow(r_x[1]-r_x[0],2)+std::pow(r_y[1]-r_y[0],2));

    c_speed = v_s;
    c_d = min;
    c_d_d = v_c;
    c_d_dd = 0.0;
    s0 = s_cal;
    a=2.0,b=1.5;
    v_x=0.0;
    v_y=0.0;
    w_x=0.0;
    w_y=0.0;

    cv::namedWindow("frenet");

    count = 0;

    timer_ = this->create_wall_timer(10ms, std::bind(&FrenetPub::timer_callback, this));
}

void FrenetPub::timer_callback()
{
    start = clock();
    auto &csp_obj2 = *csp_obj;
    FrenetPath final_path = frenet_optimal_planning(csp_obj2, s0, c_speed, c_d, c_d_d, c_d_dd, obstcles,obstcles_vel,a,b);
    end = clock();
    total_time = double(end - start) / double(CLOCKS_PER_SEC);
    cout<< total_time<<endl;
    s0 = final_path.s[1];
    c_d = final_path.d[1];
    c_d_d = final_path.d_d[1];
    c_d_dd = final_path.d_dd[1];
    c_speed = final_path.s_d[1];
    agent_v[0]=std::sqrt(std::pow((final_path.x[1] - agent_p[0])/DT, 2)+std::pow((final_path.y[1] - agent_p[1])/DT, 2));
    agent_v[1]=(std::atan2((final_path.y[1] - agent_p[1]),(final_path.x[1] - agent_p[0]))-agent_head)/DT;
    agent_head=agent_head+agent_v[1]*DT;
    vx=std::cos(agent_head)*agent_v[0];
    vy=std::sin(agent_head)*agent_v[0];
    agent_p[0]=agent_p[0]+vx*DT;
    agent_p[1]=agent_p[1]+vy*DT;
    //cout<<agent_v[0]<<" "<<agent_v[1]<<" "<<agent_head<<endl;
    if (std::pow((final_path.x[1] - r_x.back()), 2) + std::pow((final_path.y[1]-r_y.back()), 2) <= 1.0)
    {
        return;
    }

    // visualization
    cv::Mat bg(1080, 1920, CV_8UC3, cv::Scalar(255, 255, 255));
    for(unsigned int i=1; i<r_x.size(); i++)
    {
      cv::line(
        bg,
        cv_offset(0, 0, bg.cols, bg.rows),
        cv_offset(100, 0, bg.cols, bg.rows),
        cv::Scalar(0, 0, 0),
        3);
    }
    for(unsigned int i=1; i<r_x.size(); i++)
    {
      cv::line(
        bg,
        cv_offset(0, MAX_ROAD_WIDTH, bg.cols, bg.rows),
        cv_offset(100, MAX_ROAD_WIDTH, bg.cols, bg.rows),
        cv::Scalar(0, 0, 0),
        3);
    }
    for(unsigned int i=1; i<r_x.size(); i++)
    {
      cv::line(
        bg,
        cv_offset(0, -MAX_ROAD_WIDTH, bg.cols, bg.rows),
        cv_offset(100, -MAX_ROAD_WIDTH, bg.cols, bg.rows),
        cv::Scalar(0, 0, 0),
        3);
    }



    cv::ellipse(bg,
        cv_offset(final_path.x[1], final_path.y[1], bg.cols, bg.rows), cv::Size(a*bg.cols/100, b*bg.rows/100),-agent_head*180/M_PI,0,360,
        cv::Scalar(0, 255, 0),-1);
        for(unsigned int i=0; i<final_path.x.size(); i++){
    cv::circle(
        bg,
        cv_offset(final_path.x[i], final_path.y[i], bg.cols, bg.rows),
        2, cv::Scalar(255, 0, 0), -1);
    }       
    for(unsigned int i=0; i<obstcles.size(); i++)
    {
        obstcles[i][0]=obstcles[i][0]+obstcles_vel[i][0]*DT;
        obstcles[i][1]=obstcles[i][1]+obstcles_vel[i][1]*DT;
        /*cv::circle(
        bg,
        cv_offset(obstcles[i][0], obstcles[i][1], bg.cols, bg.rows),
        int(OBSTACLE_RADIUS*bg.rows/100), cv::Scalar(0, 0, 255), -1);*/
        cv::ellipse(bg,
        cv_offset(obstcles[i][0], obstcles[i][1], bg.cols, bg.rows), cv::Size(a*bg.cols/100, b*bg.rows/100),0,0,360,
        cv::Scalar(0, 0, 255),-1);   

    }
    std::ostringstream filename;
    //filename << "file_" << i << ".jpeg";
    //std::cout<<filename.str();
    cv::imshow("frenet", bg);
    //cv::resizeWindow("frenet", 1920,1080);
    //cv::imwrite(filename.str(), bg);
    cv::waitKey(5);
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrenetPub>());
  rclcpp::shutdown();
  return 0;
}