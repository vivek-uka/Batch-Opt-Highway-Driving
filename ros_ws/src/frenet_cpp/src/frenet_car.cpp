#include <chrono>
#include <functional>
#include <memory>
#include <iostream>
#include <time.h>
#include <fstream>
#include <cstdlib>
#include <iomanip> 
#include <cmath> 

#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/rclcpp.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "geometry_msgs/msg/vector3.hpp"
#include "msgs_car/msg/controls.hpp"
#include "msgs_car/msg/states.hpp"
#include "yaml-cpp/yaml.h"


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

using std::placeholders::_1;

class FrenetPub : public rclcpp::Node
{
   public:
    size_t count_;
    Vec_f wx;
    Vec_f wy;
    Vec_f agent_p;
    Vec_f agent_v;
    Vec_f goal_p;
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
    bool Gotit, first;
    float  dt;
    float prev_v_send, prev_w_send;
    ofstream outdata, outdata2, outdata3, outdata4;
    float w0, w1, w2;



    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<msgs_car::msg::States>::SharedPtr subscription_;
    rclcpp::Publisher<msgs_car::msg::Controls>::SharedPtr publisher_;

    FrenetPub();
    void timer_callback();
    void topic_callback(const msgs_car::msg::States::SharedPtr msg);
};

FrenetPub::FrenetPub() : Node("frenet_publisher"), count_(0)
{
    YAML::Node map = YAML::LoadFile("src/frenet_cpp/config.yaml");
        
    string setting = map["setting"].as<string>();

    w0 = map["configuration"][setting]["weights"][0].as<float>();
    w1 = map["configuration"][setting]["weights"][1].as<float>();
    w2 = map["configuration"][setting]["weights"][2].as<float>();

    prev_v_send = 0.0;
    prev_w_send = 0.0;
    dt = 0.08;
    Gotit = false;
    first = true;
    wx = Vec_f({0.0, 100.0});
    wy = Vec_f({0.0, 0.0});
    agent_p = Vec_f({5, 2.0});
    agent_v = Vec_f({1.0, 0.0});
    goal_p = Vec_f({1500, agent_p[1]});
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
    a=5.6,b=3.0;
    v_x=0.0;
    v_y=0.0;
    w_x=0.0;
    w_y=0.0;

    cv::namedWindow("frenet");

    count = 0;

    subscription_ = this->create_subscription<msgs_car::msg::States>(
        "ego_vehicle_obs", 10, std::bind(&FrenetPub::topic_callback, this, _1));

    publisher_ = this->create_publisher<msgs_car::msg::Controls>("ego_vehicle_cmds", 10);
    timer_ = this->create_wall_timer(10ms, std::bind(&FrenetPub::timer_callback, this));

    //outdata.open("/home/adi99/Desktop/frenet_cruise_stats.txt");
    outdata.open(map["configuration"][setting]["file"].as<string>());
}

void FrenetPub::topic_callback(const msgs_car::msg::States::SharedPtr msg)
{
    cout<<"Topic callback  "<<endl;
    agent_p = Vec_f({msg->x[0], msg->y[0]});
    agent_v = Vec_f({sqrt(msg->vx[0] * msg->vx[0] + msg->vy[0] * msg->vy[0]), msg->psidot});
    agent_head = msg->psi[0];

    obstcles = {{{msg->x[1], msg->y[1]}},
                {{msg->x[2], msg->y[2]}},
                {{msg->x[3], msg->y[3]}},
                {{msg->x[4], msg->y[4]}},
                {{msg->x[5], msg->y[5]}},
                {{msg->x[6], msg->y[6]}}};

    obstcles_vel = {{{msg->vx[1], msg->vy[1]}},
                    {{msg->vx[2], msg->vy[2]}},
                    {{msg->vx[3], msg->vy[3]}},
                    {{msg->vx[4], msg->vy[4]}},
                    {{msg->vx[5], msg->vy[5]}},
                    {{msg->vx[6], msg->vy[6]}}};
    if(first)
    {
        wx = Vec_f({agent_p[0], goal_p[0]});
        wy = Vec_f({0.0, 0.0});
        c_speed = agent_v[0];
        c_d = agent_p[1];
        first = false;
        c_d_d = 0.0;  // current lateral speed [m/s]
		c_d_dd = 0.0;  // current latral acceleration [m/s]
		s0 = 0.0;
    }
    //exit(1);
    Gotit = true;
}

void FrenetPub::timer_callback()
{
    if(Gotit)
    {
        msgs_car::msg::Controls msg;;
        csp_obj = new Spline2D(wx, wy);
        auto &csp_obj2 = *csp_obj;
        int index;
        //cout<< agent_p[0]<<", "<<agent_p[1]<<endl;
        start = clock();
        Vec_Path save_path = frenet_optimal_planning(csp_obj2, s0, c_speed, c_d, c_d_d, c_d_dd, obstcles,obstcles_vel,a,b, index, w0, w1, w2);
        end = clock();
        total_time = double(end - start) / double(CLOCKS_PER_SEC);
        cout<<"Time =  "<<total_time<<endl;
        cout<<"Num Paths = "<<save_path.size()<<endl;

        for(int i = 0; i < save_path.size(); i++)
        {
            for(int j = 0; j < save_path[i].x.size(); j++)
            {
                geometry_msgs::msg::Pose pose;
                pose.position.x = save_path[i].x[j];
                pose.position.y = save_path[i].y[j];
                msg.batch.poses.push_back(pose); 
            }
        }
        
        float v = std::sqrt(std::pow((save_path[index].x[1] - agent_p[0])/dt, 2)+std::pow((save_path[index].y[1] - agent_p[1])/dt, 2));
        float w = (std::atan2((save_path[index].y[1] - agent_p[1]),(save_path[index].x[1] - agent_p[0]))-agent_head)/dt;
        cout<< v<<", "<<w<<endl;
        s0 = save_path[index].s[1];
        c_d = save_path[index].d[1];
        c_d_d = save_path[index].d_d[1];
        c_d_dd = save_path[index].d_dd[1];
        c_speed = save_path[index].s_d[1];

        msg.v = v;
        msg.w = w;
        msg.index = index;
        msg.goals = save_path.size();

        outdata << agent_p[0] << " " << agent_p[1] << " " << agent_head << " " << msg.v << " " << msg.w 
                 << " " << (msg.v - prev_v_send)/dt << " " << (msg.w - prev_w_send)/dt
                 << " " << double(end - start) / double(CLOCKS_PER_SEC) << " " << index << endl;

        prev_v_send = msg.v;
		prev_w_send = msg.w;

        publisher_->publish(msg);
        Gotit = false;

    }
    
    //cout<<agent_v[0]<<" "<<agent_v[1]<<" "<<agent_head<<endl;
    
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrenetPub>());
  rclcpp::shutdown();
  return 0;
}