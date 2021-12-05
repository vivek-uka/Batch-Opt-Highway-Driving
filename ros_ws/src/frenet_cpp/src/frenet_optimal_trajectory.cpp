/*************************************************************************
	> File Name: frenet_optimal_trajectory.cpp
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Wed Apr  3 09:52:17 2019
 ************************************************************************/

#include<iostream>
#include<cmath>
#include<limits>
#include<vector>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<sys/time.h>
#include"frenet_cpp/cubic_spline.h"
#include"frenet_cpp/frenet_path.h"
#include"frenet_cpp/quintic_polynomial.h"
#include"frenet_cpp/quartic_polynomial.h"
#include <time.h>

#define SIM_LOOP 500
#define MAX_SPEED  24.0  // maximum speed [m/s]
#define MAX_ACCEL  4.0  // maximum acceleration [m/ss]
#define MAX_CURVATURE  10.0  // maximum curvature [1/m]
#define MAX_ROAD_WIDTH  10.5  // maximum road width [m]
#define D_ROAD_W  1.0  // road width sampling length [m]
#define DT  0.08  // time tick [s]
#define MAXT  5.0  // max prediction time [m]
#define MINT  2.8  // min prediction time [m]
#define TARGET_SPEED  15.0  // target speed [m/s]
#define D_T_S  10.0 / 3.6  // target speed sampling length [m/s]
#define N_S_SAMPLE  4  // sampling number of target speed
#define ROBOT_RADIUS  1.5  // robot radius [m]
#define OBSTACLE_RADIUS  1.5
#define _USE_MATH_DEFINES

#define KJ  0.1
#define KT  0.1
#define KD  1.0
#define KLAT  1.0
#define KLON  1.0


using namespace std;

namespace cpprobotics
{
float sum_of_power(std::vector<float> value_list){
  float sum = 0;
  for(float item:value_list){
    sum += item*item;
  }
  return sum;
}

Vec_Path calc_frenet_paths(
    float c_speed, float c_d, float c_d_d, float c_d_dd, float s0){
  std::vector<FrenetPath> fp_list;
  float t_arr[] = {2.8, 3.6, 4.5};
  for(float di=-1*MAX_ROAD_WIDTH; di<MAX_ROAD_WIDTH; di+=D_ROAD_W){
    for(int i=0; i<3; i++){
      float Ti = t_arr[i];
      FrenetPath fp;
      QuinticPolynomial lat_qp(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti);
      for(int t=0; t<Ti/DT; t++){
        fp.t.push_back(t*DT);
        fp.d.push_back(lat_qp.calc_point(t*DT));
        fp.d_d.push_back(lat_qp.calc_first_derivative(t*DT));
        fp.d_dd.push_back(lat_qp.calc_second_derivative(t*DT));
        fp.d_ddd.push_back(lat_qp.calc_third_derivative(t*DT));
      }
      for(float tv=TARGET_SPEED - D_T_S * N_S_SAMPLE;
          tv < TARGET_SPEED + D_T_S * N_S_SAMPLE;
          tv+=D_T_S){

        FrenetPath fp_bot = fp;
        QuarticPolynomial lon_qp(s0, c_speed, 0.0, tv, 0.0, Ti);

        fp_bot.max_speed = std::numeric_limits<float>::min();
        fp_bot.max_accel = std::numeric_limits<float>::min();
        for(float t_:fp.t){
          fp_bot.s.push_back(lon_qp.calc_point(t_));
          fp_bot.s_d.push_back(lon_qp.calc_first_derivative(t_));
          fp_bot.s_dd.push_back(lon_qp.calc_second_derivative(t_));
          fp_bot.s_ddd.push_back(lon_qp.calc_third_derivative(t_));
          if(fp_bot.s_d.back() > fp_bot.max_speed){
            fp_bot.max_speed = fp_bot.s_d.back();
          }
          if(abs(fp_bot.s_dd.back()) > fp_bot.max_accel){
            fp_bot.max_accel = abs(fp_bot.s_dd.back());
          }
        }

        float Jp = sum_of_power(fp.d_ddd);
        float Js = sum_of_power(fp_bot.s_ddd);
        float ds = (TARGET_SPEED - fp_bot.s_d.back());

        fp_bot.cd = KJ * Jp + KT * Ti + KD * std::pow(fp_bot.d.back(), 2);
        fp_bot.cv = KJ * Js + KT * Ti + KD * ds;
        fp_bot.cf = KLAT * fp_bot.cd + KLON * fp_bot.cv;

        fp_list.push_back(fp_bot);
      }
    }
  }
  return fp_list;
}

void calc_global_paths(Vec_Path & path_list, Spline2D csp){
  for (Vec_Path::iterator path_p=path_list.begin(); path_p!=path_list.end();path_p++){
    for(unsigned int i=0; i<path_p->s.size(); i++){
      if (path_p->s[i] >= csp.s.back()){
        break;
      }
      std::array<float, 2> poi = csp.calc_postion(path_p->s[i]);
      float iyaw = csp.calc_yaw(path_p->s[i]);
      float di = path_p->d[i];
      float x = poi[0] + di * std::cos(iyaw + M_PI/2.0);
      float y = poi[1] + di * std::sin(iyaw + M_PI/2.0);
      path_p->x.push_back(x);
      path_p->y.push_back(y);
    }

    for(int i=0; i<path_p->x.size()-1; i++){
      float dx = path_p->x[i + 1] - path_p->x[i];
      float dy = path_p->y[i + 1] - path_p->y[i];
      path_p->yaw.push_back(std::atan2(dy, dx));
      path_p->ds.push_back(std::sqrt(dx * dx + dy * dy));
    }

    path_p->yaw.push_back(path_p->yaw.back());
    path_p->ds.push_back(path_p->ds.back());


    path_p->max_curvature = std::numeric_limits<float>::min();
    for(int i=0; i<path_p->x.size()-1; i++){
      path_p->c.push_back((path_p->yaw[i+1]-path_p->yaw[i])/path_p->ds[i]);
      if(abs(path_p->c.back()) > path_p->max_curvature){
        path_p->max_curvature = abs(path_p->c.back());
      }
    }
  }
}

bool check_collision(FrenetPath path, const Vec_Poi ob, const Vec_Poi ob_v ){
  for(unsigned int j=0; j<ob.size(); j++){
    float counter=0;
    for(unsigned int i=0; i<path.x.size(); i++){
      counter=counter+DT;
      float dist = std::pow((path.x[i]-(ob[j][0]+ob_v[j][0]*DT)), 2) + std::pow((path.y[i]-(ob[j][1]+ob_v[j][1]*DT)), 2);
      if (dist <= (OBSTACLE_RADIUS+ROBOT_RADIUS) * (OBSTACLE_RADIUS+ROBOT_RADIUS)){
        return false;
      }
    }
  }
  return true;
}

bool check_collision_ellipse(FrenetPath path, const Vec_Poi ob, const Vec_Poi ob_v,double a,double b, bool id){
  a = 5.6;
  b = 3.0;
  for(unsigned int j=0; j<ob.size(); j++){
    float counter=0;
    for(unsigned int i=0; i<path.x.size(); i++){
      counter=counter+DT;
      float dist = pow(path.x[i] - (ob[j][0] + ob_v[j][0]*counter), 2)/(a*a) + pow(path.y[i] - (ob[j][1] + ob_v[j][1]*counter), 2)/(b*b);
      if (dist < 1){
        return false;
      }
    }
  }
  return true;
}

Vec_Path check_paths(Vec_Path path_list, const Vec_Poi ob, const Vec_Poi ob_v,double a,double b){
	Vec_Path output_fp_list;
  int speed = 0;
  int  acc = 0;
  int  curv = 0;
  int  coll = 0;
  for (int i = 0; i < path_list.size(); i++)
  {
    bool val = false;
    for(int j = 0; j<path_list[i].s_d.size(); j++)
    {
      if(path_list[i].s_d[j] > MAX_SPEED)
      {
        val = true;
        speed+=1;
        break;
      }
    }
    if(val)
      continue;
    for(int j = 0; j<path_list[i].s_dd.size(); j++)
    {
      if(abs(path_list[i].s_dd[j]) > MAX_ACCEL)
      {
        val = true;
        acc+=1;
        break;
      }
    }
    if(val)
      continue;
    for(int j = 0; j<path_list[i].c.size(); j++)
    {
      if(abs(path_list[i].c[j]) > MAX_CURVATURE)
      {
        val = true;
        curv+=1;
        break;
      }
    }
    if(val)
      continue;
    for(int j = 0; j<path_list[i].x.size()-1; j++)
    {
      float yaw = atan2(path_list[i].y[j+1] - path_list[i].y[j], path_list[i].x[j+1] - path_list[i].x[j]);
      if(abs(yaw) > 13*3.14159265359/180)
      {
        val = true;
        break;
      }
    }
    if(val)
      continue;
    bool id = false;
    if(!check_collision_ellipse(path_list[i], ob, ob_v, a, b, id))
    {
      coll+=1;
      continue;
    }
    output_fp_list.push_back(path_list[i]);
  }
  return output_fp_list;
}

Vec_Path frenet_optimal_planning(
    Spline2D csp, float s0, float c_speed,
    float c_d, float c_d_d, float c_d_dd, Vec_Poi ob, Vec_Poi ob_v,double a,double b, int &index, float we0, float we1, float we2){
  clock_t start1, end1;
  double total_time1;
  start1 = clock();
  Vec_Path fp_list = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0);
  end1 = clock();
  total_time1 = double(end1 - start1) / double(CLOCKS_PER_SEC);
  cout<<"Trajectory Generation = "<<fp_list.size()<<endl;
  
  start1 = clock();
  calc_global_paths(fp_list, csp);
  end1 = clock();
  total_time1 = double(end1 - start1) / double(CLOCKS_PER_SEC);
  cout<<"Trajectory Global = "<<fp_list.size()<<endl;

  start1 = clock();
  Vec_Path save_paths = check_paths(fp_list, ob, ob_v,a,b);
  end1 = clock();
  total_time1 = double(end1 - start1) / double(CLOCKS_PER_SEC);
  cout<<"Trajectory Validation and Collision Avoidance = "<<save_paths.size()<<endl;

  float min_cost = std::numeric_limits<float>::max();
  FrenetPath final_path;
  int min_cost_ind = 0;
  float cost = std::numeric_limits<float>::max();
  for(auto path:save_paths){
    //std::vector<float> vx;
    //std::vector<float> vy;
    //std::adjacent_difference(path.x.begin(), path.x.end(), vx);
    //std::adjacent_difference(path.y, path.y.end(), vy);
    float vel = 0;
    float w1 = 0;
    float w2 = 0;
    float w3 = 0;
    for(int i=0; i<path.x.size()-1; i++)
    {
      vel = sqrt(pow(path.x[i+1] - path.x[i],2) + pow(path.y[i+1] - path.y[i],2))/DT;
      
      w1+=pow(vel-TARGET_SPEED,2);
      w2+= pow(path.y[i]-(-10),2);
      w3+=pow(vel-24,2);
    }
    w1 = sqrt(w1);
    w2 = sqrt(w2);
    w3 = sqrt(w3);
    cost = we0*w1 + we1*w2 + we2*w3;
    //cout<<"cost = "<<w1<<endl;
    //exit(1);
    if (min_cost >= cost){//path.cf){
      min_cost = cost;//path.cf;
      final_path = path;
      index = min_cost_ind;
    }
    min_cost_ind+=1;
  }
  return save_paths;
}

cv::Point2i cv_offset(
    float x, float y, int image_width=1920, int image_height=1080){
  cv::Point2i output;
  output.x = int(x * image_width/100);
  output.y = int(image_height/2) - int(y*image_height/100) ;
  return output;
}
}
/*int main(){
  Vec_f wx({0.0, 50.0, 100.0});
  Vec_f wy({0.0, 0.0, 0.0});
  Vec_f agent_p({5, 2.0});
  Vec_f agent_v({1.0, 0.0});
  double agent_head=0.00;
  double csp_discretization=0.01;
  std::vector<Poi_f> obstcles{
    {{20.0, -2.0}},
    {{40.0, 0.0}},
    {{65.0, 4.0}},
    {{70.0, 4.0}},
    {{80.0, -4.0}}
  };
  std::vector<Poi_f> obstcles_vel{
    {{-0.5, 0.0}},
    {{-0.5, 0.0}},
    {{-0.5, 0.0}},
    {{-0.5, 0.0}},
    {{-0.5, 0.0}}
  };

  Spline2D csp_obj(wx, wy);
  Vec_f r_x;
  Vec_f r_y;
  Vec_f ryaw;
  Vec_f rcurvature;
  Vec_f rs;
  double vx, vy, v_s, v_c, temp, s_cal;
  
  vx=std::cos(agent_head)*agent_v[0];
  vy=std::sin(agent_head)*agent_v[0];
  float min=std::numeric_limits<float>::max();
  for(double i=0; i<csp_obj.s.back(); i+=csp_discretization){
    std::array<float, 2> point_ = csp_obj.calc_postion(i);
    r_x.push_back(point_[0]);
    r_y.push_back(point_[1]);
    temp=std::sqrt(std::pow(point_[0]-agent_p[0],2)+std::pow(point_[1]-agent_p[1],2));
    if(temp<min)
    { 
      min=temp;
      s_cal=i;
    }
    ryaw.push_back(csp_obj.calc_yaw(i));
    rcurvature.push_back(csp_obj.calc_curvature(i));
    rs.push_back(i);
  }

  v_s=(vx*(r_x[1]-r_x[0]) + vy*(r_y[1]-r_y[0]))/std::sqrt(std::pow(r_x[1]-r_x[0],2)+std::pow(r_y[1]-r_y[0],2)); 
  v_c=(vx*(r_y[0]-r_y[1]) + vy*(r_x[1]-r_x[0]))/std::sqrt(std::pow(r_x[1]-r_x[0],2)+std::pow(r_y[1]-r_y[0],2));

  clock_t start, end;
  float c_speed = v_s;
  float c_d = min;
  float c_d_d = v_c;
  float c_d_dd = 0.0;
  float s0 = s_cal;
  double a=2.0,b=1.5;
  double v_x=0.0;
  double v_y=0.0;
  double w_x=0.0;
  double w_y=0.0;
  cout<<c_speed<<" "<<min<<" "<<v_c<<" "<<s_cal<<" "<<agent_p[0]<<" "<<agent_p[1]<<" "<<agent_head<<endl;
  cv::namedWindow("frenet");
  int count = 0;
  double total_time;
  for(int i=0; i<SIM_LOOP; i++){
    start = clock();
    FrenetPath final_path = frenet_optimal_planning(
    csp_obj, s0, c_speed, c_d, c_d_d, c_d_dd, obstcles,obstcles_vel,a,b);
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
    if (std::pow((final_path.x[1] - r_x.back()), 2) + std::pow((final_path.y[1]-r_y.back()), 2) <= 1.0){
        break;
    }

    // visualization
    cv::Mat bg(1080, 1920, CV_8UC3, cv::Scalar(255, 255, 255));
    for(unsigned int i=1; i<r_x.size(); i++){
      cv::line(
        bg,
        cv_offset(0, 0, bg.cols, bg.rows),
        cv_offset(100, 0, bg.cols, bg.rows),
        cv::Scalar(0, 0, 0),
        3);
    }
    for(unsigned int i=1; i<r_x.size(); i++){
      cv::line(
        bg,
        cv_offset(0, MAX_ROAD_WIDTH, bg.cols, bg.rows),
        cv_offset(100, MAX_ROAD_WIDTH, bg.cols, bg.rows),
        cv::Scalar(0, 0, 0),
        3);
    }
    for(unsigned int i=1; i<r_x.size(); i++){
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
    for(unsigned int i=0; i<obstcles.size(); i++){
        obstcles[i][0]=obstcles[i][0]+obstcles_vel[i][0]*DT;
        obstcles[i][1]=obstcles[i][1]+obstcles_vel[i][1]*DT;
        /*cv::circle(
        bg,
        cv_offset(obstcles[i][0], obstcles[i][1], bg.cols, bg.rows),
        int(OBSTACLE_RADIUS*bg.rows/100), cv::Scalar(0, 0, 255), -1);
        cv::ellipse(bg,
        cv_offset(obstcles[i][0], obstcles[i][1], bg.cols, bg.rows), cv::Size(a*bg.cols/100, b*bg.rows/100),0,0,360,
        cv::Scalar(0, 0, 255),-1);   

      }
    std::ostringstream filename;
    filename << "file_" << i << ".jpeg";
    //std::cout<<filename.str();
    cv::imshow("frenet", bg);
    //cv::resizeWindow("frenet", 1920,1080);
    cv::imwrite(filename.str(), bg);
    cv::waitKey(5);
  }
  return 0;
};
*/