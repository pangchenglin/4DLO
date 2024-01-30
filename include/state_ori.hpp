#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include "mypcl_cloud_type.h"
#include "so3_math.h"
#include "eigen_utils.hpp"

struct StatesGroup{
    StatesGroup(){
        this->rot = Eigen::Matrix3d::Identity();
        this->pos = Eigen::Vector3d::Zero();
        this->vel = Eigen::Vector3d::Zero();
        this->bias_a = Eigen::Vector3d::Zero();
        this->bias_g = Eigen::Vector3d::Zero();
        this->cov = Eigen::Matrix<double,DIM_STATE,DIM_STATE>::Identity() * INIT_COV;
        this->F_x = Eigen::Matrix<double,DIM_STATE,DIM_STATE>::Zero();
    }


  StatesGroup(const StatesGroup &b) {
    this->rot = b.rot;
    this->pos = b.pos;
    this->vel = b.vel;
    this->bias_a = b.bias_a;
    this->bias_g = b.bias_g;
    this->cov = b.cov;
    this->F_x = b.F_x;
  };

  StatesGroup &operator=(const StatesGroup &b) {
    this->rot = b.rot;
    this->pos = b.pos;
    this->vel = b.vel;
    this->cov = b.cov;
    this->bias_a = b.bias_a;
    this->bias_g = b.bias_g;
    this->F_x = b.F_x;
    return *this;
  };

    StatesGroup operator+(const Eigen::Matrix<double, DIM_STATE, 1> &state_add) {
    StatesGroup a;
    a.rot = this->rot * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
    a.pos = this->pos + state_add.block<3, 1>(3, 0);
    a.vel = this->vel + state_add.block<3, 1>(6, 0);
    a.bias_g = this->bias_g + state_add.block<3, 1>(9, 0);
    a.bias_a = this->bias_a + state_add.block<3, 1>(12, 0);
    a.cov = this->cov;
    return a;
    };

    StatesGroup &operator+=(const Matrix<double, DIM_STATE, 1> &state_add) {
    this->rot =
    this->rot * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
    this->pos += state_add.block<3, 1>(3, 0);
    this->vel += state_add.block<3, 1>(6, 0);
    this->bias_g += state_add.block<3, 1>(9, 0);
    this->bias_a += state_add.block<3, 1>(12, 0);
    return *this;
    };

    Matrix<double, DIM_STATE, 1> operator-(const StatesGroup &b) {
    Matrix<double, DIM_STATE, 1> a;
    Eigen::Matrix3d rotd(b.rot.transpose() * this->rot);
    a.block<3, 1>(0, 0) = Log(rotd);
    a.block<3, 1>(3, 0) = this->pos - b.pos;
    a.block<3, 1>(6, 0) = this->vel - b.vel;
    a.block<3, 1>(9, 0) = this->bias_g - b.bias_g;
    a.block<3, 1>(12, 0) = this->bias_a - b.bias_a;
    return a;
    };

    void resetpose() {
    this->rot = Eigen::Matrix3d::Identity();
    this->pos = Eigen::Vector3d::Zero();
    this->vel = Eigen::Vector3d::Zero();
    }

    Eigen::Matrix3d rot;
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d bias_a;
    Eigen::Vector3d bias_g;
    Eigen::Matrix<double,DIM_STATE,DIM_STATE> F_x;
    Eigen::Matrix<double,DIM_STATE,DIM_STATE> cov;
};
/************************esti_normvector*******************************/

/************************Process************************/
class State_Process{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    State_Process();
    ~State_Process();

    void Reset();

    void only_propag(const RTVPointCloud::Ptr &meas, const double &timestamp, StatesGroup &state_input);
    void set_acc_cov_scale(const V3D &scalar);
    void set_gyr_cov_scale(const V3D &scalar);
    void set_timestamp(const double &timestamp);
    // void point_by_point_propag(const RTVPointCloud::Ptr &meas,const double &timestamp,std::vector<StatesGroup> &PoseVector );
    StatesGroup one_point_propag(const double timestamp,const StatesGroup &state_input,std::vector<pointWithCov> pv_list);

    V3D cov_acc;
    V3D cov_gyr;
    V3D cov_acc_scale;
    V3D cov_gyr_scale;
private:

    bool b_first_frame = true;
    double time_last_scan;
};

State_Process::State_Process()
: b_first_frame(true){
  cov_acc = V3D(0.1, 0.1, 0.1);
  cov_gyr = V3D(0.1, 0.1, 0.1);
  cov_acc_scale = V3D(1, 1, 1);
  cov_gyr_scale = V3D(1, 1, 1);
}

void State_Process::Reset(){
  cout<<"Reset Process"<<endl;
  time_last_scan = -1;
  b_first_frame = true;
}

void State_Process::set_gyr_cov_scale(const V3D &scaler) {
  cov_gyr_scale = scaler;
}

void State_Process::set_acc_cov_scale(const V3D &scaler) {
  cov_acc_scale = scaler;
}

StatesGroup State_Process::one_point_propag(const double timestamp,const StatesGroup &state_input,std::vector<pointWithCov> pv_list){
    cov_acc = Eye3d * cov_acc_scale;
    cov_gyr = Eye3d * cov_gyr_scale;
    double dt = timestamp + pv_list.back().time - time_last_scan;
    // double dt = pv_list.back().time - time_last_scan;

    cout<<"delta_t:"<<pv_list.back().time<<endl;
    cout<<"dt:"<<dt<<";pv_list:"<<pv_list.size()<<endl;
    MD(DIM_STATE,DIM_STATE) F_x,  cov_w;
    StatesGroup state_output = state_input;
    M3D Exp_f = Exp(state_input.bias_g,dt);

    F_x.setIdentity();
    cov_w.setZero();
    
    F_x.block<3, 3>(0, 0) = Exp(state_input.bias_g, -dt);
    F_x.block<3, 3>(0, 9) = Eye3d * dt;
    F_x.block<3, 3>(3, 6) = Eye3d * dt;
    cov_w.block<3, 3>(9, 9).diagonal() = cov_gyr * dt * dt; // for omega in constant model
    cov_w.block<3, 3>(6, 6).diagonal() = cov_acc * dt * dt; // for velocity in constant model

    state_output.F_x = F_x;
    state_output.cov = F_x * state_input.cov * F_x.transpose() + cov_w;
    state_output.rot = state_input.rot * Exp_f;
    state_output.pos = state_input.pos + state_input.vel * dt; 

    for(int i = 0; i < pv_list.size();i++){
      double t_time = pv_list[i].time + timestamp - time_last_scan;
      // double t_time = pv_list[i].time - time_last_scan;

      M3D R_i(state_input.rot * Exp(state_input.bias_g, t_time)); //点所在时刻的旋转
      V3D P_i = pv_list[i].point; //(it_pcl->x, it_pcl->y, it_pcl->z);                                   //点所在时刻的位置(雷达坐标系下)
      V3D T_ei(state_input.pos + state_input.vel * t_time + 0.5 * state_input.bias_a * t_time * t_time - state_output.pos); //从点所在的世界位置-雷达末尾世界位置
      V3D P_compensate =  state_output.rot.conjugate() * (R_i * P_i + T_ei) ; // not accurate!
      pv_list[i].point = P_compensate;
    }


    // time_last_scan = pv_list.back().time;

    time_last_scan = timestamp + pv_list.back().time;

    return state_output;
}

State_Process::~State_Process(){}


void State_Process::only_propag(const RTVPointCloud::Ptr &meas, const double &timestamp, StatesGroup &state_input)
{
    cov_acc = Eye3d * cov_acc_scale;
    cov_gyr = Eye3d * cov_gyr_scale;
    
    const double &pcl_beg_time = timestamp;
    const double &pcl_end_time = timestamp + meas->back().time;

    MD(DIM_STATE,DIM_STATE) F_x,  cov_w;
    double dt = 0;

    if(b_first_frame){
        dt = 0.1;
        b_first_frame = false;
        time_last_scan = pcl_beg_time;
    }else{
        dt = pcl_beg_time - time_last_scan;
        time_last_scan = pcl_beg_time;
    }

    
    M3D Exp_f = Exp(state_input.bias_g,dt);

    F_x.setIdentity();
    cov_w.setZero();
    
    F_x.block<3, 3>(0, 0) = Exp(state_input.bias_g, -dt);
    F_x.block<3, 3>(0, 9) = Eye3d * dt;
    F_x.block<3, 3>(3, 6) = Eye3d * dt;
    cov_w.block<3, 3>(9, 9).diagonal() = cov_gyr * dt * dt; // for omega in constant model
    cov_w.block<3, 3>(6, 6).diagonal() = cov_acc * dt * dt; // for velocity in constant model

    state_input.cov = F_x * state_input.cov * F_x.transpose() + cov_w;
    state_input.rot = state_input.rot * Exp_f;
    state_input.pos = state_input.pos + state_input.vel * dt; 
}

void State_Process::set_timestamp(const double &timestamp){
  this->time_last_scan = timestamp;
}



