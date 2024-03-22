#include <glog/logging.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2_msgs/TFMessage.h>
#include "omp.h"
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <string>
#include <algorithm>
#include <unordered_map>
#include <map>
#include <cmath>
#include <iostream>
#include "mypcl_cloud_type.h"
#include "voxelMap.hpp"
#include "state_ori.hpp"
#include <random>
#include <pcl/io/ply_io.h>

// #include "../odometry/lidarEgoVelocityEstimator.hpp"
#define HASH_P 116101
#define MAX_N 10000000000

using namespace std;
using namespace Eigen;

string lidar_topic;

double leaf_size = 0.2;
bool save_result = false;
double range_threshold = 100;
double range_cov = 0.02;
double angle_cov = 0.05;
double voxel_size = 1.0;
double min_eigen_value = 0.1;
double res_mean_last = 0.5;
double gyr_cov_scale, acc_cov_scale;
int point_filter_num = 1;
int max_layer = 3;
int NUM_MAX_ITERATIONS = 3;
int max_points_size = 100;
int max_cov_points_size = 100;
std::vector<int> layer_size;
Eigen::Vector3d ext;
vector<double> layer_point_size;
int cut_lidar_num = 6;
bool flg_EKF_converged, init_map = false, EKF_stop_flag = 0, flg_EKF_inited = 0;
bool dense_map_enable;
int pub_point_cloud_skip;
std::unordered_map<VOXEL_LOC, OctoTree *> voxel_map;
geometry_msgs::Quaternion geoQuat;
// record time
double undistort_time_mean = 0;
double down_sample_time_mean = 0;
double calc_cov_time_mean = 0;
double scan_match_time_mean = 0;
double ekf_solve_time_mean = 0;
double map_update_time_mean = 0;
double search_time = 0;

double map_incremental_time, kdtree_search_time, total_time, scan_match_time,
    solve_time;
int iterCount, frameCount = 0;
VD(DIM_STATE)
solution;
MD(DIM_STATE, DIM_STATE)
G, H_T_H, I_STATE;
V3D rot_add, t_add, v_add;
StatesGroup state_propagat, last_state, lastlast_state;
double deltaT, deltaR, aver_time_consu = 0;
V3D euler_cur;
V3D position_last(Zero3d);
shared_ptr<State_Process> process_iekf(new State_Process());
ros::Publisher pubOdometry, pubPath, pubLaserCloudFullRes;
nav_msgs::Path path;
std_msgs::Header first_header;
std_msgs::Header current_header;
std::vector<std::vector<pointWithCov>> PointCloudVector;
std::string DATASET_PATH, OUTPUT_PATH, time_consu_root;

void sub_sample(RTVPointCloud::Ptr &input_points, double size_voxel, double range)
{
    unordered_map<VOXEL_LOC, vector<RTVPoint>> grid_map;
    for (uint i = 0; i < input_points->size(); i++)
    {
        if (sqrt(input_points->points[i].x * input_points->points[i].x + input_points->points[i].y * input_points->points[i].y + input_points->points[i].z * input_points->points[i].z) < 150)
        {
            float loc_xyz[3];
            loc_xyz[0] = input_points->points[i].x / size_voxel;
            loc_xyz[1] = input_points->points[i].y / size_voxel;
            loc_xyz[2] = input_points->points[i].z / size_voxel;
            for (int j = 0; j < 3; j++)
            {
                if (loc_xyz[j] < 0)
                {
                    loc_xyz[j] -= 1.0;
                }
            }
            VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
            grid_map[position].push_back(input_points->points[i]);
        }
    }

    input_points->resize(0);
    for (const auto &n : grid_map)
    {
        if (n.second.size() > 0)
        {
            input_points->points.push_back(n.second[0]);
        }
    }
}

StatesGroup state;
std::vector<StatesGroup> state_list;
// std::vector<StatesGroup> predict_list;
RTVPointCloud::Ptr normvec(new RTVPointCloud(100000, 1));
RTVPointCloud::Ptr laserCloudOri(new RTVPointCloud(100000, 1));
RTVPointCloud::Ptr laserCloudNoeffect(new RTVPointCloud(100000, 1));
RTVPointCloud::Ptr corr_normvect(new RTVPointCloud(100000, 1));
RTVPointCloud::Ptr laserCloudRaw(new RTVPointCloud(100000, 1));

const bool time_list(RTVPoint &x, RTVPoint &y)
{
    return (x.time < y.time);
};
const bool var_contrast(pointWithCov &x, pointWithCov &y)
{
    return (x.cov.diagonal().norm() < y.cov.diagonal().norm());
};

void pointBodyToWorld(RTVPoint const *const pi, RTVPoint *const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state.rot * (p_body) + state.pos);
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->time = pi->time;
    po->velocity = pi->velocity;
}
void pointBodyToWorld(const StatesGroup &state_, RTVPoint const *const pi, RTVPoint *const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_.rot * (p_body) + state_.pos);
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->time = pi->time;
    po->velocity = pi->velocity;
}

void transformLidar(const StatesGroup &state, const RTVPointCloud::Ptr &input_cloud, RTVPointCloud::Ptr &trans_cloud)
{
    trans_cloud->clear();
    for (size_t i = 0; i < input_cloud->size(); i++)
    {
        Eigen::Vector3d p;
        p << input_cloud->points[i].x, input_cloud->points[i].y, input_cloud->points[i].z;
        Eigen::Vector3d po;
        po = state.rot * p + state.pos;
        RTVPoint pi;
        pi.x = po(0);
        pi.y = po(1);
        pi.z = po(2);

        pi.velocity = input_cloud->points[i].velocity;
        pi.time = input_cloud->points[i].time;

        trans_cloud->points.push_back(pi);
    }
}

void motionCompensate( std::vector<pointWithCov> &point_raw )
{
    double t_end = point_raw.back().time;
    for (int i = 0; i < point_raw.size(); i++)
    {
            double doppler_speed = point_raw[i].radial_vel;
            Eigen::Vector3d point3d = point_raw[i].point;
            double normal = std::sqrt(point3d.transpose() * point3d);
            Eigen::Vector3d linear_vel = (doppler_speed * point3d) / normal;
            point_raw[i].vel = linear_vel;
            double w_z = linear_vel.y() / ext.x() * 0.01;
            double w_y = linear_vel.z() / ext.z() * 0.01;
            Eigen::Vector3d angular_vel = Eigen::Vector3d(0, w_y, -w_z);
            point_raw[i].gyr = angular_vel;
            double delta_time = t_end - point_raw[i].time;
            Eigen::Vector3d aft_point = Exp(angular_vel, delta_time) * point3d + linear_vel * delta_time;
            point_raw[i].point = aft_point;
    }
    
}

void cutLidarFrame(const RTVPointCloud::Ptr &input_cloud)
{
    double time_begin = input_cloud->front().time;
    double time_end = input_cloud->back().time;
    double time_rela = time_end - time_begin;
    LOG(INFO) << "time_rela:" << time_rela << endl;
    double point_size = input_cloud->points.size();
    double slice_alpha = point_size / (cut_lidar_num);
    // std::vector<std::vector<Point3D>> slices;

    for (int i = 0; i < cut_lidar_num; i++)
    {
        std::vector<pointWithCov>().swap(PointCloudVector[i]);
    }

    for (int i = 0; i < input_cloud->size(); i++)
    {
        auto keypoint = input_cloud->points[i];

        int id = int(i / slice_alpha);
        if (id == cut_lidar_num)
            continue;
        pointWithCov pointWC;
        pointWC.point = Eigen::Vector3d(keypoint.x, keypoint.y, keypoint.z);
        pointWC.time = keypoint.time;
        pointWC.radial_vel = keypoint.velocity;

        PointCloudVector[id].push_back(pointWC);
    }
}

void pclCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    current_header = msg->header;
    RTVPointCloud::Ptr ptr(new RTVPointCloud());

    pcl::fromROSMsg(*msg, *ptr);

    auto t_downsample_start = std::chrono::high_resolution_clock::now();
    sub_sample(ptr, leaf_size, range_threshold);

    RTVPointCloud::Ptr raw_cloud(new RTVPointCloud());

    int i = 0;
    for (auto point : ptr->points)
    {
        if (pcl_isnan(point.x) || pcl_isnan(point.y) || pcl_isnan(point.z) || point.x == 0 || point.y == 0 || point.z == 0)
            continue;
        if (i % point_filter_num == 0)
        {
            raw_cloud->points.push_back(point);
        }
        i++;
    }
    // cout << "down_sample.size():" << raw_cloud->size() << endl;
    sort(raw_cloud->points.begin(), raw_cloud->points.end(), time_list);

    // auto undistort_end = std::chrono::high_resolution_clock::now();
    // auto undistort_time = std::chrono::duration_cast<std::chrono::duration<double>>(undistort_end - undistort_start).count() * 1000;
    // cout<<"undistort_time="<<undistort_time<<"ms"<<std::endl;
    auto t_downsample_end = std::chrono::high_resolution_clock::now();
    auto t_downsample = std::chrono::duration_cast<std::chrono::duration<double>>(t_downsample_end - t_downsample_start).count() * 1000;

    if (!init_map)
    {
        process_iekf->Reset();
        first_header = current_header;
        process_iekf->only_propag(ptr, current_header.stamp.toSec(), state);

        LOG(INFO) << "voxelMap inited!!!!" << endl;
        RTVPointCloud::Ptr world_lidar(new RTVPointCloud());
        transformLidar(state, ptr, world_lidar);
        vector<pointWithCov> pv_list;
        for (size_t i = 0; i < world_lidar->size(); i++)
        {
            pointWithCov pv;
            pv.point << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;

            Eigen::Vector3d point_this(ptr->points[i].x, ptr->points[i].y, ptr->points[i].z);
            if (point_this[2] == 0)
            {
                point_this[2] = 0.001;
            }

            Eigen::Matrix3d cov;
            calcBodyCov(point_this, range_cov, angle_cov, cov);
            M3D point_crossmat;
            point_crossmat << SKEW_SYM_MATRX(point_this);
            cov = state.rot * cov * state.rot.transpose() + (-point_crossmat) * state.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() + state.cov.block<3, 3>(3, 3);

            pv.cov = cov;
            pv_list.push_back(pv);
        }
        last_state = state;
        buildVoxelMap(pv_list, voxel_size, max_layer, layer_size, max_points_size,
                      max_points_size, min_eigen_value, voxel_map);
        init_map = true;
        // if (save_result) {
        //   kitti_log(fp_kitti);
        // }
        return;
    }

    cout << "frame count:" << frameCount << endl;

    // auto t_downsample_start = std::chrono::high_resolution_clock::now();
    // auto t_downsample_end = std::chrono::high_resolution_clock::now();
    // auto t_downsample = std::chrono::duration_cast<std::chrono::duration<double>>(t_downsample_end - t_downsample_start).count() * 1000;
    auto t_solve_eskf_start = std::chrono::high_resolution_clock::now();

    scan_match_time = 0.0;

    cutLidarFrame(raw_cloud);
    state_list.clear();
    // predict_list.clear();

    for (int i = 0; i < cut_lidar_num; i++)
    {
        motionCompensate(PointCloudVector[i]);

        auto pv_list = PointCloudVector[i];
        double timestamp = current_header.stamp.toSec();
        //   double timestamp = pv_list[0].time;

        auto state_update = process_iekf->one_point_propag(timestamp, state, pv_list);
        // auto state_update = process_iekf->pv_propag(timestamp, state, pv_list);
        bool nearest_search_en = true;
        double total_residual;
        int rematch_num = 0;

        vector<M3D> body_var;
        vector<M3D> crossmat_list;
        // 计算cov
        for (size_t j = 0; j < pv_list.size(); j++)
        {
            V3D point_this = pv_list[j].point;

            if (point_this[2] == 0)
            {
                point_this[2] = 0.001;
            }

            M3D cov;
            calcBodyCov(point_this, range_cov, angle_cov, cov);
            M3D point_crossmat;
            point_crossmat << SKEW_SYM_MATRX(point_this);
            crossmat_list.push_back(point_crossmat);
            body_var.push_back(cov);
        }
        state_propagat = state_update;
        // predict_list.push_back(state_propagat);
        for (iterCount = 0; iterCount < NUM_MAX_ITERATIONS; iterCount++)
        {
            laserCloudOri->clear();
            laserCloudNoeffect->clear();
            corr_normvect->clear();
            laserCloudRaw->clear();
            total_residual = 0.0;
            vector<ptpl> ptpl_list;
            vector<pointWithCov> pv_list_0;
            for (size_t j = 0; j < pv_list.size(); j++)
            {
                V3D point_this = pv_list[j].point;
                V3D point_world = state_update.rot * point_this + state_update.pos;
                M3D cov = body_var[j];
                M3D point_crossmat = crossmat_list[j];
                M3D rot_var = state_update.cov.block<3, 3>(0, 0);
                M3D t_var = state_update.cov.block<3, 3>(3, 3);
                cov = state_update.rot * cov * state_update.rot.transpose() + (-point_crossmat) * rot_var * (-point_crossmat.transpose()) + t_var;
                pointWithCov pv;
                pv.point = point_this;
                pv.point_world = point_world;
                pv.time = pv_list[j].time;
                pv.radial_vel = pv_list[j].radial_vel;
                pv.cov = cov;
                pv_list_0.push_back(pv);
            }
            auto t_search_begin = std::chrono::high_resolution_clock::now();

            std::vector<V3D> non_match_list;

            BuildResidualListOMP(voxel_map, voxel_size, 3.0, max_layer, pv_list_0, ptpl_list, non_match_list);
            auto t_search_end = std::chrono::high_resolution_clock::now();
            auto t_search = std::chrono::duration_cast<std::chrono::duration<double>>(t_search_end - t_search_begin).count() * 1000;
            search_time += t_search;

            int effect_feat_num = 0;
            for (int j = 0; j < ptpl_list.size(); j++)
            {
                RTVPoint pi_body;
                RTVPoint pi_raw;
                RTVPoint pi_world;
                RTVPoint pl;

                pi_body.x = ptpl_list[j].point(0);
                pi_body.y = ptpl_list[j].point(1);
                pi_body.z = ptpl_list[j].point(2);
                pi_body.time = ptpl_list[j].time;
                pi_body.velocity = ptpl_list[j].radial_vel;
                // pointBodyToWorld(state_update,&pi_body, &pi_world);
                pi_world.x = ptpl_list[j].point_world(0);
                pi_world.y = ptpl_list[j].point_world(1);
                pi_world.z = ptpl_list[j].point_world(2);

                pl.x = ptpl_list[j].normal(0);
                pl.y = ptpl_list[j].normal(1);
                pl.z = ptpl_list[j].normal(2);
                effect_feat_num++;
                float dis = (pi_world.x * pl.x + pi_world.y * pl.y + pi_world.z * pl.z + ptpl_list[j].d);
                pl.intensity = dis;
                laserCloudRaw->push_back(pi_raw);
                laserCloudOri->push_back(pi_body);
                corr_normvect->push_back(pl);
                total_residual += fabs(dis);
            }

            res_mean_last = total_residual / effect_feat_num;
            MatrixXd Hsub(effect_feat_num * 2, DIM_STATE);
            MatrixXd Hsub_T_R_inv(DIM_STATE, 2 * effect_feat_num);
            VectorXd R_inv(effect_feat_num);
            VectorXd meas_vec(effect_feat_num * 2);
            Hsub.setZero();
            Hsub_T_R_inv.setZero();
            R_inv.setZero();
            meas_vec.setZero();

            for (int j = 0; j < effect_feat_num; j++)
            {
                const RTVPoint &laser_p = laserCloudOri->points[j];
                V3D point_this(laser_p.x, laser_p.y, laser_p.z);
                M3D cov;
                calcBodyCov(point_this, range_cov, angle_cov, cov);

                cov = state_update.rot * cov * state_update.rot.transpose();

                M3D point_crossmat;
                point_crossmat << SKEW_SYM_MATRX(point_this);

                const RTVPoint &norm_p = corr_normvect->points[j];
                V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);
                V3D point_world = state_update.rot * point_this + state_update.pos;

                // /*** get the normal vector of closest surface/corner ***/
                Eigen::Matrix<double, 1, 6> J_nq;
                J_nq.block<1, 3>(0, 0) = point_world - ptpl_list[j].center;
                J_nq.block<1, 3>(0, 3) = -ptpl_list[j].normal;
                double sigma_l = J_nq * ptpl_list[j].plane_cov * J_nq.transpose();
                // if(frame_count)
                //doppler weight
                double sigma_doppler = 0;
                if (frameCount > 3)
                {
                    V3D v_ref = lastlast_state.rot.inverse() * (last_state.pos - lastlast_state.pos) / (last_state.timestamp - lastlast_state.timestamp);

                    double denominator = sqrt(point_this.transpose() * point_this);
                    V3D point_norm = point_this / denominator;
                //     // V3D vel_body = -state_update.rot.inverse() * v_ref;
                    double esti_doppler = point_norm.transpose() * v_ref;
                    double error = laser_p.velocity - esti_doppler;
                    sigma_doppler = 1 / (1 + error * error);
                }

                R_inv(j) = 1 / (sigma_doppler + sigma_l + norm_vec.transpose() * cov * norm_vec);
                /*** calculate the Measuremnt Jacobian matrix H ***/

                V3D A(point_crossmat * state_update.rot.transpose() * norm_vec);

                Hsub.block<1, 6>(j, 0) << VEC_FROM_ARRAY(A), norm_p.x, norm_p.y, norm_p.z;

                Hsub_T_R_inv.block<6, 1>(0, j) << A[0] * R_inv(j), A[1] * R_inv(j),
                    A[2] * R_inv(j), norm_p.x * R_inv(j), norm_p.y * R_inv(j),
                    norm_p.z * R_inv(j);
                meas_vec(j) = -norm_p.intensity;

            }
            MatrixXd K(DIM_STATE, effect_feat_num * 2);
            // LOG(INFO)<<"effect_feat_num:"<<effect_feat_num<<endl;
            EKF_stop_flag = false;
            flg_EKF_converged = false;
            /*** Iterative Kalman Filter Update ***/
            // flg_EKF_inited = true;

            if (!flg_EKF_inited)
            {
                flg_EKF_inited = true;
                cout << "||||||||||Initiallizing LiDar||||||||||" << endl;
                /*** only run in initialization period ***/
                MatrixXd H_init(MD(9, DIM_STATE)::Zero());
                MatrixXd z_init(VD(9)::Zero());
                H_init.block<3, 3>(0, 0) = M3D::Identity();
                H_init.block<3, 3>(3, 3) = M3D::Identity();

                z_init.block<3, 1>(0, 0) = -Log(state_update.rot);

                z_init.block<3, 1>(3, 0) = -state_update.pos;

                auto H_init_T = H_init.transpose();

                auto &&K_init = state_update.cov * H_init_T * (H_init * state_update.cov * H_init_T + 0.0001 * MD(9, 9)::Identity()).inverse();
                solution = K_init * z_init;
                // cout<<"solution:"<<solution.transpose()<<endl;
                state_update.resetpose();
                EKF_stop_flag = true;
            }
            else
            {
                // cout<<"|||||||||||||||||||KF_update||||||||||||||||||||||||"<<endl;

                auto &&Hsub_T = Hsub.transpose();

                H_T_H = Hsub_T_R_inv * Hsub;

                MD(DIM_STATE, DIM_STATE) &&K_1 = (H_T_H + (state_update.cov).inverse()).inverse();
                K = K_1 * Hsub_T_R_inv;

                auto vec = state_propagat - state_update;

                solution = K * meas_vec + vec - K * Hsub * vec;

                state_update += solution;
                rot_add = solution.block<3, 1>(0, 0);
                t_add = solution.block<3, 1>(3, 0);
                v_add = solution.block<3, 1>(6, 0);

                if ((rot_add.norm() * 57.3) < 0.01 & (t_add.norm()) * 100 < 0.015 & (v_add.norm()) * 100 < 0.015)
                {
                    flg_EKF_converged = true;
                }
            }
            euler_cur = RotMtoEuler(state_update.rot);
            /*** Rematch Judgement ***/
            nearest_search_en = false;
            // LOG(INFO)<<"flg_EKF_converged:"<<flg_EKF_converged<<endl;
            if (flg_EKF_converged || (rematch_num == 0) && (iterCount == (NUM_MAX_ITERATIONS - 2)))
            {
                nearest_search_en = true;
                rematch_num++;
            }

            if (!EKF_stop_flag && (rematch_num >= 2 || (iterCount == NUM_MAX_ITERATIONS - 1)))
            {
                if (flg_EKF_inited)
                {
                    /*** Covariance Update ***/
                    G.setZero();
                    // G.block<DIM_STATE, 6>(0, 0) = K * Hsub;

                    G = K * Hsub;
                    state_update.cov = (I_STATE - G) * state_update.cov;

                    // total_residual += (state_update.pos - position_last).norm();
                    // position_last = state_update.pos;
                    geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));
                    // LOG(INFO)<<"iterCount:"<<iterCount<<endl;

                    // LOG(INFO)<<"rematch_num:"<<rematch_num<<endl;
                }
                EKF_stop_flag = true;
            }
            if (EKF_stop_flag)
                break;
        }
        state = state_update;
        lastlast_state = last_state;
        last_state = state;
        state_list.push_back(state_update);
    }
    auto t_solve_eskf_end = std::chrono::high_resolution_clock::now();
    solve_time = std::chrono::duration_cast<std::chrono::duration<double>>(t_solve_eskf_end - t_solve_eskf_start).count() * 1000;

    auto map_incremental_start = std::chrono::high_resolution_clock::now();

    RTVPointCloud::Ptr world_lidar(new RTVPointCloud());
    world_lidar->clear();
    std::vector<pointWithCov> pv_list_add;

    for (size_t i = 0; i < cut_lidar_num; i++)
    {

        auto pv_list = PointCloudVector[i];
        StatesGroup state_update;

        state_update = state_list[i];

        for (size_t j = 0; j < pv_list.size(); j++)
        {

            pointWithCov pv; // = pv_list[j];
            V3D point_this = pv_list[j].point;
            pv.point = state_update.rot * point_this + state_update.pos;
            M3D point_crossmat;
            point_crossmat << SKEW_SYM_MATRX(point_this);
            M3D cov;
            calcBodyCov(point_this, range_cov, angle_cov, cov);
            cov = state_update.rot * cov * state_update.rot.transpose() +
                  (-point_crossmat) * state_update.cov.block<3, 3>(0, 0) *
                      (-point_crossmat).transpose() +
                  state_update.cov.block<3, 3>(3, 3);
            pv.cov = cov;
            pv_list_add.push_back(pv);
            RTVPoint point_in_world;
            point_in_world.x = pv.point.x();
            point_in_world.y = pv.point.y();
            point_in_world.z = pv.point.z();

            world_lidar->points.push_back(point_in_world);
        }
    }

    std::sort(pv_list_add.begin(), pv_list_add.end(), var_contrast);
    updateVoxelMap(pv_list_add, voxel_size, max_layer, layer_size, max_points_size, max_points_size, min_eigen_value, voxel_map);
    state = state_list.back();
    euler_cur = RotMtoEuler(state.rot);
    geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));

    auto map_incremental_end = std::chrono::high_resolution_clock::now();
    map_incremental_time = std::chrono::duration_cast<std::chrono::duration<double>>(map_incremental_end - map_incremental_start).count() * 1000;

    total_time = t_downsample + solve_time + map_incremental_time; //+ calc_point_cov_time;
    if (save_result)
    {
        ofstream foutC(time_consu_root, ios::app);
        foutC.setf(ios::fixed, ios::floatfield);
        foutC.precision(9);
        foutC << frameCount << "," << t_downsample << "," << solve_time << "," << map_incremental_time << "," << total_time << "," << search_time << std::endl;
        foutC.close();
    }
    search_time = 0;
    frameCount++;
    nav_msgs::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = "world";
    odomAftMapped.child_frame_id = "lidar";
    odomAftMapped.header.stamp = current_header.stamp;
    odomAftMapped.pose.pose.position.x = state.pos(0);
    odomAftMapped.pose.pose.position.y = state.pos(1);
    odomAftMapped.pose.pose.position.z = state.pos(2);
    odomAftMapped.pose.pose.orientation.x = geoQuat.x;
    odomAftMapped.pose.pose.orientation.y = geoQuat.y;
    odomAftMapped.pose.pose.orientation.z = geoQuat.z;
    odomAftMapped.pose.pose.orientation.w = geoQuat.w;

    if (save_result)
    {
        ofstream foutC(OUTPUT_PATH, ios::app);
        foutC.setf(ios::fixed, ios::floatfield);
        foutC.precision(5);
        foutC << current_header.stamp.toSec() - first_header.stamp.toSec() << " " << state.pos(0) << " " << state.pos(1) << " " << state.pos(2) << " " << geoQuat.x
              << " " << geoQuat.y << " " << geoQuat.z << " " << geoQuat.w << std::endl;
        foutC.close();
    }
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(state.pos(0), state.pos(1), state.pos(2)));
    q.setW(geoQuat.w);
    q.setX(geoQuat.x);
    q.setY(geoQuat.y);
    q.setZ(geoQuat.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "world", "lidar"));
    pubOdometry.publish(odomAftMapped);
    geometry_msgs::PoseStamped body_pose;
    body_pose.header.stamp = current_header.stamp;
    body_pose.header.frame_id = "world";
    body_pose.pose.position.x = state.pos(0);
    body_pose.pose.position.y = state.pos(1);
    body_pose.pose.position.z = state.pos(2);
    body_pose.pose.orientation.x = geoQuat.x;
    body_pose.pose.orientation.y = geoQuat.y;
    body_pose.pose.orientation.z = geoQuat.z;
    body_pose.pose.orientation.w = geoQuat.w;

    path.header.stamp = current_header.stamp;
    path.header.frame_id = "world";
    path.poses.push_back(body_pose);
    pubPath.publish(path);
    sensor_msgs::PointCloud2 pub_cloud;
    pcl::toROSMsg(*world_lidar, pub_cloud);
    pub_cloud.header.stamp = current_header.stamp;
    pub_cloud.header.frame_id = "world";
    if (frameCount % pub_point_cloud_skip == 0)
    {
        pubLaserCloudFullRes.publish(pub_cloud);
    }

    down_sample_time_mean =
        down_sample_time_mean * (frameCount - 1) / frameCount +
        (t_downsample) / frameCount;

    ekf_solve_time_mean = ekf_solve_time_mean * (frameCount - 1) / frameCount +
                          (solve_time) / frameCount;
    map_update_time_mean =
        map_update_time_mean * (frameCount - 1) / frameCount +
        (map_incremental_time) / frameCount;

    aver_time_consu = aver_time_consu * (frameCount - 1) / frameCount +
                      (total_time) / frameCount;


    cout << "[ Time ]: "
         << "average down sample: " << down_sample_time_mean << "ms" << std::endl;
    cout << "[ Time ]: "
         << "average solve: " << ekf_solve_time_mean << "ms" << std::endl;
    cout << "[ Time ]: "
         << "average map incremental: " << map_update_time_mean << "ms" << std::endl;
    cout << "[ Time ]: "
         << " average total " << aver_time_consu << "ms" << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "4dlo");
    ros::NodeHandle nh;
    nh.param<string>("common/lid_topic", lidar_topic, "/lidar_topic");
    nh.param<string>("common/input_path", DATASET_PATH, "/");
    nh.param<string>("common/output_path", OUTPUT_PATH, "/");
    nh.param<string>("common/time_path", time_consu_root, "/");
    nh.param<bool>("common/save_result", save_result, false);
    nh.param<double>("mapping/leaf_size", leaf_size, 0.2);
    nh.param<int>("mapping/cut_lidar_num", cut_lidar_num, 1);
    nh.param<double>("mapping/range_threshold", range_threshold, 100);
    nh.param<double>("noise_model/range_cov", range_cov, 0.02);
    nh.param<double>("noise_model/angle_cov", angle_cov, 0.05);
    nh.param<double>("noise_model/acc_cov_scale", acc_cov_scale, 0.1);
    nh.param<double>("noise_model/gyr_cov_scale", gyr_cov_scale, 0.1);
    nh.param<double>("mapping/voxel_size", voxel_size, 1.0);
    nh.param<double>("mapping/plannar_threshold", min_eigen_value, 0.1);
    nh.param<int>("mapping/max_layer", max_layer, 3);
    nh.param<int>("mapping/max_iteration", NUM_MAX_ITERATIONS, 3);
    nh.param<int>("mapping/max_points_size", max_points_size, 100);
    nh.param<int>("mapping/max_cov_points_size", max_cov_points_size, 100);
    nh.param<int>("mapping/point_filter_num", point_filter_num, 1);

    nh.param<vector<double>>("mapping/layer_point_size", layer_point_size, vector<double>());
    for (int i = 0; i < layer_point_size.size(); i++)
    {
        layer_size.push_back(layer_point_size[i]);
    }
    nh.param<bool>("visualization/dense_map_enable", dense_map_enable, true);
    nh.param<int>("visualization/pub_point_cloud_skip", pub_point_cloud_skip, 5);
    std::vector<double> extrinT;
    nh.param<vector<double>>("extrinsic_T_v_l", extrinT, vector<double>());
    ext << extrinT[0], extrinT[1], extrinT[2];

    pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100);
    pubOdometry = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
    pubPath = nh.advertise<nav_msgs::Path>("/path", 10);
    ros::Subscriber sub_pcl;
    LOG(INFO) << "cut_lidar_num:" << cut_lidar_num << endl;
    for (int i = 0; i < cut_lidar_num; i++)
    {
        std::vector<pointWithCov> v_point_temp;
        PointCloudVector.push_back(v_point_temp);
    }
    LOG(INFO) << "INPUT_PATH is " << DATASET_PATH;
    LOG(INFO) << "OUTPUT_PATH is " << OUTPUT_PATH;
    // state_list.resize(cut_lidar_num);
    process_iekf->set_acc_cov_scale(V3D(acc_cov_scale, acc_cov_scale, acc_cov_scale));
    process_iekf->set_gyr_cov_scale(V3D(gyr_cov_scale, gyr_cov_scale, gyr_cov_scale));
    G.setZero();
    H_T_H.setZero();
    I_STATE.setIdentity();

    
    sub_pcl = nh.subscribe(lidar_topic, 2000, pclCallBack);
    

    ros::spin();

    return 0;
}