#pragma once

#include <queue>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <tsl/robin_map.h>

#include "point.hpp"

struct Voxel {
  Voxel() = default;

  Voxel(short x, short y, short z) : x(x), y(y), z(z) {}

  bool operator==(const Voxel &vox) const { return x == vox.x && y == vox.y && z == vox.z; }

  inline bool operator<(const Voxel &vox) const {
    return x < vox.x || (x == vox.x && y < vox.y) || (x == vox.x && y == vox.y && z < vox.z);
  }

  inline static Voxel Coordinates(const Eigen::Vector3d &point, double voxel_size) {
    return {short(point.x() / voxel_size), short(point.y() / voxel_size), short(point.z() / voxel_size)};
  }

  short x;
  short y;
  short z;
};

using ArrayVector3d = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;

struct VoxelBlock {
  explicit VoxelBlock(int num_points = 20) : num_points_(num_points) { points.reserve(num_points); }

  bool IsFull() const { return num_points_ == static_cast<int>(points.size()); }

  void AddPoint(const Eigen::Vector3d &point) {
    if (num_points_ < (int)points.size())
      throw std::runtime_error{"voxel is full with size " + std::to_string(points.size())};
    points.push_back(point);
  }

  inline int NumPoints() const { return static_cast<int>(points.size()); }

  inline int Capacity() { return num_points_; }

  ArrayVector3d points;

 private:
  int num_points_;
};

using VoxelHashMap = tsl::robin_map<Voxel, VoxelBlock>;


// Specialization of std::hash for our custom type Voxel
namespace std {

template <>
struct hash<Voxel> {
  std::size_t operator()(const Voxel &vox) const {
    const size_t kP1 = 73856093;
    const size_t kP2 = 19349669;
    const size_t kP3 = 83492791;
    return vox.x * kP1 + vox.y * kP2 + vox.z * kP3;
  }
};

}  // namespace std

namespace ct_ekf{
class Map {
 public:
  ArrayVector3d pointcloud() const {
    ArrayVector3d points;
    points.reserve(size());
    for (auto &voxel : voxel_map_) {
      for (int i(0); i < voxel.second.NumPoints(); ++i) points.push_back(voxel.second.points[i]);
    }
    return points;
  }

  size_t size() const {
    size_t map_size(0);
    for (auto &voxel : voxel_map_) {
      map_size += (voxel.second).NumPoints();
    }
    return map_size;
  }

  void remove(const Eigen::Vector3d &location, double distance) {
    std::vector<Voxel> voxels_to_erase;
    for (auto &pair : voxel_map_) {
      Eigen::Vector3d pt = pair.second.points[0];
      if ((pt - location).squaredNorm() > (distance * distance)) {
        voxels_to_erase.push_back(pair.first);
      }
    }
    for (auto &vox : voxels_to_erase) voxel_map_.erase(vox);
  }

  void add(const std::vector<Point3D> &points, double voxel_size, int max_num_points_in_voxel,
           double min_distance_points, int min_num_points = 0) {
    for (const auto &point : points)
      add(point.pt, voxel_size, max_num_points_in_voxel, min_distance_points, min_num_points);
  }

  void add(const ArrayVector3d &points, double voxel_size, int max_num_points_in_voxel, double min_distance_points) {
    for (const auto &point : points) add(point, voxel_size, max_num_points_in_voxel, min_distance_points);
  }

  void add(const Eigen::Vector3d &point, double voxel_size, int max_num_points_in_voxel, double min_distance_points,
           int min_num_points = 0) {
    short kx = static_cast<short>(point[0] / voxel_size);
    short ky = static_cast<short>(point[1] / voxel_size);
    short kz = static_cast<short>(point[2] / voxel_size);

    VoxelHashMap::iterator search = voxel_map_.find(Voxel(kx, ky, kz));
    if (search != voxel_map_.end()) {
      auto &voxel_block = (search.value());

      if (!voxel_block.IsFull()) {
        double sq_dist_min_to_points = 10 * voxel_size * voxel_size;
        for (int i(0); i < voxel_block.NumPoints(); ++i) {
          auto &_point = voxel_block.points[i];
          double sq_dist = (_point - point).squaredNorm();
          if (sq_dist < sq_dist_min_to_points) {
            sq_dist_min_to_points = sq_dist;
          }
        }
        if (sq_dist_min_to_points > (min_distance_points * min_distance_points)) {
          if (min_num_points <= 0 || voxel_block.NumPoints() >= min_num_points) {
            voxel_block.AddPoint(point);
          }
        }
      }
    } else {
      if (min_num_points <= 0) {
        // Do not add points (avoids polluting the map)
        VoxelBlock block(max_num_points_in_voxel);
        block.AddPoint(point);
        voxel_map_[Voxel(kx, ky, kz)] = std::move(block);
      }
    }
  }

  using pair_distance_t = std::tuple<double, Eigen::Vector3d, Voxel>;

  struct Comparator {
    bool operator()(const pair_distance_t &left, const pair_distance_t &right) const {
      return std::get<0>(left) < std::get<0>(right);
    }
  };

  using priority_queue_t = std::priority_queue<pair_distance_t, std::vector<pair_distance_t>, Comparator>;

  ArrayVector3d searchNeighbors(const Eigen::Vector3d &point, int nb_voxels_visited, double size_voxel_map,
                                int max_num_neighbors, int threshold_voxel_capacity = 1,
                                std::vector<Voxel> *voxels = nullptr) {
    if (voxels != nullptr) voxels->reserve(max_num_neighbors);

    short kx = static_cast<short>(point[0] / size_voxel_map);
    short ky = static_cast<short>(point[1] / size_voxel_map);
    short kz = static_cast<short>(point[2] / size_voxel_map);

    priority_queue_t priority_queue;

    Voxel voxel(kx, ky, kz);
    for (short kxx = kx - nb_voxels_visited; kxx < kx + nb_voxels_visited + 1; ++kxx) {
      for (short kyy = ky - nb_voxels_visited; kyy < ky + nb_voxels_visited + 1; ++kyy) {
        for (short kzz = kz - nb_voxels_visited; kzz < kz + nb_voxels_visited + 1; ++kzz) {
          voxel.x = kxx;
          voxel.y = kyy;
          voxel.z = kzz;

          auto search = voxel_map_.find(voxel);
          if (search != voxel_map_.end()) {
            const auto &voxel_block = search.value();
            if (voxel_block.NumPoints() < threshold_voxel_capacity) continue;
            for (int i(0); i < voxel_block.NumPoints(); ++i) {
              auto &neighbor = voxel_block.points[i];
              double distance = (neighbor - point).norm();
              if (priority_queue.size() == (size_t)max_num_neighbors) {
                if (distance < std::get<0>(priority_queue.top())) {
                  priority_queue.pop();
                  priority_queue.emplace(distance, neighbor, voxel);
                }
              } else
                priority_queue.emplace(distance, neighbor, voxel);
            }
          }
        }
      }
    }

    auto size = priority_queue.size();
    ArrayVector3d closest_neighbors(size);
    if (voxels != nullptr) {
      voxels->resize(size);
    }
    for (int i = 0; i < (int)size; ++i) {
      closest_neighbors[size - 1 - i] = std::get<1>(priority_queue.top());
      if (voxels != nullptr) (*voxels)[size - 1 - i] = std::get<2>(priority_queue.top());
      priority_queue.pop();
    }

    return closest_neighbors;
  }

 private:
  VoxelHashMap voxel_map_;
};
}
