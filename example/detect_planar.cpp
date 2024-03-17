#include <iostream>
#include <array>
#include <vector>
#include <chrono>
#include <string>
#include <random>
#include <limits>
#include <functional>
#include <memory>

#include <Eigen/Dense>
#include <matplot/matplot.h>
#include <open3d/Open3D.h>

#include "math_utils.hpp"
#include "plot_helper.hpp"

namespace plt = matplot;

constexpr size_t D = 9;

int main()
{
    const double point_cloud_range = 1.0;

    std::mt19937 engine(0);
    std::uniform_real_distribution<> pos_rand(-point_cloud_range, point_cloud_range);
    std::uniform_real_distribution<> error_rand(-0.1, 0.1);

    constexpr size_t point_cloud_num = 6000;

    std::vector<Eigen::Vector3d> plane_x;
    std::vector<Eigen::Vector3d> plane_y;
    std::vector<Eigen::Vector3d> plane_z;

    plane_x.reserve(point_cloud_num / 3);
    plane_y.reserve(point_cloud_num / 3);
    plane_z.reserve(point_cloud_num / 3);
    for (size_t i = 0; i < point_cloud_num / 3; i++) {
        plane_z.push_back(Eigen::Vector3d(pos_rand(engine) + error_rand(engine), pos_rand(engine) + error_rand(engine), 0.0));
        plane_y.push_back(Eigen::Vector3d(pos_rand(engine) + error_rand(engine), point_cloud_range, pos_rand(engine) + error_rand(engine) + point_cloud_range));
        plane_x.push_back(Eigen::Vector3d(point_cloud_range, pos_rand(engine) + error_rand(engine), pos_rand(engine) + error_rand(engine) + point_cloud_range));

        // plane_z.push_back(Eigen::Vector3d(pos_rand(engine) + error_rand(engine) - 2, pos_rand(engine) + error_rand(engine) + 1, 0.0));
        // plane_y.push_back(Eigen::Vector3d(pos_rand(engine) + error_rand(engine) + 3, 0.0, pos_rand(engine) + error_rand(engine) - 1));
        // plane_x.push_back(Eigen::Vector3d(0.0, pos_rand(engine) + error_rand(engine) + 1, pos_rand(engine) + error_rand(engine) + 3));
    }

    auto plane_x_pcd = std::make_shared<open3d::geometry::PointCloud>(plane_x);
    plane_x_pcd->PaintUniformColor({1.0, 0.0, 0.0});
    auto plane_y_pcd = std::make_shared<open3d::geometry::PointCloud>(plane_y);
    plane_y_pcd->PaintUniformColor({0.0, 1.0, 0.0});
    auto plane_z_pcd = std::make_shared<open3d::geometry::PointCloud>(plane_z);
    plane_z_pcd->PaintUniformColor({0.0, 0.0, 1.0});
    // open3d::visualization::DrawGeometries({plane_x_pcd, plane_y_pcd, plane_z_pcd});

    // auto all_pcd = std::make_shared<open3d::geometry::PointCloud>(*plane_x_pcd + *plane_y_pcd + *plane_z_pcd);
    // all_pcd->EstimateNormals();
    // open3d::visualization::DrawGeometries({all_pcd}, "Point Cloud", 640, 480, 50, 50, true);

    auto all_pcd = open3d::io::CreatePointCloudFromFile("dataset/fragment.ply");
    // open3d::visualization::DrawGeometries({all_pcd}, "Point Cloud", 640, 480, 50, 50, false);
    // all_pcd = all_pcd->FarthestPointDownSample(point_cloud_num);
    // all_pcd->EstimateNormals();
    // open3d::visualization::DrawGeometries({all_pcd}, "Point Cloud", 640, 480, 50, 50, false);

    auto patches = all_pcd->DetectPlanarPatches();
    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> geometries;
    geometries.reserve(patches.size());
    for (const auto& obox : patches) {
        std::vector<size_t> idxs = obox->GetPointIndicesWithinBoundingBox(all_pcd->points_);
        for (size_t j = 0; j < idxs.size(); ++j) {
            all_pcd->colors_[idxs[j]] = obox->color_;
        }
        geometries.push_back(obox);

        // turn bounding box into a plane
        auto mesh = open3d::geometry::TriangleMesh::CreateFromOrientedBoundingBox(*obox, Eigen::Vector3d(1, 1, 0.0001));
        mesh->PaintUniformColor(obox->color_);
        geometries.push_back(mesh);
    }
    // visualize point cloud, too
    geometries.push_back(all_pcd);
    open3d::visualization::DrawGeometries(geometries, "Visualize", 1600, 900);
}
