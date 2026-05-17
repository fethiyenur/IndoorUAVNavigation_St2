#define _CRT_SECURE_NO_WARNINGS
#include <algorithm>          // <-- bunu ekle, std::max buradan geliyor
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <iostream>
#include <vector>
#include <sstream>
#include <iomanip>
int main()
{
    using PointT = pcl::PointXYZ;

    std::string base_path =
        "C:\\location2_sensorrange_005_7\\";

    // ============================================================
    // MERGE + FİLTRELEME (ÖNERI parametreleri)
    // VoxelGrid: 0.20f | SOR: MeanK=8, Thresh=1.0f | ROR: R=0.5, MinN=3
    // ============================================================

    std::vector<int> rotations = { 0, 45, 90, 135, 180, 225, 270, 315 };
    pcl::PointCloud<PointT>::Ptr merged_cloud(new pcl::PointCloud<PointT>);

    for (int index = 1; index <= 16; ++index)
    {
        int rotation = rotations[(index - 1) % 8];
        std::string height = (index <= 8) ? "075" : "220";

        std::stringstream ss;
        ss << base_path
            << std::setw(2) << std::setfill('0') << index
            << "_location_2_rotation_"
            << std::setw(3) << std::setfill('0') << rotation
            << "_height_" << height << ".pcd";

        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        if (pcl::io::loadPCDFile<PointT>(ss.str(), *cloud) == -1)
        {
            std::cerr << "Atlanıyor (yuklenemedi): " << ss.str() << std::endl;
            continue;
        }
        *merged_cloud += *cloud;
    }

    if (merged_cloud->empty())
    {
        std::cerr << "Hic nokta okunamadi." << std::endl;
        return -1;
    }
    std::cout << "Merge sonrasi: " << merged_cloud->size() << " nokta" << std::endl;

    // --- Voxel Downsample ---
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(merged_cloud);
    voxel.setLeafSize(0.20f, 0.20f, 0.20f);          // ÖNERI: 0.20
    pcl::PointCloud<PointT>::Ptr voxel_cloud(new pcl::PointCloud<PointT>);
    voxel.filter(*voxel_cloud);
    std::cout << "Voxel sonrasi: " << voxel_cloud->size() << " nokta" << std::endl;

    // --- SOR ---
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(voxel_cloud);
    sor.setMeanK(8);                                  // ÖNERI: 8
    sor.setStddevMulThresh(1.0f);                     // ÖNERI: 1.0
    pcl::PointCloud<PointT>::Ptr sor_cloud(new pcl::PointCloud<PointT>);
    sor.filter(*sor_cloud);
    std::cout << "SOR sonrasi: " << sor_cloud->size() << " nokta" << std::endl;

    // --- ROR ---
    pcl::RadiusOutlierRemoval<PointT> ror;
    ror.setInputCloud(sor_cloud);
    ror.setRadiusSearch(0.5);                         // ÖNERI: 0.5
    ror.setMinNeighborsInRadius(3);                   // ÖNERI: 3
    pcl::PointCloud<PointT>::Ptr final_cloud(new pcl::PointCloud<PointT>);
    ror.filter(*final_cloud);
    std::cout << "ROR sonrasi: " << final_cloud->size() << " nokta" << std::endl;

    // --- Kaydet ---
    pcl::io::savePCDFileBinary(base_path + "Location2_new_filtered_v2.pcd", *final_cloud);
    std::cout << "Kaydedildi: Location2_new_filtered_v2.pcd" << std::endl;

    std::cout << "\nTum islemler tamamlandi." << std::endl;
    std::cin.get();
    return 0;
}