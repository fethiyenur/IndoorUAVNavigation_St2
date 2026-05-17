#define _CRT_SECURE_NO_WARNINGS

#include <algorithm>
#include <chrono>
#include <iostream>
#include <unordered_map>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>
#include <pcl/PolygonMesh.h>

using PCL_Point = pcl::PointXYZ;
using PCL_Cloud = pcl::PointCloud<PCL_Point>;
using Clock = std::chrono::steady_clock;
using Ms = std::chrono::milliseconds;

static void printElapsed(const char* label, Clock::time_point t0)
{
    auto ms = std::chrono::duration_cast<Ms>(Clock::now() - t0).count();
    std::cout << "[" << label << "] " << ms / 1000.0 << " sn" << std::endl;
}

// ----------------------------
// Mesh acik mi kapali mi kontrol
// ----------------------------
static void checkMeshClosed(const pcl::PolygonMesh& mesh)
{
    std::unordered_map<int, int> edge_count;

    for (const auto& poly : mesh.polygons)
    {
        for (size_t i = 0; i < poly.vertices.size(); ++i)
        {
            int v1 = poly.vertices[i];
            int v2 = poly.vertices[(i + 1) % poly.vertices.size()];
            if (v1 > v2) std::swap(v1, v2);
            edge_count[v1 * 100000 + v2]++;
        }
    }

    int boundary_edges = 0;
    for (const auto& e : edge_count)
        if (e.second == 1)
            boundary_edges++;

    std::cout << "Boundary edges: " << boundary_edges << std::endl;
    std::cout << (boundary_edges == 0 ? "Mesh CLOSED" : "Mesh OPEN") << std::endl;
}

// ----------------------------
// Ana isleme fonksiyonu
// ----------------------------
static bool processMesh(const std::string& input_pcd, const std::string& output_ply)
{
    std::cout << "\n========================================" << std::endl;
    std::cout << "Girdi : " << input_pcd << std::endl;
    std::cout << "Cikti : " << output_ply << std::endl;
    std::cout << "========================================" << std::endl;

    auto t_total = Clock::now();

    // --- Yukle ---
    auto t0 = Clock::now();
    PCL_Cloud::Ptr cloud(new PCL_Cloud);

    if (pcl::io::loadPCDFile<PCL_Point>(input_pcd, *cloud) == -1)
    {
        std::cerr << "[HATA] PCD yuklenemedi: " << input_pcd << std::endl;
        return false;
    }
    std::cout << "Yuklendi: " << cloud->size() << " nokta" << std::endl;
    printElapsed("Yukleme", t0);

    // --- Normal Hesaplama (OpenMP) ---
    // KSearch: 10 | Threads: 0 (otomatik)
    t0 = Clock::now();
    pcl::NormalEstimationOMP<PCL_Point, pcl::Normal> ne;
    pcl::search::KdTree<PCL_Point>::Ptr tree(new pcl::search::KdTree<PCL_Point>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setKSearch(25);
    ne.setNumberOfThreads(0);
    ne.compute(*normals);

    std::cout << "Normal hesaplama tamamlandi." << std::endl;
    printElapsed("Normal hesaplama", t0);

    // --- Nokta + Normal Birlestir ---
    t0 = Clock::now();
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_n(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_n);

    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_n);

    // --- GP3 Mesh ---
    // SearchRadius: 0.4 | Mu: 2.5 | MaxNN: 50 | NormalConsistency: false
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh mesh;

    gp3.setSearchRadius(0.8);
    gp3.setMu(3);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 3);
    gp3.setMinimumAngle(M_PI / 18);
    gp3.setMaximumAngle(2 * M_PI / 3);
    gp3.setNormalConsistency(true);
    gp3.setInputCloud(cloud_n);
    gp3.setSearchMethod(tree2);

    gp3.reconstruct(mesh);

    std::cout << "Triangles: " << mesh.polygons.size() << std::endl;
    printElapsed("GP3 mesh", t0);

    // --- Mesh Kontrol ---
    checkMeshClosed(mesh);

    // --- Kaydet ---
    t0 = Clock::now();
    pcl::io::savePLYFile(output_ply, mesh);
    std::cout << "Kaydedildi: " << output_ply << std::endl;
    printElapsed("Kaydetme", t0);

    printElapsed("TOPLAM", t_total);

    return true;
}

int main()
{
    const std::string base_path = "C:/location2_sensorrange_005_7/";

    processMesh(
        base_path + "Location2_new_filtered.pcd",
        base_path + "Location2_new2_filtered_mesh.ply"
    );

    processMesh(
        base_path + "Location2_new_filtered_v2.pcd",
        base_path + "Location2_new2_filtered_v2_mesh.ply"
    );

    std::cout << "\nTum islemler tamamlandi." << std::endl;
    std::cin.get();
    return 0;
}