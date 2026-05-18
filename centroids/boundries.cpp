#define _CRT_SECURE_NO_WARNINGS

#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>

#include <pcl/io/ply_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>

// ----------------------------
// Edge hash (carpismadan muaf)
// ----------------------------
struct PairHash {
    size_t operator()(const std::pair<int, int>& p) const {
        size_t a = static_cast<size_t>(p.first);
        size_t b = static_cast<size_t>(p.second);
        return (a + b) * (a + b + 1) / 2 + b;
    }
};

// ----------------------------
// Boundary tespiti
// ----------------------------
struct BoundaryData
{
    std::vector<std::pair<int, int>> edges;   // boundary edge ciftleri (v1, v2)
    std::vector<int>                vertices; // tekil boundary vertex indeksleri
};

static BoundaryData extractBoundary(const pcl::PolygonMesh& mesh)
{
    std::unordered_map<std::pair<int, int>, int, PairHash> edge_count;

    for (const auto& poly : mesh.polygons)
    {
        for (size_t i = 0; i < poly.vertices.size(); ++i)
        {
            int v1 = static_cast<int>(poly.vertices[i]);
            int v2 = static_cast<int>(poly.vertices[(i + 1) % poly.vertices.size()]);
            if (v1 > v2) std::swap(v1, v2);
            edge_count[{v1, v2}]++;
        }
    }

    BoundaryData result;
    std::unordered_set<int> vert_set;

    for (const auto& e : edge_count)
    {
        if (e.second == 1) // sadece 1 ucgene ait => boundary
        {
            result.edges.push_back(e.first);
            vert_set.insert(e.first.first);
            vert_set.insert(e.first.second);
        }
    }

    result.vertices.assign(vert_set.begin(), vert_set.end());
    std::sort(result.vertices.begin(), result.vertices.end());

    return result;
}

// ----------------------------
// Boundary vertex'leri PLY olarak kaydet
// (nokta bulutu formatinda, renklendirilebilir)
// ----------------------------
static void saveBoundaryVerticesPLY(
    const pcl::PolygonMesh& mesh,
    const BoundaryData& bd,
    const std::string& output_path)
{
    // Mesh'ten tum noktalari al
    pcl::PointCloud<pcl::PointXYZ> all_points;
    pcl::fromPCLPointCloud2(mesh.cloud, all_points);

    std::ofstream ofs(output_path);
    if (!ofs.is_open()) {
        std::cerr << "[HATA] Dosya acilamadi: " << output_path << std::endl;
        return;
    }

    // PLY header
    ofs << "ply\n";
    ofs << "format ascii 1.0\n";
    ofs << "comment Boundary vertices\n";
    ofs << "element vertex " << bd.vertices.size() << "\n";
    ofs << "property float x\n";
    ofs << "property float y\n";
    ofs << "property float z\n";
    ofs << "property uchar red\n";
    ofs << "property uchar green\n";
    ofs << "property uchar blue\n";
    ofs << "end_header\n";

    // Boundary noktalar kirmizi renkte
    for (int idx : bd.vertices)
    {
        const auto& p = all_points[idx];
        ofs << p.x << " " << p.y << " " << p.z
            << " 255 0 0\n";  // RGB: kirmizi
    }

    ofs.close();
    std::cout << "Boundary vertices kaydedildi: " << output_path
        << " (" << bd.vertices.size() << " nokta)" << std::endl;
}

// ----------------------------
// Boundary edge'leri ayrı CSV olarak kaydet
// (v1_x, v1_y, v1_z, v2_x, v2_y, v2_z)
// ----------------------------
static void saveBoundaryEdgesCSV(
    const pcl::PolygonMesh& mesh,
    const BoundaryData& bd,
    const std::string& output_path)
{
    pcl::PointCloud<pcl::PointXYZ> all_points;
    pcl::fromPCLPointCloud2(mesh.cloud, all_points);

    std::ofstream ofs(output_path);
    if (!ofs.is_open()) {
        std::cerr << "[HATA] Dosya acilamadi: " << output_path << std::endl;
        return;
    }

    ofs << "v1_x,v1_y,v1_z,v2_x,v2_y,v2_z,v1_idx,v2_idx\n";

    for (const auto& e : bd.edges)
    {
        const auto& p1 = all_points[e.first];
        const auto& p2 = all_points[e.second];
        ofs << p1.x << "," << p1.y << "," << p1.z << ","
            << p2.x << "," << p2.y << "," << p2.z << ","
            << e.first << "," << e.second << "\n";
    }

    ofs.close();
    std::cout << "Boundary edges kaydedildi: " << output_path
        << " (" << bd.edges.size() << " kenar)" << std::endl;
}

// ----------------------------
// Ana fonksiyon
// ----------------------------
static void processBoundary(const std::string& input_ply, const std::string& out_prefix)
{
    std::cout << "\n========================================" << std::endl;
    std::cout << "Mesh: " << input_ply << std::endl;

    pcl::PolygonMesh mesh;
    if (pcl::io::loadPLYFile(input_ply, mesh) == -1)
    {
        std::cerr << "[HATA] PLY yuklenemedi: " << input_ply << std::endl;
        return;
    }
    std::cout << "Yuklendi: " << mesh.polygons.size() << " ucgen" << std::endl;

    BoundaryData bd = extractBoundary(mesh);
    std::cout << "Boundary edges  : " << bd.edges.size() << std::endl;
    std::cout << "Boundary vertices: " << bd.vertices.size() << std::endl;

    // 1) Boundary noktalari PLY (gorsellestirilmek icin MeshLab/CloudCompare'de ac)
    saveBoundaryVerticesPLY(mesh, bd, out_prefix + "_boundary_vertices.ply");

    // 2) Boundary edge listesi CSV
    saveBoundaryEdgesCSV(mesh, bd, out_prefix + "_boundary_edges.csv");

    std::cout << "========================================" << std::endl;
}

int main()
{
    const std::string base_path = "C:/location2_sensorrange_005_7/";

    processBoundary(
        base_path + "Location2_new2_filtered_mesh.ply",
        base_path + "Location2_new2_filtered_mesh"
    );

    processBoundary(
        base_path + "Location2_new2_filtered_v2_mesh.ply",
        base_path + "Location2_new2_filtered_v2_mesh"
    );

    std::cout << "\nTum islemler tamamlandi." << std::endl;
    std::cin.get();
    return 0;
}