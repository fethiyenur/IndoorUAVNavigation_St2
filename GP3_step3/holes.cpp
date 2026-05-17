#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <unordered_map>
#include <vector>
#include <set>
#include <fstream>
#include <iostream>

using PointT = pcl::PointXYZ;

// -------------------------
std::pair<int, int> makeEdge(int a, int b)
{
    return (a < b) ? std::make_pair(a, b) : std::make_pair(b, a);
}

// -------------------------
std::vector<Eigen::Vector3f> getPoints(const pcl::PolygonMesh& mesh)
{
    pcl::PointCloud<PointT> cloud;
    pcl::fromPCLPointCloud2(mesh.cloud, cloud);

    std::vector<Eigen::Vector3f> pts;
    for (auto& p : cloud.points)
        pts.emplace_back(p.x, p.y, p.z);

    return pts;
}

// -------------------------
std::set<std::pair<int, int>> findBoundaryEdges(const pcl::PolygonMesh& mesh)
{
    std::unordered_map<long long, int> edgeCount;

    for (const auto& poly : mesh.polygons)
    {
        for (size_t i = 0; i < poly.vertices.size(); ++i)
        {
            int a = poly.vertices[i];
            int b = poly.vertices[(i + 1) % poly.vertices.size()];

            auto e = makeEdge(a, b);
            long long key = ((long long)e.first << 32) | e.second;

            edgeCount[key]++;
        }
    }

    std::set<std::pair<int, int>> boundaryEdges;

    for (auto& [key, count] : edgeCount)
    {
        if (count == 1)
        {
            int a = key >> 32;
            int b = key & 0xffffffff;
            boundaryEdges.insert({ a,b });
        }
    }

    return boundaryEdges;
}

// -------------------------
std::vector<std::vector<int>> buildLoops(std::set<std::pair<int, int>> edges)
{
    std::unordered_map<int, std::vector<int>> adj;

    for (auto& e : edges)
    {
        adj[e.first].push_back(e.second);
        adj[e.second].push_back(e.first);
    }

    std::set<int> visited;
    std::vector<std::vector<int>> loops;

    for (auto& [start, _] : adj)
    {
        if (visited.count(start)) continue;

        std::vector<int> loop;
        int current = start;
        int prev = -1;

        while (true)
        {
            loop.push_back(current);
            visited.insert(current);

            auto& neigh = adj[current];

            int next = -1;
            for (int n : neigh)
                if (n != prev) { next = n; break; }

            if (next == -1 || visited.count(next)) break;

            prev = current;
            current = next;

            if (current == start) break;
        }

        if (loop.size() > 2)
            loops.push_back(loop);
    }

    return loops;
}

// -------------------------
Eigen::Vector3f computeLoopCentroid(
    const std::vector<int>& loop,
    const std::vector<Eigen::Vector3f>& pts)
{
    Eigen::Vector3f c(0, 0, 0);

    for (int i : loop)
        c += pts[i];

    return c / loop.size();
}

// -------------------------
float distance(const Eigen::Vector3f& a, const Eigen::Vector3f& b)
{
    return (a - b).norm();
}

bool isPassableHole(
    const std::vector<int>& loop,
    const std::vector<Eigen::Vector3f>& pts,
    const Eigen::Vector3f& centroid,
    float minRadius)
{
    float minDist = 1e9;

    for (int idx : loop)
    {
        float d = distance(pts[idx], centroid);
        if (d < minDist)
            minDist = d;
    }

    return minDist >= minRadius;
}

// -------------------------
void writeCSV(const std::vector<Eigen::Vector3f>& points)
{
    std::ofstream out("drone_holes.csv");
    out << "id,x,y,z\n";

    for (size_t i = 0; i < points.size(); ++i)
        out << i << ","
        << points[i].x() << ","
        << points[i].y() << ","
        << points[i].z() << "\n";
}

// -------------------------
// ✅ YENİ EKLENEN: PLY EXPORT
// -------------------------
void writePLY(const std::vector<Eigen::Vector3f>& pts)
{
    std::ofstream out("holes.ply");

    out << "ply\nformat ascii 1.0\n";
    out << "element vertex " << pts.size() << "\n";
    out << "property float x\nproperty float y\nproperty float z\n";
    out << "end_header\n";

    for (auto& p : pts)
        out << p.x() << " " << p.y() << " " << p.z() << "\n";
}

// -------------------------
int main()
{
    pcl::PolygonMesh mesh;

    if (pcl::io::loadPLYFile("C:\\location2_sensorrange_005_7\\Location2_new_filtered_v2_mesh.ply", mesh) == -1)
    {
        std::cout << "Mesh yüklenemedi\n";
        return -1;
    }

    auto pts = getPoints(mesh);

    auto boundaryEdges = findBoundaryEdges(mesh);
    auto loops = buildLoops(boundaryEdges);

    std::vector<Eigen::Vector3f> droneHoles;

    float DRONE_RADIUS = 0.6f;

    for (auto& loop : loops)
    {
        Eigen::Vector3f c = computeLoopCentroid(loop, pts);

        if (isPassableHole(loop, pts, c, DRONE_RADIUS))
            droneHoles.push_back(c);
    }

    writeCSV(droneHoles);

    // 🔥 BURASI YENİ
    writePLY(droneHoles);

    std::cout << "Drone hole: " << droneHoles.size() << std::endl;

    return 0;
}