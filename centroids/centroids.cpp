#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/polygon_mesh_io.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <set>
#include <cmath>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3 Point;
typedef K::Vector_3 Vector;
typedef CGAL::Polyhedron_3<K> Mesh;

Vector normalize(const Vector& v)
{
    double len = std::sqrt(v.squared_length());
    if (len == 0.0) return Vector(0, 0, 0);
    return v / len;
}

Point compute_mesh_center(Mesh& mesh)
{
    double min_x = 1e9, min_y = 1e9, min_z = 1e9;
    double max_x = -1e9, max_y = -1e9, max_z = -1e9;

    for (auto v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v)
    {
        Point p = v->point();

        min_x = std::min(min_x, p.x());
        min_y = std::min(min_y, p.y());
        min_z = std::min(min_z, p.z());

        max_x = std::max(max_x, p.x());
        max_y = std::max(max_y, p.y());
        max_z = std::max(max_z, p.z());
    }

    return Point(
        (min_x + max_x) / 2.0,
        (min_y + max_y) / 2.0,
        (min_z + max_z) / 2.0
    );
}

std::vector<std::vector<Mesh::Halfedge_handle>> extract_boundary_loops(Mesh& mesh)
{
    std::vector<std::vector<Mesh::Halfedge_handle>> loops;
    std::set<Mesh::Halfedge_handle> visited;

    for (auto h = mesh.halfedges_begin(); h != mesh.halfedges_end(); ++h)
    {
        if (h->is_border() && visited.find(h) == visited.end())
        {
            std::vector<Mesh::Halfedge_handle> loop;
            Mesh::Halfedge_handle current = h;

            do {
                loop.push_back(current);
                visited.insert(current);
                current = current->next();
            } while (current != h && current->is_border());

            loops.push_back(loop);
        }
    }
    return loops;
}

Point compute_loop_centroid(const std::vector<Mesh::Halfedge_handle>& loop)
{
    std::set<Mesh::Vertex_handle> vertices;

    for (auto h : loop)
        vertices.insert(h->vertex());

    double x = 0, y = 0, z = 0;

    for (auto v : vertices)
    {
        Point p = v->point();
        x += p.x();
        y += p.y();
        z += p.z();
    }

    int n = vertices.size();

    return Point(x / n, y / n, z / n);
}

double compute_loop_radius(
    const std::vector<Mesh::Halfedge_handle>& loop,
    const Point& centroid)
{
    std::set<Mesh::Vertex_handle> vertices;

    for (auto h : loop)
        vertices.insert(h->vertex());

    double sum = 0;

    for (auto v : vertices)
    {
        Point p = v->point();
        sum += std::sqrt(CGAL::squared_distance(p, centroid));
    }

    return sum / vertices.size();
}

void write_centroids_ply(
    const std::vector<Point>& centroids,
    const std::vector<Vector>& directions,
    const std::vector<double>& radii,
    const std::string& filename)
{
    std::ofstream out(filename);

    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "element vertex " << centroids.size() << "\n";

    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";

    out << "property float nx\n";
    out << "property float ny\n";
    out << "property float nz\n";

    out << "property float radius\n";
    out << "property int label\n";

    out << "end_header\n";

    for (size_t i = 0; i < centroids.size(); ++i)
    {
        out << centroids[i].x() << " "
            << centroids[i].y() << " "
            << centroids[i].z() << " "

            << directions[i].x() << " "
            << directions[i].y() << " "
            << directions[i].z() << " "

            << radii[i] << " "
            << i + 1 << "\n";
    }
}

void write_centroids_csv(
    const std::vector<Point>& centroids,
    const std::vector<Vector>& directions,
    const std::vector<double>& radii,
    const std::string& filename)
{
    std::ofstream out(filename);

    out << "Label,X,Y,Z,DirX,DirY,DirZ,Radius\n";

    for (size_t i = 0; i < centroids.size(); ++i)
    {
        out << "s" << i + 1 << ","
            << centroids[i].x() << ","
            << centroids[i].y() << ","
            << centroids[i].z() << ","
            << directions[i].x() << ","
            << directions[i].y() << ","
            << directions[i].z() << ","
            << radii[i] << "\n";
    }
}

int main()
{
    std::string input_mesh = "inputMesh.stl";

    std::string ply_file = "boundaryCentroids.ply";
    std::string csv_file = "boundaryCentroids.csv";

    double MIN_RADIUS = 1.2;
    double SHIFT_DISTANCE = -0.5;

    Mesh mesh;

    if (!CGAL::IO::read_polygon_mesh(input_mesh, mesh))
    {
        std::cout << "Mesh okunamadı\n";
        return 0;
    }

    Point mesh_center = compute_mesh_center(mesh);

    auto loops = extract_boundary_loops(mesh);

    std::vector<Point> centroids;
    std::vector<Vector> directions;
    std::vector<double> radii;

    for (size_t i = 0; i < loops.size(); ++i)
    {
        Point centroid = compute_loop_centroid(loops[i]);

        double radius = compute_loop_radius(loops[i], centroid);

        if (radius >= MIN_RADIUS)
        {
            Vector dir = normalize(centroid - mesh_center);

            Point shifted = centroid + dir * SHIFT_DISTANCE;

            centroids.push_back(shifted);
            directions.push_back(dir);
            radii.push_back(radius);
        }
    }

    write_centroids_ply(centroids, directions, radii, ply_file);
    write_centroids_csv(centroids, directions, radii, csv_file);

    std::cout << "PLY ve CSV üretildi\n";
}