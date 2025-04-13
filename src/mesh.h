#ifndef MESH_H
#define MESH_H
#include <fstream>
#include <iostream>
#include <cstdio>
#include <string>
#include <vector>

#include "SETTINGS.h"
#include "field.h"
#include "igl/copyleft/cgal/RemeshSelfIntersectionsParam.h"

#include <igl/point_mesh_squared_distance.h>
#include <igl/winding_number.h>
#include <igl/volume.h>
#include <igl/copyleft/cgal/intersect_other.h>

using namespace std;

class Mesh {
public:
    std::vector<VEC3F> vertices;
    std::vector<VEC3F> normals;
    std::vector<uint> indices;

    Mesh(string filename) {
        readOBJ(filename);
    }

    Mesh() {}

    int numFaces() {
        assert(indices.size() % 3 == 0);
        return indices.size() / 3;
    }

    int num_vertices() const {
        return vertices.size();
    }

    int num_faces() const {
        return indices.size() / 3;
    }

    int num_edges() const {
        // For a manifold mesh, we can use Euler's formula:
        // V - E + F = 2 (where V = vertices, E = edges, F = faces)
        // Rearranging: E = V + F - 2
        return num_vertices() + num_faces() - 2;
    }

    // Convert Mesh to Eigen matrices for libigl functions
    std::pair<Eigen::MatrixXd, Eigen::MatrixXi> toIGLMesh() const {
        Eigen::MatrixXd V(vertices.size(), 3);
        Eigen::MatrixXi F(indices.size() / 3, 3);

        for (size_t i = 0; i < vertices.size(); ++i) {
            V.row(i) << vertices[i].x(), vertices[i].y(), vertices[i].z();
        }

        for (size_t i = 0; i < indices.size(); i += 3) {
            F.row(i / 3) << indices[i], indices[i + 1], indices[i + 2];
        }

        return std::make_pair(V, F);
    }

    std::vector<std::array<int, 3>> getTriangles() {
        std::vector<std::array<int, 3>> out;

        for (int i=0; i < indices.size(); i+=3) {
            std::array<int, 3> triangle{static_cast<int>(indices[i]), static_cast<int>(indices[i+1]), static_cast<int>(indices[i+2])};
            out.push_back(triangle);
        }

        return out;
    }

    //readOBJ function adapted from http://www.opengl-tutorial.org
    void readOBJ(std::string filename) {
        FILE * file = fopen(filename.c_str(), "r");
        if (file == NULL){
            printf("Could not open OBJ file %s for reading.\n", filename.c_str());
            exit(1);
        }
        while (true){
            char lineHeader[128];
            // read the first word of the line
            int res = fscanf(file, "%s", lineHeader);
            if (res == EOF)
                break;

            if (strcmp(lineHeader, "v") == 0) {
                VEC3F vertex;
                fscanf(file, "%lf %lf %lf\n", &vertex.x(), &vertex.y(), &vertex.z());
                vertices.push_back(vertex);
            } else if (strcmp(lineHeader, "f") == 0){
                size_t a_i, b_i, c_i;
                int matches = fscanf(file, "%zd %zd %zd\n", &a_i, &b_i, &c_i);
                if (matches != 3) {
                    printf("Encountered malformed face data when reading OBJ %s. Make sure that UV and vertex normals are not included in the file.\n", filename.c_str());
                    exit(1);
                }

                indices.push_back(a_i-1);
                indices.push_back(b_i-1);
                indices.push_back(c_i-1);
            }
        }

        printf("Read %lu vertices and %lu faces from %s\n", vertices.size(), indices.size() / 3, filename.c_str());
    }

    void writeOBJ(std::string filename) {
        std::ofstream out;
        out.open(filename);
        if (out.is_open() == false)
            return;
        out << "g " << "Obj" << std::endl;
        for (size_t i = 0; i < vertices.size(); i++)
            out << "v " << vertices.at(i).x() << " " << vertices.at(i).y() << " " << vertices.at(i).z() << '\n';
        for (size_t i = 0; i < normals.size(); i++)
            out << "vn " << normals.at(i).x() << " " << normals.at(i).y() << " " << normals.at(i).z() << '\n';
        for (size_t i = 0; i < indices.size(); i += 3) {
            out << "f " << indices.at(i) + 1 << "//" << indices.at(i) + 1
                << " " << indices.at(i + 1) + 1 << "//" << indices.at(i + 1) + 1
                << " " << indices.at(i + 2) + 1 << "//" << indices.at(i + 2) + 1
                << '\n';
        }
        out.close();

        std::cout << "Wrote " << vertices.size() << " vertices and " << indices.size() / 3 << " faces to " << filename << std::endl;
    }

    // Function to compute point-mesh distance
    Real distance(const VEC3F& point) const {

        auto [V, F] = toIGLMesh();

        Eigen::RowVector3d query_point(point.x(), point.y(), point.z());
        Eigen::VectorXd sqrD;
        Eigen::VectorXi I;
        Eigen::MatrixXd C;

        igl::point_mesh_squared_distance(query_point, V, F, sqrD, I, C);

        return std::sqrt(sqrD(0));
    }

    // Function to compute signed distance from a point to the mesh
    Real signedDistance(const VEC3F& point) const {
        Real unsigned_distance = distance(point);
        int insideFac = contains(point) ? -1 : 1;

        return unsigned_distance * insideFac;
    }

    bool contains(const VEC3F& point) const {
        // Check if the point is inside or outside the mesh
        auto [V, F] = toIGLMesh();

        Eigen::RowVector3d query_point(point.x(), point.y(), point.z());

        // Use winding number to determine if the point is inside or outside
        Real winding_number = igl::winding_number(V, F, query_point);

        return abs(winding_number) > 0.1;
    }

    // Function to compute mesh volume
    Real meshVolume() const {
        auto [V, F] = toIGLMesh();

        Eigen::MatrixXd V2(V.rows() + 1, V.cols());
        V2.topRows(V.rows()) = V;
        V2.bottomRows(1).setZero();

        Eigen::MatrixXi T(F.rows(), 4);
        T.leftCols(3) = F;
        T.rightCols(1).setConstant(V.rows());

        Eigen::VectorXd vol;
        igl::volume(V2, T, vol);

        return std::abs(vol.sum());

    }

    // Function to scale the mesh by a scalar factor around its centroid
    void scaleMesh(Real factor) {
        VEC3F centroid = getCentroid();

        // Translate to origin
        for (auto& vertex : vertices) {
            vertex -= centroid;
        }

        // Scale
        for (auto& vertex : vertices) {
            vertex *= factor;
        }

        // Translate back
        for (auto& vertex : vertices) {
            vertex += centroid;
        }
    }

    // Function to get the centroid of the mesh
    VEC3F getCentroid() const {
        VEC3F centroid(0.0, 0.0, 0.0);
        for (const auto& vertex : vertices) {
            centroid += vertex;
        }
        centroid /= static_cast<Real>(vertices.size());
        return centroid;
    }

    // Function to set the centroid of the mesh to a specific point
    void setCentroid(const VEC3F& newCentroid) {
        VEC3F currentCentroid = getCentroid();
        VEC3F translation = newCentroid - currentCentroid;

        for (auto& vertex : vertices) {
            vertex += translation;
        }
    }

    // Function to check if this mesh intersects with another mesh
    bool intersects(const Mesh& other) const {
        auto [V1, F1] = toIGLMesh();
        auto [V2, F2] = other.toIGLMesh();

        Eigen::MatrixXd IF;
        Eigen::MatrixXd VVAB;
        Eigen::MatrixXi FFAB;
        Eigen::VectorXi JAB;
        Eigen::VectorXi IMAB;

        // Perform boolean intersection
        auto param = igl::copyleft::cgal::RemeshSelfIntersectionsParam();
        param.first_only  = true;
        param.detect_only = true;

        igl::copyleft::cgal::intersect_other(V1, F1, V2, F2, param, IF, VVAB, FFAB, JAB, IMAB);

        // If the result has any faces, the meshes intersect
        return IF.rows() > 0;
    }

    AABB bbox() {
        AABB out = AABB::insideOut();
        for (VEC3F& v : vertices) {
            out.include(v);
        }
        return out;
    }

};

#endif

