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
#include <igl/copyleft/cgal/convex_hull.h>

using namespace std;

class Mesh {
public:
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    string filename;

    Mesh(string filename): filename(filename) {
        readOBJ(filename);
    }

    Mesh() {}

    int numFaces() {
        return F.rows();
    }

    int num_vertices() const {
        return V.rows();
    }

    int num_faces() const {
        return F.rows();
    }

    std::vector<std::array<int, 3>> getTriangles() {
        std::vector<std::array<int, 3>> out;

        for (int i = 0; i < F.rows(); i++) {
            std::array<int, 3> triangle{F(i, 0), F(i, 1), F(i, 2)};
            out.push_back(triangle);
        }

        return out;
    }

    void readOBJ(std::string filename) {
        std::vector<VEC3F> vertices;
        std::vector<uint> indices;

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

        V.resize(vertices.size(), 3);
        F.resize(indices.size() / 3, 3);

        for (size_t i = 0; i < vertices.size(); ++i) {
            V.row(i) << vertices[i].x(), vertices[i].y(), vertices[i].z();
        }

        for (size_t i = 0; i < indices.size(); i += 3) {
            F.row(i / 3) << indices[i], indices[i + 1], indices[i + 2];
        }

        printf("Read %d vertices and %d faces from %s\n", num_vertices(), num_faces(), filename.c_str());
    }

    void writeOBJ(std::string filename) {
        std::ofstream out;
        out.open(filename);
        if (out.is_open() == false)
            return;
        out << "g " << "Obj" << std::endl;
        for (int i = 0; i < V.rows(); i++)
            out << "v " << V(i, 0) << " " << V(i, 1) << " " << V(i, 2) << '\n';
        for (int i = 0; i < F.rows(); i++) {
            out << "f " << F(i, 0) + 1 << "//" << F(i, 0) + 1
                << " " << F(i, 1) + 1 << "//" << F(i, 1) + 1
                << " " << F(i, 2) + 1 << "//" << F(i, 2) + 1
                << '\n';
        }
        out.close();

        std::cout << "Wrote " << num_vertices() << " vertices and " << num_faces() << " faces to " << filename << std::endl;
    }

    Real distance(const VEC3F& point) const {
        Eigen::RowVector3d query_point(point.x(), point.y(), point.z());
        Eigen::VectorXd sqrD;
        Eigen::VectorXi I;
        Eigen::MatrixXd C;

        igl::point_mesh_squared_distance(query_point, V, F, sqrD, I, C);

        return std::sqrt(sqrD(0));
    }

    Real signedDistance(const VEC3F& point) const {
        Real unsigned_distance = distance(point);
        int insideFac = contains(point) ? -1 : 1;

        return unsigned_distance * insideFac;
    }

    bool contains(const VEC3F& point) const {
        Eigen::RowVector3d query_point(point.x(), point.y(), point.z());

        Real winding_number = igl::winding_number(V, F, query_point);

        return abs(winding_number) > 0.1;
    }

    Mesh convexHull() const {
        Mesh out;
        igl::copyleft::cgal::convex_hull(V, out.V, out.F);
        return out;
    }

    Real meshVolume() const {
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

    void scaleMesh(Real factor) {
        VEC3F centroid = getCentroid();

        // Translate to origin
        V.rowwise() -= centroid.transpose();

        // Scale
        V *= factor;

        // Translate back
        V.rowwise() += centroid.transpose();
    }

    void scaleMeshXY(Real factor) {
        VEC3F centroid = getCentroid();

        // Translate to origin
        V.rowwise() -= centroid.transpose();

        // Scale only X and Y
        V.col(0) *= factor;
        V.col(1) *= factor;

        // Translate back
        V.rowwise() += centroid.transpose();
    }

    void scaleMeshZ(Real factor) {
        VEC3F centroid = getCentroid();

        // Translate to origin
        V.rowwise() -= centroid.transpose();

        // Scale only Z
        V.col(2) *= factor;

        // Translate back
        V.rowwise() += centroid.transpose();
    }

    VEC3F getCentroid() const {
        return VEC3F(V.col(0).mean(), V.col(1).mean(), V.col(2).mean());
    }

    void setCentroid(const VEC3F& newCentroid) {
        VEC3F currentCentroid = getCentroid();
        VEC3F translation = newCentroid - currentCentroid;

        V.rowwise() += translation.transpose();
    }

    bool intersects(const Mesh& other) const {
        Eigen::MatrixXd IF;
        Eigen::MatrixXd VVAB;
        Eigen::MatrixXi FFAB;
        Eigen::VectorXi JAB;
        Eigen::VectorXi IMAB;

        auto param = igl::copyleft::cgal::RemeshSelfIntersectionsParam();
        param.first_only  = true;
        param.detect_only = true;

        igl::copyleft::cgal::intersect_other(V, F, other.V, other.F, param, IF, VVAB, FFAB, JAB, IMAB);

        return IF.rows() > 0;
    }

    AABB bbox() {
        AABB out = AABB::insideOut();
        for (int i = 0; i < V.rows(); i++) {
            out.include(VEC3F(V(i, 0), V(i, 1), V(i, 2)));
        }
        return out;
    }
};

class MCMesh: public Mesh {
public:
    std::vector<VEC3F> vertices;
    std::vector<VEC3F> normals;
    std::vector<uint> indices;

    void finalize() {
        V.resize(vertices.size(), 3);
        F.resize(indices.size() / 3, 3);

        for (size_t i = 0; i < vertices.size(); ++i) {
            V.row(i) << vertices[i].x(), vertices[i].y(), vertices[i].z();
        }

        for (size_t i = 0; i < indices.size(); i += 3) {
            F.row(i / 3) << indices[i], indices[i + 1], indices[i + 2];
        }

        // Clear temporary vectors to save memory
        vertices.clear();
        normals.clear();
        indices.clear();
    }
};

#endif
