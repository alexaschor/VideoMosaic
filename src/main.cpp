#include "packing.h"
#include "MC.h"
#include <format>

using namespace std;

// int xx_main(int argc, char* argv[]) {
//     vector<Mat> images;
//     string imagePath;
//
//     string outputPath;
//
//     // Load images
//     if (argc > 2) {
//
//         outputPath = argv[1];
//
//         cout << "Reading images..." << endl;
//         for (int i = 2; i < argc; ++i) {
//             Mat img = imread(argv[i]);
//             if (img.empty()) {
//                 cout << "Error loading image: " << argv[i] << endl;
//                 continue;
//             }
//             images.push_back(img);
//         }
//     } else {
//         cout << "Arrg! I need args!" << endl;
//         return 1;
//     }
//
//     if (images.empty()) {
//         cout << "No images loaded. Exiting." << endl;
//         return 1;
//     }
//
//     cout << "Generating tile..." << endl;
//     // Convert images to sparse matrix
//     VideoTile tile(images);
//     VideoTileOccupancyField occ(&tile);
//
//     VEC3F span = tile.max - tile.min;
//     span[2] = 0;
//     float scale = 200.0 / span.norm();
//     VEC3F dims = span * scale;
//     dims[2] = (tile.max - tile.min)[2];
//
//     VirtualGrid3D vg(dims[0], dims[1], dims[2], VEC3F(0, 0, 0), VEC3F(1, 1, 1), &occ);
//
//     Mesh m;
//     MC::march_cubes(&vg, m);
//
//     auto mesh = m.toCGALMesh();
//     CGAL::Polygon_mesh_processing::reverse_face_orientations(mesh);
//
//     CGAL_Mesh hull;
//     CGAL::convex_hull_3(mesh.points().begin(), mesh.points().end(), hull);
//
//     CGAL::IO::write_OBJ(outputPath + "_orig.obj", mesh);
//     CGAL::IO::write_OBJ(outputPath + "_wrap.obj", hull);
//
//     cout << "Wrote " << outputPath << "_(orig, wrap).obj" << endl;
//
//     return 0;
// }

int main(int argc, char* argv[]) {

    // Mesh m1("data/ball.obj");
    // Mesh m2("data/ball.obj");
    //
    // PRINTI(m1.intersects(m2)?1:0);
    //
    // m2.scaleMesh(0.1);
    //
    // m2.writeOBJ("processed.obj");
    //
    // PRINTI(m1.intersects(m2)?1:0);

    vector<string> tiles;
    tiles.push_back("hulls/0.obj");
    tiles.push_back("hulls/1.obj");
    tiles.push_back("hulls/2.obj");
    tiles.push_back("hulls/3.obj");
    tiles.push_back("hulls/4.obj");
    tiles.push_back("hulls/5.obj");
    tiles.push_back("hulls/6.obj");
    tiles.push_back("hulls/7.obj");
    tiles.push_back("hulls/8.obj");

    MeshPacker mp("data/ball.obj", tiles, 0.001);
    cout << "Created..." << endl;
    mp.pack();

    for (int i = 0; i < mp.packed_tiles.size(); ++i) {
        Mesh m = mp.packed_tiles[i];
        auto filename = format("packed_tile_{}.obj", i);
        m.writeOBJ(filename);
    }



    return 0;
}
