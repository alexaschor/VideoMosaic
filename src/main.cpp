#include "packing.h"
#include "MC.h"
#include "tiling.h"
#include "glob.h"

#include <format>

using namespace std;

/*
int main(int argc, char* argv[]) {
    vector<Mat> images;
    string imagePath;

    string outputPath;

    // Load images
    if (argc > 2) {
        outputPath = argv[1];

        cout << "Reading images..." << endl;
        for (int i = 2; i < argc; ++i) {
            Mat img = imread(argv[i]);
            if (img.empty()) {
                cout << "Error loading image: " << argv[i] << endl;
                continue;
            }
            images.push_back(img);
        }
    } else {
        cout << "Arrg! I need args!" << endl;
        return 1;
    }

    if (images.empty()) {
        cout << "No images loaded. Exiting." << endl;
        return 1;
    }

    cout << "Generating tile..." << endl;
    // Convert images to sparse matrix
    VideoTile tile(images);
    VideoTileOccupancyField occ(&tile);

    VEC3F span = tile.max - tile.min;
    span[2] = 0;
    float scale = 200.0 / span.norm();
    VEC3F dims = span * scale;
    dims[2] = (tile.max - tile.min)[2];

    VirtualGrid3D vg(dims[0], dims[1], dims[2], VEC3F(0, 0, 0), VEC3F(1, 1, 1), &occ);

    MCMesh m;
    MC::march_cubes(&vg, m);

    m.writeOBJ(outputPath + "_orig.obj");
    m.convexHull().writeOBJ(outputPath + "_hull.obj");

    cout << "Wrote " << outputPath << "_(orig, hull).obj" << endl;

    return 0;
}
*/

int main(int argc, char* argv[]) {

    vector<string> tiles;

        for (auto& p : glob::glob("./hulls_consolidated/*.obj")) {
            tiles.push_back(p);
        }

    MeshPacker mp("./origs_processed/bear.obj", tiles, 0.0005);
    cout << "Created..." << endl;
    mp.pack();

    for (int i = 0; i < mp.packed_tiles.size(); ++i) {
        Mesh m = mp.packed_tiles[i];
        auto orig_fn = m.filename;
        auto bbox = m.bbox();

        string ident = orig_fn.substr(orig_fn.find_last_of("/") + 1);
        size_t pos   = ident.find("__");
        string group = ident.substr(0, pos);
        string color = ident.substr(pos + 2);
        color = color.substr(0, color.find("__"));

        printf("{'group': '%s', 'color' : '%s', 'minX': %f, 'minY', %f 'maxX': %f, 'maxY', %f, 'minF': %f, 'maxF': %f},",
            group.c_str(),
            color.c_str(),
            bbox.min()[0], bbox.min()[1], // Top left XY
            bbox.max()[0], bbox.max()[1], // Bottom right XY
            bbox.min()[2], bbox.max()[2]  // Frame range
            );

        auto out_fn  = std::format("packed_tile__{}__{}__{}.obj", i, group, color);
        m.writeOBJ(out_fn);
    }



    return 0;
}
