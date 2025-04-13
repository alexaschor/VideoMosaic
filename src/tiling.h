#ifndef TILING_H
#define TILING_H

#include <filesystem>
#include <limits>
#include <opencv2/opencv.hpp>
#include <tuple>
#include <fstream>

#include "field.h"

using namespace std;
using namespace cv;

typedef VEC3B Color;
typedef map<tuple<int, int, int>, Color> VideoMap;

class VideoTile {
public:
    VideoMap grid;
    VEC3F min;
    VEC3F max;

    VideoTile() {
        float fMax = numeric_limits<float>().max();
        float fMin = numeric_limits<float>().lowest();

        this->min = VEC3F(fMax, fMax, fMax);
        this->max = VEC3F(fMin, fMin, fMin);
    }

    static tuple<int, int, int> v3ToTuple(VEC3I v) {
        return make_tuple(v[0], v[1], v[2]);
    }

    bool hasPixel(VEC3I coords) {
        return grid.find(v3ToTuple(coords)) != grid.end();
    }

    Color getPixel(VEC3I coords) {
        return grid[v3ToTuple(coords)];
    }

    void setPixel(VEC3I coords, Color col) {
        grid[v3ToTuple(coords)] = col;

        for (int c = 0; c < 3; c++) {
            if (coords[c] > max[c]) {
                max[c] = coords[c];
            }

            if (coords[c] < min[c]) {
                min[c] = coords[c];
            }
        }
    }

    VideoTile(const vector<Mat>& images): VideoTile() {
        for (int n = 0; n < images.size(); ++n) {
            const Mat& img = images[n];
            for (int y = 0; y < img.rows; ++y) {
                for (int x = 0; x < img.cols; ++x) {
                    VEC3B pixel = img.at<VEC3B>(y, x);
                    if (pixel != VEC3B(0, 0, 0)) {
                        setPixel(VEC3I(x, y, n), pixel);
                    }
                }
            }
        }
    }
    void writeToVTK(const string& filename) {
        ofstream file(filename);
        if (!file.is_open()) {
            cerr << "Error: Unable to open file for writing." << endl;
            return;
        }

        // Write VTK header
        file << "# vtk DataFile Version 3.0\n";
        file << "VideoTile data\n";
        file << "ASCII\n";
        file << "DATASET STRUCTURED_POINTS\n";

        // Write dimensions
        int dims[3] = {
            static_cast<int>(max[0] - min[0] + 1),
            static_cast<int>(max[1] - min[1] + 1),
            static_cast<int>(max[2] - min[2] + 1)
        };
        file << "DIMENSIONS " << dims[0] << " " << dims[1] << " " << dims[2] << "\n";

        // Write origin
        file << "ORIGIN " << min[0] << " " << min[1] << " " << min[2] << "\n";

        // Write spacing (assume unit spacing)
        file << "SPACING 1 1 1\n";

        // Write point data
        file << "POINT_DATA " << (dims[0] * dims[1] * dims[2]) << "\n";
        file << "COLOR_SCALARS color 3\n";

        for (int z = min[2]; z <= max[2]; ++z) {
            for (int y = min[1]; y <= max[1]; ++y) {
                for (int x = min[0]; x <= max[0]; ++x) {
                    VEC3I coords(x, y, z);
                    Color color = getPixel(coords);
                    file << static_cast<float>(color[0]) / 255.0 << " "
                        << static_cast<float>(color[1]) / 255.0 << " "
                        << static_cast<float>(color[2]) / 255.0 << "\n";
                }
            }
        }

        file.close();
    }

    VideoTile(const string& filename): VideoTile() {

        ifstream file(filename);
        if (!file.is_open()) {
            cerr << "Error: Unable to open file for reading." << endl;
        }

        string line;
        int dims[3];
        VEC3F origin;

        // Read header and metadata
        while (getline(file, line)) {
            if (line.find("DIMENSIONS") != string::npos) {
                sscanf(line.c_str(), "DIMENSIONS %d %d %d", &dims[0], &dims[1], &dims[2]);
            } else if (line.find("ORIGIN") != string::npos) {
                sscanf(line.c_str(), "ORIGIN %f %f %f", &origin[0], &origin[1], &origin[2]);
            } else if (line.find("COLOR_SCALARS") != string::npos) {
                break;
            }
        }

        cout << dims[0] << " " << dims[1] << " " << dims[2] << endl;
        cout << origin << endl;

        // Read color data
        for (int z = 0; z < dims[2]; ++z) {
            for (int y = 0; y < dims[1]; ++y) {
                for (int x = 0; x < dims[0]; ++x) {
                    float r, g, b;
                    file >> r >> g >> b;
                    VEC3I coords(x + origin[0], y + origin[1], z + origin[2]);
                    Color color(
                        static_cast<uchar>(r * 255),
                        static_cast<uchar>(g * 255),
                        static_cast<uchar>(b * 255)
                        );

                    setPixel(coords, color);
                }
            }
        }

        cout << "MIN: "<< min << endl;
        cout << "MAX: "<< max << endl;

        file.close();
    }

    void writeToPNGSequence(const string& directory) {
        // Create the directory if it doesn't exist
        filesystem::create_directories(directory);

        for (int z = min[2]; z <= max[2]; ++z) {
            // Create a new image for this z-slice
            Mat image(max[1] - min[1] + 1, max[0] - min[0] + 1, CV_8UC4, Scalar(0, 0, 0, 0));

            for (int y = min[1]; y <= max[1]; ++y) {
                for (int x = min[0]; x <= max[0]; ++x) {
                    VEC3I coords(x, y, z);
                    if (hasPixel(coords)) {
                        Color color = getPixel(coords);
                        image.at<VEC4B>(y - min[1], x - min[0]) = VEC4B(color[0], color[1], color[2], 255);
                    }
                }
            }

            // Save the image
            string filename = directory + "/slice_" + to_string(z - min[2]) + ".png";
            imwrite(filename, image);
        }
    }
};

class VideoTileOccupancyField: public FieldFunction3D {
public:
    VideoTile* tile;

    VideoTileOccupancyField(VideoTile* tile): tile(tile) {}

    virtual Real getFieldValue(const VEC3F& pos) const override {
        VEC3F p = tile->min + (pos.cwiseProduct(tile->max - tile->min));
        VEC3I pi(p[0], p[1], p[2]);

        return (tile->hasPixel(pi)) ? -1 : 1;
    }

};


#endif

