#ifndef PACKING_H
#define PACKING_H

#include <random>
#include <vector>
#include <string>

#include "mesh.h"

class MeshPacker {
public:
    Mesh target_mesh;
    std::vector<Mesh> tile_meshes;
    std::vector<Mesh> packed_tiles;
    Real min_size_ratio;

    MeshPacker(const std::string& target_obj, const std::vector<std::string>& tile_objs, Real min_size_ratio)
        : min_size_ratio(min_size_ratio) {
            target_mesh = Mesh(target_obj);

            for (const auto& tile_obj : tile_objs) {
                Mesh m(tile_obj);
                tile_meshes.push_back(m);
            }
        }

    void pack() {
        Real targetVolume = target_mesh.meshVolume();
        int  numTries = 0;

        PRINT("Begin packing mesh");

        while (true) {
            auto position = find_random_position();
            auto tile = spawn_random_tile(position);

            grow_tile(tile);

            Real tileVolume = tile.meshVolume();

            if ( tileVolume / targetVolume >= min_size_ratio) {
                packed_tiles.push_back(tile);
                numTries = 0;
            } else if ( numTries <= 100 ) {
                // If the tile is too small, discard it and try again
                numTries += 1;
                continue;
            } else {
                Real totalVolume = calculate_total_packed_volume();
                PRINTFn("Terminated early. Tile #%zu: %f volume ratio", packed_tiles.size(), totalVolume / targetVolume);
                break;
            }


            // Check if we've filled the target mesh sufficiently
            Real totalVolume = calculate_total_packed_volume();
            PRINTFn("Spawned tile #%zu: %f volume ratio", packed_tiles.size(), totalVolume / targetVolume);
            if (totalVolume / targetVolume > 0.40) {
                break;
            }
        }
    }

    Mesh spawn_random_tile(VEC3F position) {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, tile_meshes.size() - 1);

        Mesh tile = tile_meshes[dis(gen)];
        tile.setCentroid(position);
        tile.scaleMesh(0.0001);

        return tile;
    }

    void grow_tile(Mesh& tile) {
        Real scale = 1;
        int  iters = 0;
        const Real growth_rate = 0.1;
        while (!check_collision(tile)) {
            tile.scaleMesh(1.0 + growth_rate);
            scale *= (1.0 + growth_rate);
            iters++;
        }
        // PRINTFn(" -> Collided! (scale: %f, iters: %d, volume ratio: %f)", scale, iters, tile.meshVolume() / target_mesh.meshVolume());
        tile.scaleMesh(1.0 / (1.0 + growth_rate)); // Revert the last growth
    }

    bool check_collision(const Mesh& tile) {
        if (tile.intersects(target_mesh)) {
            return true;
        }

        for (const auto& packed_tile : packed_tiles) {
            if (tile.intersects(packed_tile)) {
                return true;
            }
        }

        return false;
    }

    bool isValidStartingPos(const VEC3F& pos) {
        if (!target_mesh.contains(pos)) {
            return false;
        }

        for (const auto& packed_tile : packed_tiles) {
            if (packed_tile.contains(pos)) {
                return false;
            }
        }

        return true;
    }

    VEC3F find_random_position() {
        auto bbox = target_mesh.bbox();
        VEC3F point;

        do {
            point = bbox.randomPointInside();
        } while (!isValidStartingPos(point));

        return point;
    }

    Real calculate_total_packed_volume() {
        Real total_volume = 0.0;
        for (const auto& tile : packed_tiles) {
            total_volume += tile.meshVolume();
        }
        return total_volume;
    }

};


#endif


