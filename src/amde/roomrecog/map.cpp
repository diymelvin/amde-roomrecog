// This library was created as a part of Autonomous Mapping of Dynamic Environments
// project work in the Intelligent Robotics laboratory, Aalto University
// School of Electrical Engineering, in the spring of 2017.
//
// Copyright (c) 2017 Samuli Peurasaari
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
//   * The above copyright notice and this permission notice shall be included in
//     all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <cmath>

#include "../../../include/amde/roomrecog/map.h"

namespace amde {
    namespace roomrecog {

        Map::Map(const occupancy_type* original, unsigned int width, unsigned int height, unsigned int scale,
                 float originalResolution, Rectangle subRect)
                : width(subRect.width / scale), height(subRect.height / scale), resolution(originalResolution * scale) {

            unsigned int w = this->width;
            unsigned int h = this->height;
            map = new occupancy_type[w * h];
            for (unsigned int y = 0; y < h; y++) {
                unsigned int mapY = y * scale + subRect.y;
                for (unsigned int x = 0; x < w; x++) {
                    unsigned int mapX = x * scale + subRect.x;
                    map[y * w + x] = average(original, width, height, mapX, mapX + scale, mapY, mapY + scale);
                }
            }
        }

        Map::~Map() {
            delete[] map;
        }

        occupancy_type Map::average(const occupancy_type* map, unsigned int mapWidth, unsigned int mapHeight, unsigned int x1, unsigned int x2,
                          unsigned int y1, unsigned int y2) const {
            int unknown = 0;
            int occupied = 0;
            int free = 0;
            for (int y = y1; y < y2 && y < mapHeight; y++) {
                for (int x = x1; x < x2 && x < mapWidth; x++) {
                    int mapPx = map[y * mapWidth + x];
                    if (mapPx < 0) {
                        unknown++;
                    } else if (mapPx < AMDE_MAP_OCCUPIED) {
                        free++;
                    } else {
                        occupied++;
                    }
                }
            }
            int n = unknown + occupied + free;
            if (unknown <= n / 2) {
                if (occupied > round(n * 0.1f)) return 100;
                else return 0;
            } else {
                if (occupied > 1) {
                    return 100;
                } else if (free > 0) {
                    return 0;
                } else return -1;
            }
        }
    }
}