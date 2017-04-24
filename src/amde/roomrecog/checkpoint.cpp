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

#include "checkpoint.h"

namespace amde {
    namespace roomrecog {

        CheckPoint::CheckPoint() : MapPoint() {

        }

        CheckPoint::CheckPoint(unsigned int x, unsigned int y, unsigned int width, unsigned int height, room_type roomIndex,
                               unsigned int brushSize)
                : MapPoint(x, y, width, height, roomIndex), brushSize(brushSize) {

        }

        void CheckPoint::visit(room_type* checked, room_type* rooms, const occupancy_type* occupancy, const BrushPalette& brushPalette,
                               std::priority_queue<CheckPoint>& queue) {
            bool valid = brushPalette.getBrush(brushSize).isValid(x, y, width, occupancy);
            if (valid) {
                // Check if a bigger brush would also be valid
                while (brushSize < brushPalette.getMaxBrushSize()) {
                    if (brushPalette.getBrush(brushSize + 1).isValid(x, y, width, occupancy)) {
                        brushSize++;
                        if (brushSize == brushPalette.getMaxBrushSize()) {
                            roomIndex++;
                            break;
                        }
                    } else {
                        break;
                    }
                }
            } else {
                while (!valid && brushSize > 0) {
                    brushSize--;
                    valid = brushPalette.getBrush(brushSize).isValid(x, y, width, occupancy);
                }
            }
            if (!valid) {
                // No brush is valid at this point
                return;
            }
            // Mark pixel checked (and painted) by the valid brush size + 2
            checked[y * width + x] = (room_type) (brushSize + 2);
            rooms[y * width + x] = roomIndex;

            for (unsigned int i = (y > 0 ? (y - 1) : 0); i < y + 2 && i < height; ++i) {
                for (unsigned int j = (x > 0 ? (x - 1) : 0); j < x + 2 && j < width; ++j) {
                    if ((i != y || j != x) && checked[i * width + j] < 1) {
                        queue.push(CheckPoint(j, i, width, height, roomIndex, brushSize));
                        checked[i * width + j] = 1;
                    }
                }
            }
        }
    }
}
