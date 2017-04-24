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

#include "brushpalette.h"

namespace amde {
    namespace roomrecog {

        BrushPalette::BrushPalette(float resolution, float doorSize, float padding, unsigned int brushCount) : brushes() {
            // Largest brush (minimum) radius
            unsigned int maxSize = std::max(AMDE_BRUSHPALETTE_MIN_BRUSH_COUNT, static_cast<unsigned int>(std::round(
                    ((doorSize / 2) + padding) / resolution)));
            brushCount = std::max(AMDE_BRUSHPALETTE_MIN_BRUSH_COUNT, brushCount);
            brushCount = std::min(AMDE_BRUSHPALETTE_MAX_BRUSH_COUNT, brushCount);
            int minSize = maxSize - brushCount + 1;
            if (minSize < 1) {
                int increment = 1 - minSize;
                minSize += increment;
                maxSize += increment;
            }
            unsigned int stepSize = std::max(1U, maxSize / brushCount);
            this->brushCount = brushCount;
            for (unsigned int i = 0; i < brushCount; i++) {
                unsigned int radius = minSize + i * stepSize;
                std::vector<bool> brush;
                createBrush(brush, radius);
                this->brushes.push_back(Brush(brush, radius));
            }
        }

        void BrushPalette::createBrush(std::vector<bool>& brush, unsigned int radius) {
            unsigned int size = 2 * radius + 1;
            for (unsigned int i = 0; i < size; ++i) {
                int y = i - radius;
                for (unsigned int j = 0; j < size; ++j) {
                    int x = j - radius;
                    if (y == 0 && x == 0) {
                        brush.push_back(true);
                    } else {
                        int dist = (int) round(sqrt(x * x + y * y) + 0.2);
                        brush.push_back(dist <= radius);
                    }
                }
            }
        }
    }
}