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

#include "brush.h"

namespace amde {
    namespace roomrecog {
        Brush::Brush(const std::vector<bool>& brush, unsigned int radius) : brush(brush), radius(radius), size(radius * 2 + 1) {

        }

        bool Brush::isValid(unsigned int x, unsigned int y, unsigned int width, const occupancy_type* occupancy) const {
            for (unsigned int i = 0; i < size; ++i) {
                if (y + i >= radius) {
                    for (unsigned int j = 0; j < size; ++j) {
                        if (x + j >= radius) {
                            occupancy_type occupied = occupancy[(y - radius + i) * width + (x - radius + j)];
                            if (brush[i * size + j] && (occupied < 0 || occupied >= AMDE_MAP_OCCUPIED)) {
                                return false;
                            }
                        }
                    }
                }
            }
            return true;
        }
    }
}
