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

#ifndef AALTO_AMDE_ROOM_RECOG_RECTANGLE_H_
#define AALTO_AMDE_ROOM_RECOG_RECTANGLE_H_

namespace amde {
    namespace roomrecog {
        struct Rectangle {
            const unsigned int x;
            const unsigned int y;
            const unsigned int width;
            const unsigned int height;

            Rectangle(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
                    : x(x), y(y), width(width), height(height) {};
        };
    }
}

#endif // AALTO_AMDE_ROOM_RECOG_RECTANGLE_H_
