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

#ifndef AALTO_AMDE_ROOM_RECOG_ROOMPOINT_H_
#define AALTO_AMDE_ROOM_RECOG_ROOMPOINT_H_

#include <queue>

#include "mappoint.h"

namespace amde {
    namespace roomrecog {
        struct RoomPoint : public MapPoint {
            RoomPoint();
            RoomPoint(unsigned int x, unsigned int y, unsigned int width, unsigned int height, room_type roomIndex);

            void setMajorityRoom(room_type* rooms, unsigned char* adjacency);
            bool isSingle(unsigned char* adjacency);

            bool fillRoom(room_type roomIndex, room_type* checked, room_type* rooms, room_type* separated,
                          std::priority_queue<RoomPoint>& nextRoomPoints);

            friend bool operator<(const RoomPoint& lhs, const RoomPoint& rhs) {
                return lhs.roomIndex >= rhs.roomIndex;
            }

        private:
            void visit(room_type roomIndex, room_type* checked, room_type* rooms, room_type* separated, std::queue<RoomPoint>& queue,
                       std::priority_queue<RoomPoint>& nextRoomPoints);

            std::vector<room_type> findAdjacent(room_type* rooms, unsigned char* adjacency, unsigned char minAdjacent);
        };
    }
}

#endif // AALTO_AMDE_ROOM_RECOG_ROOMPOINT_H_
