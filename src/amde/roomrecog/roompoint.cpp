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

#include <algorithm>

#include "roompoint.h"

namespace amde {
    namespace roomrecog {

        RoomPoint::RoomPoint() : MapPoint() {

        }

        RoomPoint::RoomPoint(unsigned int x, unsigned int y, unsigned int width, unsigned int height,
                             room_type roomIndex)
                : MapPoint(x, y, width, height, roomIndex) {

        }

        void RoomPoint::setMajorityRoom(room_type* rooms, unsigned char* adjacency) {
            std::vector<room_type> adjacent = findAdjacent(rooms, adjacency, 3);
            if (adjacent.empty()) return;
            std::sort(adjacent.begin(), adjacent.end());
            unsigned int count = 0;
            room_type bestRoom = 0;
            unsigned int bestCount = 0;
            room_type counting = 0;
            for (unsigned int i = 0; i < adjacent.size(); ++i) {
                room_type room = adjacent[i];
                if (room == counting) {
                    count++;
                    if (bestRoom == 0 || count > bestCount) {
                        bestRoom = counting;
                        bestCount = count;
                        if (bestCount > 4) break;
                    }
                } else {
                    counting = room;
                    count = 1;
                    if (bestRoom == 0) {
                        bestRoom = counting;
                        bestCount = count;
                    }
                }
            }
            if (bestCount > 0) {
                rooms[y * width + x] = bestRoom;
            }
        }

        bool RoomPoint::isSingle(unsigned char* adjacency) {
            return adjacency[y * width + x] < 2;
        }

        bool RoomPoint::fillRoom(room_type roomIndex, room_type* checked, room_type* rooms, room_type* separated,
                                 std::priority_queue<RoomPoint>& nextRoomPoints) {
            if (checked[y * width + x] > 1) return false;
            std::queue<RoomPoint> queue;
            queue.push(*this);
            while (!queue.empty()) {
                RoomPoint point = queue.front();
                queue.pop();
                point.visit(roomIndex, checked, rooms, separated, queue, nextRoomPoints);
            }
            return true;
        }

        void RoomPoint::visit(room_type roomIndex, room_type* checked, room_type* rooms, room_type* separated,
                              std::queue<RoomPoint>& queue, std::priority_queue<RoomPoint>& nextRoomPoints) {
            if (checked[y * width + x] > 1) return;
            separated[y * width + x] = roomIndex;
            checked[y * width + x] = 2;
            for (unsigned int i = (y > 0 ? (y - 1) : 0); i < y + 2 && i < height; ++i) {
                for (unsigned int j = (x > 0 ? (x - 1) : 0); j < x + 2 && j < width; ++j) {
                    if ((i != y || j != x) && rooms[i * width + j] > 0 && checked[i * width + j] < 1) {
                        if (rooms[y * width + x] != rooms[i * width + j]) {
                            nextRoomPoints.push(RoomPoint(j, i, width, height, (occupancy_type) (roomIndex + 1)));
                        } else {
                            queue.push(RoomPoint(j, i, width, height, roomIndex));
                        }
                        checked[i * width + j] = 1;
                    }
                }
            }
        }

        std::vector<room_type> RoomPoint::findAdjacent(room_type* rooms, unsigned char* adjacency, unsigned char minAdjacent) {
            std::vector<room_type> adjacent;
            for (unsigned int i = (y > 0 ? (y - 1) : 0); i < y + 2 && i < height; ++i) {
                for (int j = (x > 0 ? (x - 1) : 0); j < x + 2 && j < width; ++j) {
                    if ((i != y || j != x) && rooms[i * width + j] > 0 && adjacency[i * width + j] >= minAdjacent) {
                        adjacent.push_back(rooms[i * width + j]);
                    }
                }
            }
            return adjacent;
        }
    }
}