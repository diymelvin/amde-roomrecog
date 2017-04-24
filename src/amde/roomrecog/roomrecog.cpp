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

#include "../../../include/amde/roomrecog/roomrecog.h"
#include "brushpalette.h"
#include "checkpoint.h"
#include "roompoint.h"

namespace amde {
    namespace roomrecog {

        // Local functions' forward declarations
        void separateRooms(room_type* rooms, const Rectangle& point, room_type* separated);
        void findRooms(const occupancy_type* occupancy, const Rectangle& point, room_type* rooms, const BrushPalette& brushPalette);
        void removeSingles(room_type* rooms, unsigned int width, unsigned int height);
        unsigned char findAdjacency(room_type* rooms, unsigned int x, unsigned int y, unsigned int width, unsigned int height);
        bool findAdjacentSameRoom(room_type* rooms, const RoomPoint& point, RoomPoint& adjacent);

        bool findRooms(const Map& map, unsigned int x, unsigned int y, room_type* rooms) {
            const occupancy_type* occupancy = map.map;
            unsigned int width = map.width;
            unsigned int height = map.height;

            BrushPalette brushPalette(map.resolution, 0.9f, 0.15f, 3);

            if (occupancy[y * width + x] >= 0 && occupancy[y * width + x] < AMDE_MAP_OCCUPIED) {
                Rectangle point(x, y, width, height);

                unsigned int size = map.width * map.height;
                room_type initRooms[size];
                for (unsigned int i = 0; i < size; ++i) {
                    initRooms[i] = 0;
                }

                findRooms(occupancy, point, initRooms, brushPalette);

                // Remove single or few pixel rooms
                removeSingles(initRooms, width, height);

                separateRooms(initRooms, point, rooms);
                return true;
            } else {
                return false;
            }
        }

        void separateRooms(room_type* rooms, const Rectangle& mapPoint, room_type* separated) {
            unsigned int size = mapPoint.width * mapPoint.height;
            room_type checked[size];
            for (unsigned int i = 0; i < size; ++i) {
                checked[i] = 0;
            }

            std::priority_queue<RoomPoint> queue;
            queue.push(RoomPoint(mapPoint.x, mapPoint.y, mapPoint.width, mapPoint.height, 1));
            room_type roomIndex = 1;
            while (!queue.empty()) {
                RoomPoint point = queue.top();
                queue.pop();
                if (point.fillRoom(roomIndex, checked, rooms, separated, queue)) {
                    roomIndex += 1;
                    for (int i = 0; i < size; i++) {
                        if (checked[i] < 2) checked[i] = 0;
                    }
                }
            }
        }

        void findRooms(const occupancy_type* occupancy, const Rectangle& mapPoint, room_type* rooms, const BrushPalette& brushPalette) {
            unsigned int size = mapPoint.width * mapPoint.height;
            room_type checked[size];
            for (unsigned int i = 0; i < size; ++i) {
                checked[i] = 0;
            }

            std::priority_queue<CheckPoint> queue;
            queue.push(CheckPoint(mapPoint.x, mapPoint.y, mapPoint.width, mapPoint.height, 1,
                                  brushPalette.getMaxBrushSize()));
            while (!queue.empty()) {
                CheckPoint point = queue.top();
                queue.pop();
                point.visit(checked, rooms, occupancy, brushPalette, queue);
            }
        }

        void removeSingles(room_type* rooms, unsigned int width, unsigned int height) {
            std::queue<RoomPoint> queue;
            unsigned char adjacency[width * height];
            for (unsigned int i = 0; i < height; ++i) {
                for (unsigned int j = 0; j < width; ++j) {
                    room_type room = rooms[i * width + j];
                    if (room > 0) {
                        unsigned char ad = findAdjacency(rooms, j, i, width, height);
                        adjacency[i * width + j] = ad;
                        if (ad < 2) {
                            queue.push(RoomPoint(j, i, width, height, room));
                        }
                    } else {
                        adjacency[i * width + j] = 0;
                    }
                }
            }
            while (!queue.empty()) {
                RoomPoint point = queue.front();
                queue.pop();
                point.setMajorityRoom(rooms, adjacency);
                RoomPoint adjacent;
                if (findAdjacentSameRoom(rooms, point, adjacent)) {
                    if (adjacent.isSingle(adjacency)) {
                        queue.push(adjacent);
                    }
                }
            }
        }

        unsigned char findAdjacency(room_type* rooms, unsigned int x, unsigned int y, unsigned int width, unsigned int height) {
            unsigned char sameRoomNeighbors = 0;
            char room = rooms[y * width + x];
            for (unsigned int i = (y > 0 ? (y - 1) : 0); i < y + 2 && i < height; ++i) {
                for (unsigned int j = (x > 0 ? (x - 1) : 0); j < x + 2 && j < width; ++j) {
                    if ((i != y || j != x) && rooms[i * width + j] == room) {
                        sameRoomNeighbors++;
                    }
                }
            }
            return sameRoomNeighbors;
        }

        bool findAdjacentSameRoom(room_type* rooms, const RoomPoint& point, RoomPoint& adjacent) {
            for (unsigned int i = (point.y > 0 ? (point.y - 1) : 0); i < point.y + 2 && i < point.height; ++i) {
                for (unsigned int j = (point.x > 0 ? (point.x - 1) : 0); j < point.x + 2 && j < point.width; ++j) {
                    if ((i != point.y || j != point.x) &&
                        rooms[i * point.width + j] == rooms[point.y * point.width + point.x]) {
                        adjacent.x = j;
                        adjacent.y = i;
                        adjacent.width = point.width;
                        adjacent.height = point.height;
                        adjacent.roomIndex = point.roomIndex;
                        return true;
                    }
                }
            }
            return false;
        }
    }
}