# AMDE Room recognition
This algorithm library was implemented with the intention of recognizing and separating rooms from a <a href='http://www.ros.org/'>ROS</a> OccupancyGrid created by many mapping algorithms such as SLAM.

As input the algorithm takes an OccupancyGrid integer array, -1 marking unknown, 0 free and 100 occupied cells. It generates a 3x3 cell average, and then produces a similar occupancy grid of the separated rooms. In the resulting array 0 marks unknown and values 1+ mark room numbers.

This algorithm was developed as a part of Aalto University School of Electrical Engineering's Intelligent Robotics group project work. The project is Autonomous Mapping of Dynamic Environments, AMDE.

### Licence:

Copyright (c) 2017 Samuli Peurasaari

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

* The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.