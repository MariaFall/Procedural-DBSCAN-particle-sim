# 2D Particles & Dynamic DBSCAN Simulation
A real-time 2D particle simulation utilizing Python for the graphical interface and a C backend for computations. Includes implementations for collision physics, dynamic DBSCAN clustering, and a quadtree spatial partitioning implementation to improve performance.

![dbscangif](https://github.com/user-attachments/assets/c3f6531b-ea66-4389-8660-84e46c55ddd2)


## Features

* **C Backend:** Intensive loops (DBSCAN and line pairing) are written in C and compiled as a shared library.
* **Quadtree Spatial Partitioning:** Reduces collision detection complexity.
* **Dynamic DBSCAN Clustering:** Real-time point classification and visual cluster connections.
* **Interactive GUI:** Adjust parameters like EPS distance, MinPts and particle count on the fly using PyQtGraph.

![editgif](https://github.com/user-attachments/assets/d353a843-79a2-41a2-97bb-59328d954f6e)


## Requirements
* `Python 3.13`
* `numpy`
* `pyqtgraph`
* `PyQt5` 
* `GCC` 

## Build Instructions

Compile the C shared library before running the Python script. 

**Linux/macOS:**
```bash
gcc -shared -o calc.so -fPIC calc.c -lm
