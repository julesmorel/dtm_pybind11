# Terrain Reconstruction from Point Clouds Using Implicit Deformable Model

* Python package, implementation of [Morel et al. 2020](https://link.springer.com/chapter/10.1007/978-3-030-50433-5_20).

## Setup
### Requirements
* Linux (tested on Ubuntu 21.10)
* PCL 1.11
* Python 3.9.7
* GNU Scientific Library

### Build
Compile the package by running the following commands:
```shell
cmake .
make
```

## Usage
Import the package:
```shell
import dtm
```
### Build the implicit surface model:
* MINIMUM_POINTS: the path to the ASCII file containing the minimum points to approximate.
* BB_POINTS: the path to the ASCII file containing the 4 points of the minimum bounding rectangle of the minimum points.
* MINIMUM_POINTS_NUMBER: the minimum number of points in the quadtree leaves.
* MINIMUM_SIZE: the minimum size (in meter) of the quadtree leaves.
```shell
surface = dtm.dtm(MINIMUM_POINTS,BB_POINTS,MINIMUM_POINTS_NUMBER,MINIMUM_SIZE)
```

### Apply the deformation to the initial implicit surface (optional):
* NUMBER_ITERATIONS: the number of iterations of the deformable model.
* GAMMA: the ratio adjusting the elasticity of the surface.
```shell
surface.applyDeformableModel(NUMBER_ITERATIONS,GAMMA)
```

### Extract the surface mesh model by polygonization of the implicit surface:
* NX, NY and NZ: the division of the polygonizer 3D grid
```shell
surface.polygonize(NX,NY,NZ)
```

### Save the surface mesh model as an .obj:
* OBJ_FILE: the path to the .obj file.
```shell
surface.exportDTM(OBJ_FILE)
```

### Plot the results of each steps of the algorithm (optional):
```shell
surface.display()
```

## Example
To run the algorithm on the testing data, simply run:
```shell
python test.py
```
The graphical output should be similar to the image below
![screenshot](screenshot.jpg?raw=true "Screenshot")
