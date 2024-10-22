# pcdto2dmap
Simple code to obtain a binary obstacle map from a 3D map in a .pcd file.

Modify the path to the .pcd file in the .cpp code to include your .pcd file.
Compile with:
```
mkdir build
cd build
cmake ..
make
```
Run simply with:
```
./pcd_to_2dmap
```
## Explanation.
Code will search for a .pcd file in the specified path, then set any points between the specified minimum and maximum heights as obstacles, and then it will save the resulting 2D map in a .pgm file in the /build folder.
Then it's set to define any points between 0.5 and 5m as obstacles, ignoring ground and ceiling.
