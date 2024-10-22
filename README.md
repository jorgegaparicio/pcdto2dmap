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
Code will search for a .pcd file at /home/grvc/Escritorio/filtered_scans_obstaculos.pcd
