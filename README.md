# gtsam_node
graph optimization in ROS using GTSAM
# GTSAM installation

clone gtsam from github
https://github.com/borglab/gtsam

```
$ mkdir build
$ cd build
$ cmake ..
$ make check (optional, runs unit tests)
$ make install
```
change the parameter below in CmakeLists to where gtsam is built

"set(GTSAM_DIR "/home/iman/gtsam/build")"


