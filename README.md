# Move UR in cube

This script aims to provide a convinent way to roughly position UR (using urx with python 2.7).
I assume the workspace of UR is a cube and command UR to go to the specific locations inside the cubes.

The flow goes like this:
1. It partition the workspace in x,y,z direction (as specify by the user)

<img src="https://github.com/maggielovedd/move_ur_in_cube/blob/main/doc/partition.png" width="300" alt="">

2. The user specify the partition to go, then it calculate the center position of that partition
3. execute the position, the orientation is assumed constant


## Result

The robot move from left up to right bottom (execute using ursim).

<img src="https://github.com/maggielovedd/move_ur_in_cube/blob/main/doc/mic.gif" alt="">

The full videos can be found on [youtube](https://youtu.be/joxHA418XJI)
