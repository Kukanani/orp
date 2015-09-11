#!/bin/bash
for thefile in ./*;
do
  echo $thefile;
  filenam=${thefile##*/}
  echo $filenam;
  rosrun orp virtual_train $thefile ${filenam%.ply} /home/adam/nrg/nrg_sia5/src/nrg-ros-support/orp/data/sixdof/; 
done;

