#!/bin/bash
# A helper script for running realtime runs on xavier. More or less hard coded.

# See note in run_realtime_xavier.bash regarding publishing frequency performance impacts.

# Clear existing results
sudo rm *.csv
rm *.txt
rm *.ohm
rm *.ply
rm *.bt

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# Ohm CPU NDT
timeout 310 intel_load.py cpu -o ohmpop_cpu_ndt_nucstats.csv &
roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=hz pub_hz:=1 mapper:=ohm_cpu ohm_mode:=ndt
fg

# timeout 310 intel_load.py cpu -o ohmpop_cpu_ndt_no_vis_nucstats.csv &
# roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=none mapper:=ohm_cpu ohm_mode:=ndt
# fg

# Ohm CPU NDT-TM
timeout 310 intel_load.py cpu -o ohmpop_cpu_ndt_tm_nucstats.csv &
roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=hz pub_hz:=1 mapper:=ohm_cpu ohm_mode:=ndt_tm
fg

# timeout 310 intel_load.py cpu -o ohmpop_cpu_ndt_tm_no_vis_nucstats.csv &
# roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=none mapper:=ohm_cpu ohm_mode:=ndt_tm
# fg

# Ohm OpenCL NDT
sudo --preserve-env=PYTHONPATH,PATH,LD_LIBRARY_PATH env LD_LIBRARY_PATH=$LD_LIBRARY_PATH timeout 310 intel_load.py gpu -o ohmpop_ocl_ndt_nucstats.csv &
roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=hz pub_hz:=1 mapper:=ohm_ocl ohm_mode:=ndt
fg

# sudo --preserve-env=PYTHONPATH,PATH,LD_LIBRARY_PATH env LD_LIBRARY_PATH=$LD_LIBRARY_PATH timeout 310 intel_load.py gpu -o ohmpop_ocl_ndt_no_vis_nucstats.csv &
# roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=none mapper:=ohm_ocl ohm_mode:=ndt
# fg

# Ohm OpenCL NDT
sudo --preserve-env=PYTHONPATH,PATH,LD_LIBRARY_PATH env LD_LIBRARY_PATH=$LD_LIBRARY_PATH timeout 310 intel_load.py gpu -o ohmpop_ocl_ndt_tm_nucstats.csv &
roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=hz pub_hz:=1 mapper:=ohm_ocl ohm_mode:=ndt_tm
fg

# sudo --preserve-env=PYTHONPATH,PATH,LD_LIBRARY_PATH env LD_LIBRARY_PATH=$LD_LIBRARY_PATH timeout 310 intel_load.py gpu -o ohmpop_ocl_ndt_tm_no_vis_nucstats.csv &
# roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=none mapper:=ohm_ocl ohm_mode:=ndt_tm
# fg

# Ohm CPU occupancy
timeout 310 intel_load.py cpu -o ohmpop_cpu_occ_nucstats.csv &
roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=hz pub_hz:=1 mapper:=ohm_cpu ohm_mode:=occ
fg

# timeout 310 intel_load.py cpu -o ohmpop_cpu_occ_no_vis_nucstats.csv &
# roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=none mapper:=ohm_cpu ohm_mode:=occ
# fg

# Ohm OpenCL occupancy
sudo --preserve-env=PYTHONPATH,PATH,LD_LIBRARY_PATH env LD_LIBRARY_PATH=$LD_LIBRARY_PATH timeout 310 intel_load.py gpu -o ohmpop_ocl_occ_nucstats.csv &
roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=hz pub_hz:=1 mapper:=ohm_ocl ohm_mode:=occ
fg

# sudo --preserve-env=PYTHONPATH,PATH,LD_LIBRARY_PATH env LD_LIBRARY_PATH=$LD_LIBRARY_PATH timeout 310 intel_load.py gpu -o ohmpop_ocl_occ_no_vis_nucstats.csv &
# roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=none mapper:=ohm_ocl ohm_mode:=occ
# fg

# Ohm CPU tsdf
timeout 310 intel_load.py cpu -o ohmpop_cpu_tsdf_nucstats.csv &
roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=hz pub_hz:=1 mapper:=ohm_cpu ohm_mode:=tsdf
fg

# timeout 310 intel_load.py cpu -o ohmpop_cpu_tsdf_no_vis_nucstats.csv &
# roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=none mapper:=ohm_cpu ohm_mode:=tsdf
# fg

# Ohm OpenCL tsdf
sudo --preserve-env=PYTHONPATH,PATH,LD_LIBRARY_PATH env LD_LIBRARY_PATH=$LD_LIBRARY_PATH timeout 310 intel_load.py gpu -o ohmpop_ocl_tsdf_nucstats.csv &
roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=hz pub_hz:=1 mapper:=ohm_ocl ohm_mode:=tsdf
fg

# sudo --preserve-env=PYTHONPATH,PATH,LD_LIBRARY_PATH env LD_LIBRARY_PATH=$LD_LIBRARY_PATH timeout 310 intel_load.py gpu -o ohmpop_ocl_tsdf_no_vis_nucstats.csv &
# roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=none mapper:=ohm_ocl ohm_mode:=tsdf
# fg

# Voxblox tsdf
timeout 310 intel_load.py cpu -o voxblox_tsdf_nucstats.csv &
roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=hz pub_hz:=1 mapper:=voxblox_tsdf
fg

# timeout 310 intel_load.py cpu -o voxblox_tsdf_no_vis_nucstats.csv &
# roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=none mapper:=voxblox_tsdf
# fg

# Copy info files from .ros
mv ~/.ros/*.txt .
mv ~/.ros/*.csv .
mv ~/.ros/*.ohm .
mv ~/.ros/*.ply .
mv ~/.ros/*.bt .
