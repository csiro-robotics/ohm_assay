#!/bin/bash
# A helper script for running realtime runs on xavier. More or less hard coded.

# Note: publishing frequency has a performance impact. The following impacts were observed on a Jetson Xavier:
# - 5Hz
#   - ~50% reduction in rays processed for all CPU algorithms.
#   - ~3% Reduction in rays processed for CUDA algorithms.
# - 1Hz
#   - 1-3% reduction in rays processed.
#
# The difference in CPU vs CUDA algorithms is mostly due to the CUDA algorithm (at the time of testing) syncing from
# GPU to a map copy on the main thread, then off loading the processing to another thread.
#
# This speaks to a general need for good data handling of reading maps while continuing to process incoming data.
# Copying large, contiguous memory blocks - as supported by ohm and voxblox - then processing on another thread should
# generally proove to be a good usage pattern.

# Clear existing results
rm *.log
rm *.txt
rm *.csv
rm *.ohm
rm *.ply
rm *.bt

# Octomap
# Start timer background
timeout 310 tegrastats --logfile octomap_tegrastats.log &
# Launch.
roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=hz pub_hz:=1 mapper:=octomap
# Bring timer to forergound to wait for it to finish
fg

# Run again without any topic subscribers.
# timeout 310 tegrastats --logfile octomap_no_vis_tegrastats.log &
# roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=none mapper:=octomap
# fg

# Ohm CPU NDT
timeout 310 tegrastats --logfile ohmpop_cpu_ndt_tegrastats.log &
roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=hz pub_hz:=1 mapper:=ohm_cpu ohm_mode:=ndt
fg

# timeout 310 tegrastats --logfile ohmpop_cpu_ndt_no_vis_tegrastats.log &
# roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=none mapper:=ohm_cpu ohm_mode:=ndt
# fg

# Ohm CPU NDT
timeout 310 tegrastats --logfile ohmpop_cpu_ndt_tm_tegrastats.log &
roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=hz pub_hz:=1 mapper:=ohm_cpu ohm_mode:=ndt_tm
fg

# timeout 310 tegrastats --logfile ohmpop_cpu_ndt_tm_no_vis_tegrastats.log &
# roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=none mapper:=ohm_cpu ohm_mode:=ndt_tm
# fg

# Ohm CUDA NDT
timeout 310 tegrastats --logfile ohmpop_cuda_ndt_tegrastats.log &
roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=hz pub_hz:=1 mapper:=ohm_cuda ohm_mode:=ndt
fg

# timeout 310 tegrastats --logfile ohmpop_cuda_ndt_no_vis_tegrastats.log &
# roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=none mapper:=ohm_cuda ohm_mode:=ndt
# fg

# Ohm CUDA NDT-TM
timeout 310 tegrastats --logfile ohmpop_cuda_ndt_tm_tegrastats.log &
roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=hz pub_hz:=1 mapper:=ohm_cuda ohm_mode:=ndt_tm
fg

# timeout 310 tegrastats --logfile ohmpop_cuda_ndt_tm_no_vis_tegrastats.log &
# roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=none mapper:=ohm_cuda ohm_mode:=ndt_tm
# fg

# Ohm CPU occupancy
timeout 310 tegrastats --logfile ohmpop_cpu_occ_tegrastats.log &
roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=hz pub_hz:=1 mapper:=ohm_cpu ohm_mode:=occ
fg

# timeout 310 tegrastats --logfile ohmpop_cpu_occ_no_vis_tegrastats.log &
# roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=none mapper:=ohm_cpu ohm_mode:=occ
# fg

# Ohm CUDA occupancy
timeout 310 tegrastats --logfile ohmpop_cuda_occ_tegrastats.log &
roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=hz pub_hz:=1 mapper:=ohm_cuda ohm_mode:=occ
fg

# timeout 310 tegrastats --logfile ohmpop_cuda_occ_no_vis_tegrastats.log &
# roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=none mapper:=ohm_cuda ohm_mode:=occ
# fg

# Ohm CPU tsdf
timeout 310 tegrastats --logfile ohmpop_cpu_tsdf_tegrastats.log &
roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=hz pub_hz:=1 mapper:=ohm_cpu ohm_mode:=tsdf
fg

# timeout 310 tegrastats --logfile ohmpop_cpu_tsdf_no_vis_tegrastats.log &
# roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=none mapper:=ohm_cpu ohm_mode:=tsdf
# fg

# Ohm CUDA tsdf
timeout 310 tegrastats --logfile ohmpop_cuda_tsdf_tegrastats.log &
roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=hz pub_hz:=1 mapper:=ohm_cuda ohm_mode:=tsdf
fg

# timeout 310 tegrastats --logfile ohmpop_cuda_tsdf_no_vis_tegrastats.log &
# roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=none mapper:=ohm_cuda ohm_mode:=tsdf
# fg

# Voxblox occupancy
timeout 310 tegrastats --logfile voxblox_occupancy_tegrastats.log &
roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=hz pub_hz:=1 mapper:=voxblox_occ
fg

# timeout 310 tegrastats --logfile voxblox_occupancy_no_vis_tegrastats.log &
# roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=none mapper:=voxblox_occ
# fg

# Voxblox tsdf
timeout 310 tegrastats --logfile voxblox_tsdf_tegrastats.log &
roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=hz pub_hz:=1 mapper:=voxblox_tsdf
fg

# timeout 310 tegrastats --logfile voxblox_tsdf_no_vis_tegrastats.log &
# roslaunch ohmassay_launch playback.launch bags:="$(echo $(pwd)/*.bag)" stats:=csv subscribe:=none mapper:=voxblox_tsdf
# fg

# Copy info files from .ros
mv ~/.ros/*.txt .
mv ~/.ros/*.csv .
mv ~/.ros/*.ohm .
mv ~/.ros/*.ply .
mv ~/.ros/*.bt .
