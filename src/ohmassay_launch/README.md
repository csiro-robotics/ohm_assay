# What is this?

A launch file package to help launch various ohmassay ros configurations. The launch configurations are tailored to try yield the best performance for each application while providing valid comparative run configurations.

The launch file launches the appropriate map generation program (`mapper` argument), starts playback of the specified bag files and launches an RViz configuration for viewing the results.

## Usage

```
roslaunch ohmassay_launch playback.launch bag:="bag1.bag bag2.bag ..." mapper:=[ohm_cpu,ohm_cuda,ohm_ocl,octomap,voxblox_occ,voxblox_tsdf]
```

## Additional launch arguments

- `output` - Output destination `default="screen`
- `bags` - The list of paths (space separated) to the bags to replay.
- `rate` - The real time factor to replay at. `default="1.0"`
- `start` - The number of seconds into the bag to start replaying from. `default="0.0"`
- `duration` - Playback duration `default="0.0"`
- `mapper` - Mapping program: [ohm_cpu, ohm_cuda, ohm_ocl, octomap, voxblox_occ, voxblox_tsdf] `default="ohm_cuda"`
- `resolution` - Voxel size. `default="0.1"`
- `half_extents` - Map half extents `default="10"`
- `sensor_topics` - Sensor frame/topic pairs list. `default="slam_base_link:/slam/odom/high/cloud"`
- `ohm_mode` - Ohm mapping mode: [mean, ndt, occ] `default="ndt"`
- `ocl_vendor` - OpenCL device vendor name"


## How to collect ohm comparative stats

### Online

1. Build `ohm_assay` on target - Xavier, Nuc, etc
2. Source `ohm_assay/install/setup.bash`
3. Copy playback data on to target
4. Use two terminals for the following steps starting playback immediately after timing stats logger
   1. Start timing logger in the results directory with `<runtype>=[octomap, ohmpop_cpu, ohmpop_cuda, ohm_pop_ocl, voxblox_occupancy, voxblox_tsdf]`:
      1. Xavier: `timeout 310 tegrastats --logfile <runtype>_stats.log`
      2. Nuc: `sudo timeout 310 intel_gpu_top -o <runtype>_istats.log` - for Intel GPU stats only
         1. Note: can use `psutil` to log CPU as well though it won't exactly match up. TBD
   2. Start playback from the data directory:
      1. `roslaunch ohmassay_launch playback.launch bags:=$(echo $(pwd)/*.bag) mapper:=<runtype> subscribe:=hz stats:=off`
         1. `<runtype>` options: `[octomap, ohm_cpu, ohm_cuda, ohm_ocl, voxblox_occ, voxblox_tsdf]`
5. Run the steps above again, using `subscribe:=none` and using `<runtype>_no_vis_[i]stats.log` for the timing files.
6. Complete all desired run types
7. Copy `stats.log` files into the results directories.
8. Copy `ohm_assay` run details to the results directories. These log the parameters of the run, found in `~/.ros`
   1. `cp ~/.ros/*.txt <results_dir>/`
9. Extract stats from the results directory to CSV:
   1. `ohmassay_parse_tegrastats *_stats.log`; or
   2. `ohmassay_parse_intel_top *_istats.log` (not yet implemented)

### Offline

Find the offline data. This is SLAM processed data and we expect a PLY file and a text based trajectory. Additional formats supported allow PLY trajectory or PLY ray cloud inputs with no trajectory. Ohm can also be built with PDAL support for other point cloud inputs, but `ohm_assay` is not configured to do so.

1. Build `ohm_assay` on target
2. Source `ohm_assay/install/setup.bash`
3. Create a results directory to run from
4. Run each map generation program targeting SLAM output files using `ohmassay_offline` script:
   1. `ohmassay_offline --cloud <cloud.ply> --traj <traj.txt> --app cpu cuda ocl oct vox --occ occ mean ndt ndt-tm tsdf --resolution 0.1 --ocl-vendor intel nvidia --gpu-segment-min 10 --gpu-segment-max 20 --save-maps --threads 1 2 4 6 --tsdf-mode fast merged simple`
5. Wait for results to complete - octomap may be very slown or even unable to complete offline processing in which case it can be removed from the run and targetted separately at lower resolution.
6. Tabulate results:
   1. `ohmassay_parse_offline --mode rps -o <dataset>_rps.csv`
   2. `ohmassay_parse_offline --mode rtf -o <dataset>_rtf.csv`
   3. `ohmassay_parse_offline --mode rtfi -o <dataset>_rtfi.csv`
   4. `ohmassay_parse_offline --mode time -o <dataset>_timing.csv`

The invocation of `ohmassay_offline` is configured to run all valid permutations of `--app` `--ocl-vendor` `--threads` and the range described by `--gpu-segment-min` to `--gpu-segment-max`. The following combinations are compatible:

| `--app`        | Option                                        |
| -------------- | --------------------------------------------- |
| cuda           | - `--gpu-segment-min` and `--gpu-segment-max` |
| ocl            | - `--gpu-segment-min` and `--gpu-segment-max` |
| ocl            | - `--ocl-vendor`                              |
| cpu, cuda, ocl | `--occ occ mean ndt ndt-tm`                   |
| oct            | `--occ occ`                                   |
| vox            | `--occ occ tsdf`                              |
| vox            | `--threads` when `--occ=tsdf`                 |
| vox            | `--tsdf-mode` when `--occ=tsdf`               |

Adding `--dry-run` will list the command lines to be executed.
