# Project for comparative assessment of OHM

This repo is provided to assess the performance of OHM against several other voxel or octree based libraries.

## Dependencies

- ROS
   - See http://wiki.ros.org/ROS/Installation
- CUDA - see ohm dependencies
- OpenCL - see ohm dependencies
- colcon and ninja for building (python3-colcon-common-extensions, python3-colcon-mixin, ninja-build)
   - `sudo apt install -y python3-colcon-common-extensions python3-colcon-mixin ninja-build`
- rosdep to install other dependencies.

The following bash code can be use to install the dependencies.

`rosdep` installation

```bash
# ROS Noetic
sudo apt install -y python3-rosdep
# ROS Melodic and earlier
sudo apt install -y python-rosdep
# ROS Melodic or Noetic
sudo rosdep init
rosdep update
```

Other dependencies:
```bash
sudo apt install -y python3-colcon-common-extensions python3-colcon-mixin ninja-build
sudo apt install -y cmake zlib1g-dev libglm-dev libtbb-dev libpdal-dev doxygen
# From ohm_assay root directory
rosdep install --from-paths src --ignore-src -r -y
```

> Note: with Ubuntu 20.04 and ROS noetic, this will fail to install the following `python-requests`. This should be `python3-requests` on Ubuntu 20.04. The `rosdep` installation steps above ensure `python3-requests` is already installed.

See also the dependencies for the ohm project.

## Build instructions

### Command line

1. Set the environment variable `COLCON_HOME=./.colcon`
2. Source the ros `setup` script: e.g., `source /opt/ros/melodic/setup.bash`
3. `colcon build --mixin <build-type>`

Available build types:

- `rel-with-deb`
- `release`
- `default`

Omitting the build type mixin defaults to `release`

### Building from VSCode

1. Set the environment variable `COLCON_HOME=./.colcon`
2. Source the ros `setup` script: e.g., `source /opt/ros/melodic/setup.bash`
3. Start VSCode
4. Run the VSCode command `Tasks: Run Build Task` (shortcut: `Ctrl+Shift+B` or `Ctrl+/` depending on bindings)
   - Select build task and type

## Usage instructions

Example data files are available for download from TODO(KS). This includes data in ROS bag format containing online SLAM odometry and lidar sensor data. These files simulate running an online OHM solution. Additionally, there are pre-processed, globally optimal point cloud and trajectory data files.

To run the online solution:

1. Source the workspace `setup` script: e.g., `source install/setup.bash`
2. Launch ohmassay playback: e.g., `roslaunch ohmassay_launch playback.launch "bags:=$(echo /path/to/bags/*.bag)" [options]`

See [playback.launch](./src/ohmassay_launch/launch/playback.launch) for a list of options and arguments.

For offline processing, OHM assay contains the following programs:

| Program               | Description                                                     |
| --------------------- | --------------------------------------------------------------- |
| `ohmpocpu`            | Map generation using OHM algorithm in CPU                       |
| `ohmpocuda`           | Map generation using OHM algorithm in GPU using CUDA            |
| `ohmpococl`           | Map generation using OHM algorithm in GPU using OpenCL          |
| `octomappop`          | Map generation using the octomap library (CPU, single threaded) |
| `voxbloxpopoccupancy` | Map generation using voxblox library occupancy algorithms       |
| `voxbloxpoptsdf`      | Map generation using voxblox library TSDF algorithms            |

To run the offline solutions

1. Source the workspace `setup` script: e.g., `source install/setup.bash`
2. Launch `<lib>pop<type>`: e.g., `ohmpopcuda point_cloud.ply trajectory.txt [options]"`

Run `<lib>pop<type> --help` for a list of options and arguments.

### ROS bag data requirements

OHM assay requires the following data from a ros bag data source:

- A `tf` tree containing a moving frame which represents the lidar sensor.
- A `PointCloud2` topic containing lidar sensor data in odometry or map frame.

### Point cloud requirements

The OHM assay map population programs require a point cloud with a trajectory or raycloud generated from a SLAM algorithm. The point cloud and trajectory are expected to be globally optimal SLAM solutions and must have correlated timestamps. Each point in the point cloud must have a timestamp with (double precision is recommended) and each point in the trajectory must have a similar timestamp from which the sensor location for each point can be inferred.

The cloud files must be PLY format, while a trajectory can be either a text format (see below) or a PLY series of trajectory points.

A trajectory may be omitted when the cloud file is a [raycloud](https://github.com/csiro-robotics/raycloudtools) PLY file.

Valid trajectory text formats are:

- Space delimited, `%time x y z`, one entry per line, column names ignored. Additional fields are allowed, but are ignored.
- Point cloud XYZ ASCII file format, column names dictate content.
- Point cloud PLY format.

Below is a list of point cloud fields used by OHM.

| Field Name      | Mandatory? | Description                                                                          |
| --------------- | ---------- | ------------------------------------------------------------------------------------ |
| `gps_time`      | Yes*       | Timestamp field. Only one of the time fields is required, listed in preferred order. |
| `gpstime`       |            |                                                                                      |
| `internal_time` |            |                                                                                      |
| `internaltime`  |            |                                                                                      |
| `offset_time`   |            |                                                                                      |
| `offsettime`    |            |                                                                                      |
| `timestamp`     |            |                                                                                      |
| `time`          |            |                                                                                      |
| `x`             | Yes        | Point cloud position X coordinate channel.                                           |
| `y`             | Yes        | Point cloud position Y coordinate channel.                                           |
| `x`             | Yes        | Point cloud position Z coordinate channel.                                           |
| `nx`            | No**       | Point normal X channel.                                                              |
| `normal_x`      |            |                                                                                      |
| `ny`            | No**       | Point normal Y channel.                                                              |
| `normal_y`      |            |                                                                                      |
| `nz`            | No**       | Point normal Z channel.                                                              |
| `normal_z`      |            |                                                                                      |
| `red`           | No         | Point colour, red channel.                                                           |
| `r`             |            |                                                                                      |
| `green`         | No         | Point colour, green channel.                                                         |
| `g`             |            |                                                                                      |
| `blue`          | No         | Point colour, blue channel.                                                          |
| `b`             |            |                                                                                      |
| `alpha`         | No         | Point colour, alpha channel.                                                         |
| `a`             |            |                                                                                      |
| `intensity`     | No***      |                                                                                      |

\* Any of the time field names are acceptable. The first match is used as the timestamp field.<br>
\*\* All three normal channels are required for a ray cloud with the normal stored as a vector from the sample point back to the sensor (not unit length).<br>
\*\*\* Intensity is required for the Normal Distribution Transform Traversability Model (NDT-TM).<br>
