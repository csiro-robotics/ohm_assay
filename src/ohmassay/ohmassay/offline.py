#!/usr/bin/python3

# Copyright (c) 2021
# Commonwealth Scientific and Industrial Research Organisation (CSIRO)
# ABN 41 687 119 230
#
# Author: Kazys Stepanas
"""Ohm timing utility.

This program runs ohm and related offline comparison applications in various, specified configurations recording the
results to file.

Results may be tabulated using parse_timing.

Supported applications:
- ohmpopcpu
- ohmpopcuda
- ohmpopocl
- octopop
- voxbloxpoptsdf
- voxbloxpopoccupancy

"""

import argparse
import copy
import subprocess
import sys


def parse_args(cmd_args=None, description=None):
    """Parse command line arguments."""
    if description is None:
        description = 'ohm timing utility'
    parser = argparse.ArgumentParser(
        description=description, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('--cloud', type=str, help='Point cloud file')
    parser.add_argument('--traj', type=str, help='Trajectory file')
    parser.add_argument('--app',
                        type=str,
                        nargs='+',
                        default=['cpu'],
                        choices=['cpu', 'cuda', 'ocl', 'oct', 'vox'],
                        help='Application type(s) to use for the run')
    parser.add_argument('--occ',
                        type=str,
                        nargs='+',
                        default=['occ'],
                        choices=['mean', 'occ', 'ndt', 'ndt-tm', 'tsdf'],
                        help='Occupancy type(s) to use for the run')
    parser.add_argument('--resolution',
                        type=float,
                        default=0.1,
                        help='Map resolution')
    parser.add_argument('--ocl-vendor',
                        type=str,
                        nargs='+',
                        default=['none'],
                        help='OpenCL accelerator vendor type(s).')
    parser.add_argument('--gpu-segment-length',
                        type=float,
                        nargs='+',
                        default=[0],
                        help='GPU ray segment lengths to try')
    parser.add_argument('--ray-length-max',
                        type=float,
                        default=0,
                        help='Maximum ray length (clipped).')
    parser.add_argument('--dry-run',
                        action='store_true',
                        help='Show command lines, but do not execute')
    parser.add_argument('--save-maps',
                        action='store_true',
                        help='Save data files?')
    parser.add_argument(
        '--threads',
        type=int,
        nargs='+',
        default=[0],
        help='Number of threads for apps which support threads.')
    parser.add_argument('--tsdf-mode',
                        choices=['fast', 'merged', 'simple'],
                        nargs='+',
                        default=['fast'],
                        help='TSDF integration mode.')
    parser.add_argument('--dir',
                        type=str,
                        nargs='+',
                        choices=['forward', 'reverse'],
                        default=[],
                        help='GPU ray tracing direction. Default is reverse.')
    parser.add_argument('--stats',
                        action='store_true',
                        default=False,
                        help='Log ongoing CVS stats?')

    args = parser.parse_args(cmd_args)
    return args


class RunDefinition:
    """Defines a run including application name and command line arguments."""
    def __init__(self):
        """Initialise the run definition."""
        self.cloud = None
        self.traj = None
        self.app_type = 'cpu'
        self.occupancy = 'occ'
        self.tsdf_mode = 'fast'
        self.resolution = 0.1
        self.ocl_vendor = None
        self.gpu_segment_length = 0
        self.ray_length_max = 0
        self.save_maps = False
        self.threads = 0
        self.forward = False
        self.stats = False
        self.dry_run = False

    def execute(self):
        """Execute the run."""
        program_name = self._application_name()
        out_name = self.output_name()

        save_maps = 'true' if self.save_maps else 'false'
        args = [
            program_name, self.cloud, self.traj, out_name, '--save-info',
            '--preload', '--save-cloud={}'.format(save_maps),
            '--save-map={}'.format(save_maps)
        ]
        args.append('--resolution={}'.format(self.resolution))

        if self.ray_length_max:
            args.append('--ray-length-max={}'.format(self.ray_length_max))

        if self.app_type == 'ocl' and self.ocl_vendor:
            args.append('--vendor={}'.format(self.ocl_vendor))

        if self.is_gpu_app():
            if self.gpu_segment_length > 0:
                args.append('--gpu-ray-segment-length={}'.format(
                    self.gpu_segment_length))

        if self.is_ohm_app():
            if self.occupancy == 'mean':
                args.append('--voxel-mean')
            elif self.occupancy == 'ndt':
                args.append('--ndt')
            elif self.occupancy == 'ndt-tm':
                args.append('--ndt=tm')
            elif self.occupancy == 'tsdf':
                args.append('--tsdf')

        elif self.is_tsdf():
            args.append('--mode={}'.format(self.tsdf_mode))

        if self.is_multithreaded_app():
            args.append('--threads={}'.format(self.threads))

        if self.stats:
            args.append('--stats=csv')

        if self.is_gpu_app() and self.forward:
            args.append('--forward')

        if not self.dry_run:
            print(program_name)
            proc = subprocess.Popen(' '.join(args), shell=True)
            proc.wait()
        else:
            print(' '.join(args))

    def _application_name(self):
        """Return the application executable name."""
        if self.app_type == 'cpu':
            return 'ohmpopcpu'
        if self.app_type == 'cuda':
            return 'ohmpopcuda'
        if self.app_type == 'ocl':
            return 'ohmpopocl'
        if self.app_type == 'oct':
            return 'octopop'
        if self.app_type == 'vox':
            if self.occupancy == 'tsdf':
                return 'voxbloxpoptsdf'
            return 'voxbloxpopoccupancy'
        raise 'Unknown application type: ' + self.app_type

    def output_name(self):
        """Return the output file name for this configuration."""
        out_name = self.app_type
        if self.app_type == 'ocl' and self.ocl_vendor is not None:
            out_name += '-{}'.format(self.ocl_vendor)
        if self.is_tsdf() and self.tsdf_mode:
            out_name += '-{}'.format(self.tsdf_mode)
        if self.is_gpu_app():
            if self.gpu_segment_length > 0:
                out_name += '-s{}m'.format(int(self.gpu_segment_length))
        if self.is_multithreaded_app():
            if self.threads > 0:
                out_name += '-threads{}'.format(self.threads)
        out_name += '-{}-r{}cm'.format(self.occupancy,
                                       int(self.resolution * 100))

        if self.forward:
            out_name += '-fwd'

        return out_name

    def is_gpu_app(self):
        """Check if this is a GPU app."""
        gpu_apps = ['cuda', 'ocl']
        return self.app_type in gpu_apps

    def is_ohm_app(self):
        """Check if this is an ohmp program."""
        ohm_apps = ['cpu', 'cuda', 'ocl']
        return self.app_type in ohm_apps

    def is_occupancy_type_supported_for_app(self):
        if self.is_ohm_app():
            return True
        if self.app_type == 'vox':
            return self.occupancy in ['occ', 'tsdf']
        return self.occupancy == 'occ'

    def is_multithreaded_app(self):
        return self.app_type == 'vox' and self.occupancy == 'tsdf'

    def is_tsdf(self):
        return self.app_type == 'vox' and self.occupancy == 'tsdf'


def float_range(start, stop, step):
    """Generator function to iterate a floating point range."""
    while start < stop:
        yield float(start)
        start += float(step)


def main(cmd_args=None):
    """Main function."""
    args = parse_args(cmd_args, __doc__)

    # Build run list
    runs = []

    for app_type in args.app:
        run = RunDefinition()
        run.cloud = args.cloud
        run.traj = args.traj
        run.resolution = args.resolution
        run.dry_run = args.dry_run
        run.save_maps = args.save_maps
        run.gpu_segment_length = 0
        run.ray_length_max = args.ray_length_max
        run.forward = False
        run.stats = args.stats

        run.app_type = app_type

        for occ in args.occ:
            run.occupancy = occ
            if not run.is_occupancy_type_supported_for_app():
                continue

            if not run.is_gpu_app():
                if run.is_tsdf():
                    for tsdf_mode in args.tsdf_mode:
                        run.tsdf_mode = tsdf_mode
                        if run.is_multithreaded_app():
                            for threads in args.threads:
                                run.threads = threads
                                runs.append(copy.copy(run))
                        else:
                            runs.append(copy.copy(run))
                elif run.is_multithreaded_app():
                    for threads in args.threads:
                        run.threads = threads
                        runs.append(copy.copy(run))
                else:
                    runs.append(copy.copy(run))
            else:
                # GPU based run. Need to add the segment lengths
                segments = args.gpu_segment_length
                if len(segments) == 0:
                    segments = [0]
                ocl_vendors = args.ocl_vendor if app_type == 'ocl' else [
                    'none'
                ]
                for ocl_vendor in ocl_vendors:
                    run.ocl_vendor = ocl_vendor if ocl_vendor != 'none' else None
                    for segment in segments:
                        run.gpu_segment_length = segment
                        dirs = args.dir
                        if not dirs:
                            dirs = ['reverse']
                        elif isinstance(dirs, str):
                            dirs = [dirs]
                        for dir in dirs:
                            run.forward = dir == 'forward'
                            runs.append(copy.copy(run))

    # execute the runs
    for run in runs:
        run.execute()
    print(len(runs), 'items completed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
