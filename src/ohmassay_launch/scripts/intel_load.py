#!/usr/bin/python3
#
# Copyright (c) 2021
# Commonwealth Scientific and Industrial Research Organisation (CSIRO)
# ABN 41 687 119 230
#
# Author: Kazys Stepanas
"""Display intel GPU and CPU load using intel_gpu_top."""

import argparse
import datetime
import os
import shutil
import subprocess
import sys
import time

import psutil


def checkSudo() -> bool:
    """Check for sudo privileges."""
    return os.geteuid() == 0


def parseGpuTop(top_out):
    # First call has header or could be an error
    split = top_out.split()
    return split[0], split[1]


def parse_args(cmd_args=None, description=None):
    """Parse command line arguments."""
    if description is None:
        description = 'ohm tegrastats utility'
    parser = argparse.ArgumentParser(
        description=description, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('mode',
                        type=str,
                        default='cpu',
                        choices=['cpu', 'gpu'],
                        help='Logging mode.')
    parser.add_argument('--output',
                        '-o',
                        type=str,
                        default=None,
                        help='Output to file.')

    args = parser.parse_args(cmd_args)

    return args


def logWithGpu(out):
    """Log with GPU stats using intel_gpu_top."""
    gpu_top_exe = shutil.which('intel_gpu_top')
    if gpu_top_exe is None:
        sys.stderr.write('Unable to find intel_gpu_top\n')
        return 1
    if not checkSudo():
        sys.stderr.write('sudo required for running intel_gpu_top\n')
        return 2

    gpu_top = subprocess.Popen(['intel_gpu_top', '-o', '-'],
                               stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT)
    out.write('time      \tCPU%\tGPU%\n')
    done = False
    first_top_result = True
    while not done:
        try:
            top_out = gpu_top.stdout.readline()
            if top_out:
                top_out = top_out.decode('utf-8')
                # First top line is headings. Ignore.
                if not first_top_result:
                    # Grab CPU load at this time.
                    cpu_load = sum(psutil.cpu_percent(percpu=True))
                    # Extract GPU load
                    try:
                        time, gpu_load = parseGpuTop(top_out)
                        out.write('{:<10.2f}\t{}\t{}\n'.format(
                            float(time), cpu_load, gpu_load))
                        out.flush()
                    except ValueError:
                        done = True
                        print(top_out)
                else:
                    first_top_result = False
        except KeyboardInterrupt:
            done = True
    if gpu_top:
        gpu_top.kill()
    return 0


def sleep():
    """Sleep until the next integer second."""
    now = datetime.datetime.today()
    future = now + datetime.timedelta(seconds=1.0)
    time.sleep((future - now).total_seconds())


def logCpu(out):
    """Log CPU stats only."""
    out.write('time      \tCPU%\n')
    done = False
    timestamp = 1.0
    time_start = datetime.datetime.today()
    while not done:
        try:
            now = datetime.datetime.today()
            cpu_load = sum(psutil.cpu_percent(percpu=True))
            timestamp = (now - time_start).total_seconds()
            out.write('{:<10.2f}\t{}\n'.format(timestamp, cpu_load))
            out.flush()
            sleep()
            timestamp += 1.0
        except KeyboardInterrupt:
            done = True


def main(cmd_args=None):
    """Run the program."""
    args = parse_args(cmd_args, __doc__)
    out = sys.stdout
    if args.output:
        out = open(args.output, 'w')
    if args.mode == 'gpu':
        return logWithGpu(out)
    logCpu(out)
    if args.output:
        out.close()


if __name__ == '__main__':
    sys.exit(main())
