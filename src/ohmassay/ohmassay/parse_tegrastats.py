#!/usr/bin/python3
# Copyright (c) 2021
# Commonwealth Scientific and Industrial Research Organisation (CSIRO)
# ABN 41 687 119 230
#
# Author: Kazys Stepanas
"""Ohm assay tegrastats parsing utility.

Parses the input files assuming results from tegrastats and extracts the following data to CSV file:
- Total CPU load
- GPU load
- RAM usage
"""

import argparse
import glob
import os.path
import re
import sys

cpu_load_total_expr = re.compile(r'CPU \[(([0-9]{1,3}%@[0-9]+,?)+)\]')
cpu_load_groups_expr = re.compile(r'([0-9]{1,3})%@([0-9]+)')

gpu_load_expr = re.compile(r'GR3D_FREQ ([0-9]{1,3})%')

ram_expr = re.compile(r'RAM ([0-9]+)/([0-9]+)([KMG]?)B')


def parse_args(cmd_args=None, description=None):
    """Parse command line arguments."""
    if description is None:
        description = 'ohm tegrastats utility'
    parser = argparse.ArgumentParser(description=description, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('input', type=str, nargs='+', default=None, help='Input files (glob).')
    # parser.add_argument('-o', '--output', type=str, default=None, help='Output file name (CSV).')
    parser.add_argument('--interval', type=float, default=1.0,
                        help='Sample interval expected for the files. Affects timestamping.')

    args = parser.parse_args(cmd_args)

    return args


def parse_cpu(line):
    """Parse CPU load and frequency from line."""
    cpu_details = cpu_load_total_expr.search(line)
    if not cpu_details:
        raise RuntimeError('Unable to parse CPU from line:\n"{}"'.format(line))
    cpu_details = cpu_details.group(1)
    cpu_count = 0
    cpu_percentage_total = 0
    cpu_frequency_total = 0
    for cpu_item in cpu_load_groups_expr.finditer(cpu_details):
        cpu_count += 1
        cpu_percentage_total += int(cpu_item.group(1))
        cpu_frequency_total += int(cpu_item.group(2))
    cpu_hz_average = cpu_frequency_total / cpu_count
    return cpu_percentage_total, cpu_hz_average


def parse_gpu(line):
    """Parse GPU load from line."""
    gpu_details = gpu_load_expr.search(line)
    if not gpu_details:
        raise RuntimeError('Unable to parse GPU from line:\n"{}"'.format(line))
    return gpu_details.group(1)


def parse_ram(line):
    """Parse RAM usage from line."""
    ram_details = ram_expr.search(line)
    if not ram_details:
        raise RuntimeError('Unable to parse RAM from line:\n"{}"'.format(line))
    return ram_details.group(1)


def parse_file(file_path, csv_path, sample_interval=1.0):
    """Parse the given tegrastats file and convert to csv_path."""
    item_number = 0
    with open(file_path, 'r') as data_file:
        with open(csv_path, 'w') as csv_file:
            csv_file.write('time,cpu,cpu_hz,gpu,ram\n')
            for line in data_file.readlines():
                timestamp = item_number * sample_interval
                cpu_load, cpu_hz = parse_cpu(line)
                gpu_load = parse_gpu(line)
                ram_usage = parse_ram(line)
                csv_file.write('{},{},{},{},{}\n'.format(timestamp, cpu_load, cpu_hz, gpu_load, ram_usage))
                item_number += 1


def main(cmd_args=None):
    """Main function."""
    args = parse_args(cmd_args, __doc__)

    data_files = []
    for input in args.input:
        glob_items = glob.glob(input)
        data_files += glob_items

    for file_path in data_files:
        try:
            csv_path = os.path.splitext(file_path)[0]
            csv_path += '.csv'
            parse_file(file_path, csv_path)
        except RuntimeError:
            print('Skipping {} - failed parsing'.format(file_path))


if __name__ == '__main__':
    sys.exit(main())
