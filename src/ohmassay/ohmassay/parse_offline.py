#!/usr/bin/python3
# Copyright (c) 2021
# Commonwealth Scientific and Industrial Research Organisation (CSIRO)
# ABN 41 687 119 230
#
# Author: Kazys Stepanas
"""Ohm results tabulation utility.
"""
import argparse
import enum
import glob
import os.path
import re
import sys

content_res_expr = re.compile(
    r'^Map resolution: ([0-9\.]+)$', re.MULTILINE)
content_mean_expr = re.compile(
    r'^Voxel mean position: (on|off)$', re.MULTILINE)
content_ndt_expr = re.compile(
    r'^NDT mode: (om|tm)$', re.MULTILINE)
content_seg_len_expr = re.compile(
    r'^Gpu max ray segment: ([0-9]+)B?$', re.MULTILINE)
content_tsdf_mode_expr = re.compile(
    r'^TSDF mode: ([a-zA-Z_0-9]+)$', re.MULTILINE)
content_threads_expr = re.compile(
    r'^Threads: ([0-9]+)$', re.MULTILINE)
filename_info = re.compile(
    r'(cpu|cuda|ocl|oct|vox)(-(intel|nvidia))?.*\.txt')

# Extraction expressions corresponding to DataMode values.
content_rays_per_second_expr = re.compile(
    r'^Average samples/sec: ([0-9,]+)$', re.MULTILINE)
content_rtf_expr = re.compile(
    r'^Realtime Factor: ([0-9\.]+)$', re.MULTILINE)
content_rtf_inverse_expr = re.compile(
    r'^RTF inverse: ([0-9\.]+)$', re.MULTILINE)
content_time_expr = re.compile(
    r'^Total processing time: ([0-9\.,]+)s$', re.MULTILINE)
content_sample_count_expr = re.compile(
    r'^Sample count: ([0-9,]+)$', re.MULTILINE)


class DataMode(enum.Enum):
    """Data extraction mode.

    RPS: rays per second
    RTF: real-time factor
    RTFI: real-time factor inverse (1/RTF)
    SAMPLES: sample count processed - more for real time deviations
    TIME: processing time
    """
    RPS = 0
    RTF = 1
    RTFI = 2
    SAMPLES = 3
    TIME = 4


def get_info_item(info, expr, group, default=None):
    first = True
    duplicates = False
    for item in expr.finditer(info):
        if first:
            return item.group(group)
        else:
            duplicates = True
            break
        first = False
    if not duplicates:
        if default is not None:
            return default
        raise RuntimeError('Unable to find content for expression', expr)
    raise RuntimeError('Multiple content items for expression', expr)


def pull_timing(file_path, table, data_mode):
    with open(file_path, 'r') as data_file:
        try:
            filename = os.path.basename(file_path)
            content = data_file.read()
            # resolution = float(get_info_item(content, content_res_expr, 1))
            mean = get_info_item(content, content_mean_expr, 1, False)
            mean = True if mean == 'on' else False
            ndt = get_info_item(content, content_ndt_expr, 1, '')
            segment_length = int(get_info_item(content, content_seg_len_expr, 1, default=0))
            threads = int(get_info_item(content, content_threads_expr, 1, default=0))
            tsdf_mode = get_info_item(content, content_tsdf_mode_expr, 1, default='')
            run_type = get_info_item(filename, filename_info, 1)
            ocl_gpu_type = get_info_item(filename, filename_info, 3, '')

            metric_value = None
            if data_mode == DataMode.RPS:
                # Ray's per second is an integer but may contain digit separators. We assume Australian/US locale.
                # Otherwise it gets really complicated on the decimal values.
                rps_str = get_info_item(content, content_rays_per_second_expr, 1)
                rps_str = rps_str.replace(',', '')
                metric_value = int(rps_str)
            elif data_mode == DataMode.RTF:
                metric_value = float(get_info_item(content, content_rtf_expr, 1))
            elif data_mode == DataMode.RTFI:
                metric_value = float(get_info_item(content, content_rtf_inverse_expr, 1))
            elif data_mode == DataMode.SAMPLES:
                # As with rays/second, we have an integer with digit separators. Only handle ','
                samples_str = get_info_item(content, content_sample_count_expr, 1)
                samples_str = samples_str.replace(',', '')
                metric_value = int(samples_str)
            elif data_mode == DataMode.TIME:
                # Time values can have digit separators (commas) when we run *very* long
                time_str = get_info_item(content, content_time_expr, 1)
                time_str = time_str.replace(',', '')
                metric_value = float(time_str)

            occupancy_type = 'occ'
            if mean:
                occupancy_type = 'mean'
            if ndt == 'om':
                occupancy_type = 'ndt'
            elif ndt == 'tm':
                occupancy_type = 'ndt-tm'
            if tsdf_mode:
                occupancy_type = 'tsdf'

            if ocl_gpu_type:
                run_type = '{} {}'.format(run_type, ocl_gpu_type)

            if segment_length:
                run_type = '{} s{:02d}m'.format(run_type, segment_length)

            if tsdf_mode and 'vox' in filename:
                run_type = '{} {}'.format(run_type, tsdf_mode)

            if threads > 0:
                run_type = '{} mt{}'.format(run_type, threads)

            if run_type not in table:
                table[run_type] = {}

            table[run_type][occupancy_type] = metric_value
        except RuntimeError:
            print('Skipping {} - failed extraction'.format(file_path))


def parse_args(cmd_args=None, description=None):
    """Parse command line arguments."""
    if description is None:
        description = 'ohm timing utility'
    parser = argparse.ArgumentParser(description=description, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('-o', '--output', type=str, default=None, help='Output file name (CSV).')
    parser.add_argument('--mode', type=str, default='time',
                        choices=['rps', 'rtf', 'rtfi', 'samples', 'time'],
                        help='Statistic extraction mode: rps=rays/second, rtf=real-time factor, rtfi=1/rtf, '
                        'samples=sample count, time=processing time')

    args = parser.parse_args(cmd_args)

    # Set default output file name by the data mode.
    if args.output is None:
        data_mode = DataMode[args.mode.upper()]
        if data_mode == DataMode.RPS:
            args.output = 'rays_per_second.csv'
        elif data_mode == DataMode.RTF:
            args.output = 'real_time_factor.csv'
        elif data_mode == DataMode.RTFI:
            args.output = 'real_time_factor_inverse.csv'
        elif data_mode == DataMode.SAMPLES:
            args.output = 'sample_count.csv'
        elif data_mode == DataMode.TIME:
            args.output = 'timing.csv'
        else:
            raise RuntimeError('Unsupported data mode: {}'.format(data_mode))

    return args


def main(cmd_args=None):
    """Main function."""
    args = parse_args(cmd_args, __doc__)
    table = {}
    data_files = glob.glob('*.txt')

    data_mode = DataMode[args.mode.upper()]

    for data_file in data_files:
        pull_timing(data_file, table, data_mode)

    delimit = ','
    with open(args.output, 'w') as out:
        def write_result(out, name, data_line):
            if name in data_line:
                out.write(str(data_line[name]))
            else:
                out.write('')

        header = 'Run Type{0}occ{0}mean{0}ndt{0}ndt-tm{0}tsdf\n'.format(delimit)
        out.write(header)
        keys = list(table.keys())
        keys.sort()
        for run_type in keys:
            run_data = table[run_type]
            out.write(run_type)
            out.write(delimit)
            write_result(out, 'occ', run_data)
            out.write(delimit)
            write_result(out, 'mean', run_data)
            out.write(delimit)
            write_result(out, 'ndt', run_data)
            out.write(delimit)
            write_result(out, 'ndt-tm', run_data)
            out.write(delimit)
            write_result(out, 'tsdf', run_data)
            out.write('\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
