#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import sys
import argparse


def align_and_clean_spacing_graphs(data={}):
    for k, v in data.items():
        data[k] = np.array([float(x) for x in v])

        for i in range(len(data[k])):
            if data[k][i] > 900 or data[k][i] < 100:
                data[k][i] = np.NaN
            else:
                data[k][i] = data[k][i] - 320


def align_and_clean_lash_graphs(data={}):
    for k, v in data.items():
        data[k] = np.array([float(x) for x in v])

        for i in range(len(data[k])):
            if data[k][i] > 20:
                data[k][i] = np.NaN


def extract_data_from_files(files):
    spacing_data = {}
    lash_data = {}

    for filename in files:
        with open(filename) as fil:
            lines = [
                x.strip('\r\n %').split(',') for x
                in fil.readlines()
                if x.startswith('%')
            ]

        spacing = [int(x[1]) for x in lines]
        lash = [int(x[0]) for x in lines]
        spacing_data[filename] = spacing
        lash_data[filename] = lash

    return spacing_data, lash_data


def build_spacing_graph(data):
    plots = []

    for filename, spacing in data.items():

        sans_gap = np.array([x for x in spacing if not np.isnan(x)])

        lmin = np.min(sans_gap)
        lmax = np.max(sans_gap)
        avg = np.average(sans_gap).round()
        print("min/max/average spacing for {}: {} | {} | {}".format(
            filename, lmin, lmax, avg
        ))

        plots.append((filename, sans_gap))

    max_len = min((len(x[1]) for x in plots))
    adj_x = range(max_len)
    lines = []
    for filename, data in plots:
        line, *_ = plt.plot(adj_x, data[:max_len], label=filename)
        lines.append(line)

    axes = plt.gca()
    axes.set_ylim([-5, 20])
    plt.legend(handles=lines)


def build_lash_graph(data):
    plots = []
    for filename, lash in data.items():

        sans_gap = [x for x in lash if x > 0 and x < 20]

        lmin = np.min(sans_gap)
        lmax = np.max(sans_gap)
        avg = np.average(sans_gap).round()
        print("min/max/average lash for {}: {} | {} | {}".format(
            filename, lmin, lmax, avg
        ))

        plots.append((filename, sans_gap))

    max_len = min((len(x[1]) for x in plots))
    adj_x = range(max_len)
    lines = []
    for filename, data in plots:
        line, *_ = plt.plot(adj_x, data[:max_len], label=filename)
        lines.append(line)

    axes = plt.gca()
    axes.set_ylim([-5, 20])
    plt.legend(handles=lines)


def do_lash_analysis(data):
    align_and_clean_lash_graphs(data)
    build_lash_graph(data)


def do_spacing_analysis(data):
    align_and_clean_spacing_graphs(data)
    build_spacing_graph(data)


def do_analysis(args):
    spacing_data, lash_data = extract_data_from_files(args.files)
    if args.spacing:
        do_spacing_analysis(spacing_data)
    elif args.lash:
        do_lash_analysis(lash_data)

    plt.show()


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('files', nargs='*')
    group = parser.add_argument_group('Analysis Type')
    group_ex = group.add_mutually_exclusive_group()
    group_ex.add_argument("--lash", action="store_true", default=False)
    group_ex.add_argument("--spacing", action="store_true", default=False)

    args = parser.parse_args()

    do_analysis(args)

if __name__ == '__main__':
    main()
