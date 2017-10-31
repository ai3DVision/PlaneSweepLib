#!/usr/bin/env python
"""Script to transform binary depth data exported using the PlaneSweep library
to numpy array"""
import argparse
import os
import sys

import numpy as np

def read_data(f):
    version = np.fromfile(f, count=1, dtype=np.uint8)
    endian = np.fromfile(f, count=1, dtype=np.uint8)
    uint_size = np.fromfile(f, count=1, dtype=np.uint8)

    t_size = np.fromfile(f, count=1, dtype=np.uint32)
    u_size = np.fromfile(f, count=1, dtype=np.uint32)
    P = np.fromfile(f, count=3*4, dtype=np.float64).reshape(3, 4)

    width = np.fromfile(f, count=1, dtype=np.uint32)
    height = np.fromfile(f, count=1, dtype=np.uint32)
    D = np.fromfile(f, count=width*height, dtype=np.float32)

    return D, height[0], width[0]


def main(argv):
    parser = argparse.ArgumentParser(
        description=("Transform binary depth data to numpy data")
    )
    parser.add_argument(
        "input_directory",
        help="Path to the directory containing the file to be transformed"
    )
    parser.add_argument(
        "output_directory",
        help="Path to the directory where tha data will be saved"
    )
    parser.add_argument(
        "ref_idx",
        type=int,
        help="Reference index"
    )

    args = parser.parse_args(argv)

    f = open(os.path.join(args.input_directory, "depth_%d.bin" % (args.ref_idx,)), "rb")
    D, height, width = read_data(f)

    f.close()
    np.save(os.path.join(args.output_directory, "depth_%d" %(args.ref_idx,)), D.reshape(height, width))

if __name__ == "__main__":
    main(sys.argv[1:])
