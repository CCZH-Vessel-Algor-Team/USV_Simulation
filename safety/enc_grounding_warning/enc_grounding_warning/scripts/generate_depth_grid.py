"""Generate a depth grid CSV from sim_depth_grid.yaml (mode=flat)."""

from __future__ import annotations

import argparse
import os

import numpy as np

from ..bathymetry import SimGridProvider


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, help="path to sim_depth_grid.yaml")
    parser.add_argument("--output", default="", help="output CSV path")
    ns = parser.parse_args(args)

    provider = SimGridProvider(ns.config)
    output = ns.output
    if not output:
        output = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
            "data",
            "depth_grid.csv",
        )
    os.makedirs(os.path.dirname(output), exist_ok=True)
    np.savetxt(output, provider.depth, delimiter=",", fmt="%.3f")
    print(f"wrote {output} ({provider.depth.shape[0]}x{provider.depth.shape[1]})")


if __name__ == "__main__":
    main()
