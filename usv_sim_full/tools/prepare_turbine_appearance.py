#!/usr/bin/env python3
"""Bake appearance-only turbine variants; never alter the supplied source model."""
import argparse
import json
from pathlib import Path

from usv_sim_full.wind_turbine_appearance import generate


if __name__ == '__main__':
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path,
                        default=package/'worlds/models/iea15_offshore_turbine')
    parser.add_argument('--output-root', type=Path, default=package/'worlds/models')
    args = parser.parse_args()
    report = generate(args.source, args.output_root)
    print(json.dumps(report['statistics'], indent=2))
