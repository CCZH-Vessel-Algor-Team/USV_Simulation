#!/usr/bin/env python3
"""test_generate_and_check.py

Simple test harness that:
 - runs `colcon build --packages-select usv_sim_full`
 - sources the workspace and runs session_manager to produce final_robot.urdf
 - parses the generated URDF and asserts that the base_link inertial mass equals the expected value

Usage:
  python3 test_generate_and_check.py --expected-mass 180.0

This script is intended to be run from the workspace root:
  /home/cczh/USV_ROS/src/USV_Simulation
"""
import argparse
import glob
import os
import subprocess
import sys
import xml.etree.ElementTree as ET


def find_latest_final_urdf(base_dir="src/usv_sim_full/logs"):
    pattern = os.path.join(base_dir, 'session_*', 'final_robot.urdf')
    files = glob.glob(pattern)
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def run_cmd(cmd, env=None, shell=True):
    print(f"Running: {cmd}")
    result = subprocess.run(cmd, shell=shell, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    print(result.stdout)
    return result.returncode


def build_package():
    return run_cmd('colcon build --packages-select usv_sim_full')


def generate_urdf():
    # Source install/setup.bash then run session_manager
    cmd = "bash -lc 'source install/setup.bash && python3 src/usv_sim_full/scripts/session_manager.py --config-path src/usv_sim_full/config/full_config.yaml'"
    return run_cmd(cmd)


def parse_and_check(urdf_path, expected_mass_str):
    try:
        tree = ET.parse(urdf_path)
        root = tree.getroot()
    except ET.ParseError as e:
        print(f"ERROR: Failed to parse URDF: {e}")
        return False

    # Find link with name 'wamv/base_link'
    for link in root.findall('link'):
        if link.get('name') == 'wamv/base_link':
            inertial = link.find('inertial')
            if inertial is None:
                print('No <inertial> found under wamv/base_link')
                return False
            mass = inertial.find('mass')
            if mass is None:
                print('No <mass> element found under inertial')
                return False
            val = mass.get('value')
            print(f'Found mass value: {val}')
            return val == expected_mass_str
    print('Link wamv/base_link not found in URDF')
    return False


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument('--expected-mass', default='180.0', help='Expected mass value (string exact match)')
    args = p.parse_args(argv)

    # Step 1: build
    rc = build_package()
    if rc != 0:
        print('Build failed')
        sys.exit(2)

    # Step 2: generate URDF
    rc = generate_urdf()
    if rc != 0:
        print('session_manager failed to generate URDF')
        sys.exit(3)

    # Step 3: locate URDF
    urdf = find_latest_final_urdf()
    if not urdf:
        print('final_robot.urdf not found')
        sys.exit(4)
    print(f'Using URDF: {urdf}')

    # Step 4: parse and check
    ok = parse_and_check(urdf, args.expected_mass)
    if ok:
        print('TEST PASSED: expected mass found in final_robot.urdf')
        sys.exit(0)
    else:
        print('TEST FAILED: expected mass not found')
        sys.exit(5)


if __name__ == '__main__':
    main()
