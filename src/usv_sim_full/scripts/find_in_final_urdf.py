#!/usr/bin/env python3
"""find_in_final_urdf.py

Search for a fixed value inside a generated final_robot.urdf and report where it appears.

Usage:
  python3 find_in_final_urdf.py --value 180.0 [--file PATH]

If --file is omitted the script will pick the newest
`src/usv_sim_full/logs/session_*/final_robot.urdf` file.
"""
import argparse
import glob
import os
import re
import sys
import xml.etree.ElementTree as ET


def find_latest_final_urdf(base_dir="src/usv_sim_full/logs"):
    pattern = os.path.join(base_dir, 'session_*', 'final_robot.urdf')
    files = glob.glob(pattern)
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def build_parent_map(root):
    parent = {}
    for p in root.iter():
        for c in list(p):
            parent[c] = p
    return parent


def elem_path(elem, parent_map):
    parts = []
    e = elem
    while e is not None:
        tag = e.tag
        # include name attribute if present for clarity
        if 'name' in e.attrib:
            parts.append(f"{tag}[@name=\"{e.attrib['name']}\"]")
        else:
            parts.append(tag)
        e = parent_map.get(e)
    return '/' + '/'.join(reversed(parts))


def find_matches_by_attr(root, value_str):
    matches = []
    parent_map = build_parent_map(root)
    for elem in root.iter():
        for attr, val in elem.attrib.items():
            if val == value_str:
                matches.append((elem, attr, val, parent_map))
    return matches


def find_raw_lines(filepath, value_str):
    lines = []
    with open(filepath, 'r', encoding='utf-8') as f:
        for i, line in enumerate(f, start=1):
            if value_str in line:
                lines.append((i, line.rstrip('\n')))
    return lines


def main(argv=None):
    p = argparse.ArgumentParser(description='Find a value in final_robot.urdf and report locations')
    p.add_argument('--value', '-v', required=True, help='Value to search for (string exact match)')
    p.add_argument('--file', '-f', help='Path to final_robot.urdf (optional)')
    args = p.parse_args(argv)

    value = args.value
    urdf_file = args.file or find_latest_final_urdf()
    if not urdf_file:
        print('ERROR: final_robot.urdf not found under src/usv_sim_full/logs/session_*')
        sys.exit(2)

    print(f"Using URDF file: {urdf_file}")
    print(f"Searching for exact value: {value}\n")

    # raw line matches
    raw_lines = find_raw_lines(urdf_file, value)
    if raw_lines:
        print('Raw text matches (line: text):')
        for ln, text in raw_lines:
            print(f'  {ln}: {text}')
    else:
        print('No raw text matches found.')

    # parse xml and find attribute-equality matches
    try:
        tree = ET.parse(urdf_file)
        root = tree.getroot()
    except ET.ParseError as e:
        print(f'ERROR: Failed to parse URDF as XML: {e}')
        sys.exit(3)

    matches = find_matches_by_attr(root, value)
    if matches:
        print('\nAttribute exact-match elements:')
        for elem, attr, val, parent_map in matches:
            path = elem_path(elem, parent_map)
            # show a short xml fragment
            attribs = ' '.join([f'{k}="{v}"' for k, v in elem.attrib.items()])
            snippet = f'<{elem.tag} {attribs}>'
            print(f'  Element path: {path}')
            print(f'    tag: <{elem.tag}>   attribute: {attr}="{val}"')
            print(f'    snippet: {snippet}\n')
    else:
        print('\nNo attribute matches where attribute value == search value.')

    # attempt to correlate raw line matches with elements by searching for attr patterns
    if raw_lines and matches:
        print('Correlation (trying to map raw lines -> attribute matches):')
        with open(urdf_file, 'r', encoding='utf-8') as f:
            text = f.read()
        for elem, attr, val, parent_map in matches:
            tag = elem.tag
            pattern = re.compile(rf"<\s*{re.escape(tag)}[^>]*\b{re.escape(attr)}\s*=\s*\"{re.escape(val)}\"", re.IGNORECASE)
            m = pattern.search(text)
            if m:
                # determine line number
                prefix = text[:m.start()]
                line_no = prefix.count('\n') + 1
                print(f'  Element <{tag} {attr}="{val}"> appears near line {line_no}')
            else:
                print(f'  Could not find text occurrence for <{tag} {attr}="{val}">')


if __name__ == '__main__':
    main()
