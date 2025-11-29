#!/usr/bin/env python3
"""
Utility script to dump YOLO class names into JSON/CSV files.

Usage (from repository root):
    python final_project/scripts/export_yolo_class_names.py \
        --model final_project/final_project_ws/models/yolo11x.pt

By default, outputs are stored next to this script as yolo_class_names.json/csv.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Dict, Iterable, List, Sequence

from ultralytics import YOLO


def parse_args() -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    default_model = (
        script_dir.parent / 'final_project_ws' / 'models' / 'yolo11x.pt'
    )
    parser = argparse.ArgumentParser(
        description='Export YOLO class names to JSON and CSV files.'
    )
    parser.add_argument(
        '--model',
        type=Path,
        default=default_model,
        help='Path to a YOLO model checkpoint (default: %(default)s).',
    )
    parser.add_argument(
        '--json',
        type=Path,
        default=script_dir / 'yolo_class_names.json',
        help='Output JSON path (default: %(default)s).',
    )
    parser.add_argument(
        '--csv',
        type=Path,
        default=script_dir / 'yolo_class_names.csv',
        help='Output CSV path (default: %(default)s).',
    )
    return parser.parse_args()


def normalize_names(raw_names: Sequence | Dict[int, str]) -> List[str]:
    if isinstance(raw_names, dict):
        # YOLO stores names in dict keyed by class index.
        return [raw_names[k] for k in sorted(raw_names.keys())]
    return list(raw_names)


def write_json(path: Path, names: Sequence[str]) -> None:
    with path.open('w', encoding='utf-8') as fp:
        json.dump({'classes': list(names)}, fp, indent=2)


def write_csv(path: Path, names: Sequence[str]) -> None:
    with path.open('w', encoding='utf-8', newline='') as fp:
        writer = csv.writer(fp)
        writer.writerow(['id', 'name'])
        for idx, name in enumerate(names):
            writer.writerow([idx, name])


def main() -> None:
    args = parse_args()
    model_path = args.model.expanduser().resolve()
    if not model_path.exists():
        raise FileNotFoundError(f'Model checkpoint not found: {model_path}')

    model = YOLO(str(model_path))
    names = normalize_names(model.names)
    print(f'Loaded {len(names)} classes from {model_path}:')
    for idx, name in enumerate(names):
        print(f'{idx:3d}: {name}')

    write_json(args.json, names)
    write_csv(args.csv, names)
    print(f'JSON written to: {args.json}')
    print(f'CSV written to: {args.csv}')


if __name__ == '__main__':
    main()
