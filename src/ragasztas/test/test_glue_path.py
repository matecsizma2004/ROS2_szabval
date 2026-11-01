import os

import pytest
import yaml

from ragasztas.glue_path import (cell_center, find_strokes, grid_to_pixels,
                                 image_to_grid, parse_grid)

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'glue_map.yaml')


def load_grid_rows():
    with open(CONFIG, encoding='utf-8') as f:
        return yaml.safe_load(f)['/**']['ros__parameters']['grid']


def test_config_grid_is_13x10():
    grid = parse_grid(load_grid_rows())
    assert len(grid) == 13
    assert all(len(row) == 10 for row in grid)


def test_strokes_of_assignment_map():
    strokes = find_strokes(parse_grid(load_grid_rows()))
    assert strokes == [
        (0, 0, 2), (0, 5, 7), (1, 2, 4), (3, 8, 9), (4, 2, 4),
        (6, 4, 9), (8, 0, 2), (10, 2, 4), (12, 5, 9),
    ]


def test_image_roundtrip():
    grid = parse_grid(load_grid_rows())
    data = grid_to_pixels(grid)
    assert len(data) == 130
    assert image_to_grid(data, 10, 13) == grid


def test_single_cells_are_skipped():
    assert find_strokes(parse_grid(['1010011'])) == [(0, 5, 6)]


def test_invalid_grid_rejected():
    with pytest.raises(ValueError):
        parse_grid(['101', '10'])
    with pytest.raises(ValueError):
        parse_grid(['10x'])


def test_cell_center():
    x, y, z = cell_center(0, 0, 0.82, 0.10, 0.005, 0.02)
    assert (x, y, z) == pytest.approx((0.81, 0.09, 0.005))
    x, y, _ = cell_center(12, 9, 0.82, 0.10, 0.0, 0.02)
    assert (x, y) == pytest.approx((0.57, -0.09))
