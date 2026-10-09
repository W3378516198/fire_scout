#!/usr/bin/env python3
"""Validate bounded, precise local geometry without importing ROS transport."""
import ast
import math
from pathlib import Path

root = Path(__file__).resolve().parents[1]
tree = ast.parse((root / "scripts/fire_scout_light_logger.py").read_text())
function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "near_path_geometry")
scope = {"math": math}
exec(compile(ast.Module(body=[function], type_ignores=[]), "logger-helper", "exec"), scope)
near = scope["near_path_geometry"]
assert near([], None)["near_geometry"] == []
points = [(i * .001, .00017 * math.sin(i / 10), 2.) for i in range(4000)]
result = near(points, points[2000])
assert result["near_index"] == 2000
assert len(result["near_geometry"]) <= 64
assert result["near_geometry"][0][0] < 2.
assert result["near_geometry"][-1][0] >= 3.5 - .002
assert any(p[1] != 0 for p in result["near_geometry"])
assert len(near([(0., 0., 2.)], None)["near_geometry"]) == 1
small = near([(0., .00017, 2.), (.04, .00018, 2.), (1., 0., 2.)], None)
assert small["near_geometry"][0][1] == .00017
print("light_logger_geometry_test: PASS bounded local window and 5-decimal precision")
