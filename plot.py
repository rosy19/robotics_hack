"""Simple plot script: read first two columns of points.csv, normalize to [-1,1], and save a plot.

Usage:
	python plot.py              # reads ./points.csv and writes ./points_plot.png
	python plot.py input.csv out.png

The first column is treated as X, the second as Y.
If a column is constant, it will be centered at 0.
"""
from __future__ import annotations

import os
import sys
from typing import Tuple


def normalize_to_minus1_1(arr):
	"""Normalize a 1D numpy array to the range [-1, 1]."""
	import numpy as np

	if arr.size == 0:
		return arr
	mn = arr.min()
	mx = arr.max()
	if mx == mn:
		# constant array -> return zeros
		return np.zeros_like(arr)
	return 2.0 * (arr - mn) / (mx - mn) - 1.0


def read_xy_from_csv(path: str) -> Tuple[list, list]:
	"""Read first two columns from CSV and return (x, y) as numpy arrays."""
	import numpy as np

	data = np.loadtxt(path, delimiter=",", usecols=(0, 1))
	# If there's only one row, ensure shape is (n,)
	if data.ndim == 1:
		x = data[0:1]
		y = data[1:2]
		x = np.atleast_1d(x)
		y = np.atleast_1d(y)
	else:
		x = data[:, 0]
		y = data[:, 1]
	return x, y # type: ignore


def plot_and_save(x, y, out_path: str):
	try:
		import matplotlib.pyplot as plt
	except Exception as e:  # pragma: no cover - runtime
		raise

	plt.figure(figsize=(6, 6))
	# Connect points in sequence with lines and show markers at points
	plt.plot(x, y, marker="o", markersize=4, linestyle="-", color="tab:blue", alpha=0.9)
	plt.axhline(0, color="gray", linewidth=0.7)
	plt.axvline(0, color="gray", linewidth=0.7)
	plt.xlim(-1.0, 1.0)
	plt.ylim(-1.0, 1.0)
	plt.xlabel("X (normalized to [-1,1])")
	plt.ylabel("Y (normalized to [-1,1])")
	plt.title("Points (first two columns of CSV) - normalized")
	plt.grid(True, linestyle="--", alpha=0.3)
	plt.tight_layout()
	plt.savefig(out_path, dpi=150)
	plt.close()


def main(argv=None):
	argv = argv if argv is not None else sys.argv[1:]
	if len(argv) >= 1:
		csv_path = argv[0]
	else:
		csv_path = os.path.join(os.path.dirname(__file__), "points.csv")

	out_path = argv[1] if len(argv) >= 2 else os.path.join(os.path.dirname(__file__), "points_plot3.png")

	if not os.path.exists(csv_path):
		print(f"CSV file not found: {csv_path}")
		return 2

	try:
		x, y = read_xy_from_csv(csv_path)
	except Exception as exc:  # pragma: no cover - runtime
		print(f"Failed to read CSV '{csv_path}': {exc}")
		return 3

	try:
		import numpy as np
	except ImportError:
		print("numpy is required but not installed. Install with: pip install numpy")
		return 4

	# Normalize each series to [-1, 1]
	try:
		x_n = normalize_to_minus1_1(x)
		y_n = normalize_to_minus1_1(y)
	except Exception as exc:  # pragma: no cover - runtime
		print(f"Normalization failed: {exc}")
		return 5

	try:
		plot_and_save(x_n, y_n, out_path)
	except ImportError:
		print("matplotlib is required but not installed. Install with: pip install matplotlib")
		return 6
	except Exception as exc:  # pragma: no cover - runtime
		print(f"Plotting failed: {exc}")
		return 7

	print(f"Saved plot to: {out_path}")
	return 0


if __name__ == "__main__":
	raise SystemExit(main())
