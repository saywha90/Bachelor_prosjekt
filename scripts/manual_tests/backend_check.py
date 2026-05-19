"""
Quick check that matplotlib's 3D projection works with the current backend.
Run this to verify the visualizer will function before running full demos.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "src"))

import matplotlib
import matplotlib.pyplot as plt

print("Default backend:", matplotlib.get_backend())

plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot([0, 1], [0, 1], [0, 1])
plt.show(block=False)
plt.pause(1)
print("Worked!")
