import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from Tarok_Dymensions import Tarok_Dymensions_Class
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np


def plot_torso():
    dim = Tarok_Dymensions_Class()

    L = dim.Torso_Lenght  # 0.479 m
    W = dim.Torso_Width   # 0.3179 m
    H = dim.Torso_Height  # 0.09919 m

    fig = plt.figure(figsize=(14, 6))
    fig.suptitle("TAROK - Torso Visualization", fontsize=15, fontweight='bold')

    # ── 2D Top View ──────────────────────────────────────────────────────────
    ax1 = fig.add_subplot(1, 2, 1)
    ax1.set_title("Top View (Length × Width)")

    torso_rect = patches.Rectangle(
        (-L / 2, -W / 2), L, W,
        linewidth=2, edgecolor='steelblue', facecolor='lightsteelblue', label='Torso'
    )
    ax1.add_patch(torso_rect)

    # Shoulder positions
    shoulders = dim.Shoulder_Positions()
    labels = ['Front Left', 'Front Right', 'Hind Left', 'Hind Right']
    colors = ['green', 'limegreen', 'red', 'tomato']

    for pos, label, color in zip(shoulders, labels, colors):
        ax1.plot(pos[0], pos[1], 'o', markersize=10, color=color, label=label)
        ax1.annotate(label, (pos[0], pos[1]),
                     textcoords="offset points", xytext=(6, 6), fontsize=8)

    # Dimension arrows
    ax1.annotate('', xy=(L/2, -W/2 - 0.03), xytext=(-L/2, -W/2 - 0.03),
                 arrowprops=dict(arrowstyle='<->', color='black'))
    ax1.text(0, -W/2 - 0.05, f'Length: {L*100:.1f} cm', ha='center', fontsize=9)

    ax1.annotate('', xy=(L/2 + 0.03, W/2), xytext=(L/2 + 0.03, -W/2),
                 arrowprops=dict(arrowstyle='<->', color='black'))
    ax1.text(L/2 + 0.06, 0, f'Width:\n{W*100:.1f} cm', ha='left', fontsize=9, va='center')

    ax1.set_xlim(-L/2 - 0.12, L/2 + 0.18)
    ax1.set_ylim(-W/2 - 0.12, W/2 + 0.1)
    ax1.set_aspect('equal')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.axhline(0, color='gray', linewidth=0.5, linestyle='--')
    ax1.axvline(0, color='gray', linewidth=0.5, linestyle='--')
    ax1.legend(loc='upper left', fontsize=8)
    ax1.grid(True, linestyle='--', alpha=0.4)

    # ── 3D View ───────────────────────────────────────────────────────────────
    ax2 = fig.add_subplot(1, 2, 2, projection='3d')
    ax2.set_title("3D View")

    # Define the 8 corners of the box
    x = [-L/2, L/2]
    y = [-W/2, W/2]
    z_offset = dim.Upper_Leg + dim.Lower_Leg  # lifts torso by full leg length
    z = [-H/2 + z_offset, H/2 + z_offset]

    vertices = [
        [x[0], y[0], z[0]], [x[1], y[0], z[0]],
        [x[1], y[1], z[0]], [x[0], y[1], z[0]],
        [x[0], y[0], z[1]], [x[1], y[0], z[1]],
        [x[1], y[1], z[1]], [x[0], y[1], z[1]],
    ]

    faces = [
        [vertices[0], vertices[1], vertices[2], vertices[3]],  # bottom
        [vertices[4], vertices[5], vertices[6], vertices[7]],  # top
        [vertices[0], vertices[1], vertices[5], vertices[4]],  # front
        [vertices[2], vertices[3], vertices[7], vertices[6]],  # back
        [vertices[0], vertices[3], vertices[7], vertices[4]],  # left
        [vertices[1], vertices[2], vertices[6], vertices[5]],  # right
    ]

    box = Poly3DCollection(faces, alpha=0.35, facecolor='steelblue', edgecolor='navy', linewidth=0.8)
    ax2.add_collection3d(box)

    # Shoulder positions in 3D (on top of torso)
    shoulder_positions_3d = [
        [ L/2, -W/2, z_offset],
        [ L/2,  W/2, z_offset],
        [-L/2, -W/2, z_offset],
        [-L/2,  W/2, z_offset],
    ]
    s_colors = ['green', 'limegreen', 'red', 'tomato']
    s_labels = ['FL', 'FR', 'HL', 'HR']

    for pos, color, label in zip(shoulder_positions_3d, s_colors, s_labels):
        ax2.scatter(*pos, color=color, s=60, zorder=5)
        ax2.text(pos[0], pos[1], pos[2] + 0.01, label, fontsize=8, color=color)

    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_zlabel('Z (m)')
    ax2.set_xlim(-L/2 - 0.05, L/2 + 0.05)
    ax2.set_ylim(-W/2 - 0.05, W/2 + 0.05)
    ax2.set_zlim(0, z_offset + H)

    # Stats text
    stats = (f"Length: {L*100:.2f} cm\n"
             f"Width:  {W*100:.2f} cm\n"
             f"Height: {H*100:.2f} cm\n"
             f"Weight: {dim.Tarok_Weight:.2f} kg")
    ax2.text2D(0.02, 0.97, stats, transform=ax2.transAxes,
               fontsize=8, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    plt.tight_layout()
    # plt.savefig("Torso_Visualization.png", dpi=150, bbox_inches='tight')
    plt.show()
    print("Plot saved as Torso_Visualization.png")


if __name__ == "__main__":
    plot_torso()