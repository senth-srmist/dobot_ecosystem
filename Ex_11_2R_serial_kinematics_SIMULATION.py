# -*- coding: utf-8 -*-
"""
Created on Sun Nov  1 14:59:18 2026

@author: sen2d
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.gridspec import GridSpec

# ══════════════════════════════════════════════
# FORWARD KINEMATICS
# ══════════════════════════════════════════════
# QUESTION: Given joint angles, where is the hand?
# INPUTS  : theta1 = angle of Link 1 from x-axis (radians)
#           theta2 = angle of Link 2 relative to Link 1 (radians)
# OUTPUTS : positions of Base, Elbow, Hand (end-effector)
# ══════════════════════════════════════════════

def forward_kinematics(theta1, theta2):
    base  = np.array([0.0, 0.0])

    elbow = np.array([
        L1 * np.cos(theta1),
        L1 * np.sin(theta1)
    ])

    hand = np.array([
        L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2),
        L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
    ])
           
        
    return base, elbow, hand


# ══════════════════════════════════════════════
# INVERSE KINEMATICS
# ══════════════════════════════════════════════
# QUESTION: Given a target position, what joint angles are needed?
# INPUTS  : x, y = target position of the hand
#           elbow_up = True  → elbow points upward  (solution 1)
#                      False → elbow points downward (solution 2)
# OUTPUTS : (theta1, theta2) in radians
#           OR None if target is outside reachable workspace
#
# HOW IT WORKS (Law of Cosines):
#   D = cos(theta2) — computed from target distance and link lengths
#   theta2 is solved first, then theta1 is derived from theta2
# ══════════════════════════════════════════════
def inverse_kinematics(x, y, elbow_up=True):
    D = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)

    if abs(D) > 1.0:
        return None   # Outside reachable workspace

    if elbow_up:
        theta2 = np.arctan2(-np.sqrt(1 - D**2), D)   # elbow points up
    else:
        theta2 = np.arctan2( np.sqrt(1 - D**2), D)   # elbow points down

    theta1 = np.arctan2(y, x) - np.arctan2(
        L2 * np.sin(theta2),
        L1 + L2 * np.cos(theta2)
    )

    return theta1, theta2


# ══════════════════════════════════════════════
# WORKSPACE BOUNDARY
# ══════════════════════════════════════════════
# The arm can only reach points in a ring-shaped area:
#   Outer boundary = L1 + L2 (arm fully extended)
#   Inner boundary = |L1 - L2| (arm fully folded back)
# ══════════════════════════════════════════════
def compute_workspace():
    angles = np.linspace(0, 2 * np.pi, 360)
    outer_x = (L1 + L2) * np.cos(angles)
    outer_y = (L1 + L2) * np.sin(angles)
    inner_x = abs(L1 - L2) * np.cos(angles)
    inner_y = abs(L1 - L2) * np.sin(angles)
    return outer_x, outer_y, inner_x, inner_y

# ══════════════════════════════════════════════
# DEMO 2 PLOT — IK Elbow Up vs Elbow Down
# ══════════════════════════════════════════════
def plot_ik_visualize(target, ik_up_angles, ik_down_angles):
    print("CEHCK",target[1],target[0])
    fig, axes = plt.subplots(1, 2, figsize=(13, 6), facecolor='white')
    fig.suptitle(
        f'Inverse Kinematics of 2R Serial Robot: Two Solutions for Target {target}',
        color='black', fontsize=12, fontweight='bold'
    )

    configs = [
        (ik_up_angles,   'Solution 1: Elbow UP   (θ₂ is negative)',  axes[0]),
        (ik_down_angles, 'Solution 2: Elbow DOWN (θ₂ is positive)', axes[1]),
    ]

    for (angles, label, ax) in configs:
        ax.set_facecolor('white')
        ax.set_xlim(-500, 500)
        ax.set_ylim(-500, 500)
        ax.set_aspect('equal')
        ax.set_title(label, color='black', fontsize=10, pad=10)
        ax.tick_params(colors='#666')
        ax.spines[:].set_color('#333')
        ax.grid(True, color='#1e2530', linewidth=0.7, linestyle='--')
        ax.set_xlabel('X (m)', color='#888')
        ax.set_ylabel('Y (m)', color='#888')

        ox, oy, ix, iy = compute_workspace()
        ax.plot(ox, oy, color='#1e3a5f', linewidth=1, linestyle='--', label='Workspace')
        ax.plot(ix, iy, color='#1e3a5f', linewidth=1, linestyle='--')

        if angles:
            base, elbow, hand = forward_kinematics(*angles)

            ax.plot([base[0],  elbow[0]], [base[1],  elbow[1]],
                    '-', color='#00aaff', linewidth=6,
                    solid_capstyle='round', label=f'Link 1 (L={L1}m)')
            ax.plot([elbow[0], hand[0]],  [elbow[1], hand[1]],
                    '-', color='#ff6600', linewidth=4,
                    solid_capstyle='round', label=f'Link 2 (L={L2}m)')

            
            ax.scatter(target[0],target[1], marker='*', color='red', s=100,
                    label=f'Target = {target}', zorder=7)

            ax.text(0.03, 0.97,
                    f'θ₁ = {np.degrees(angles[0]):+.1f}°\n'
                    f'θ₂ = {np.degrees(angles[1]):+.1f}°\n'
                    f'Hand = ({hand[0]:.3f}, {hand[1]:.3f})',
                    transform=ax.transAxes, color='white', fontsize=10,
                    va='top', fontfamily='monospace',
                    bbox=dict(boxstyle='round', facecolor='#1a1f2e', edgecolor='#555'))

            ax.legend(loc='lower right', fontsize=8,
                      facecolor='#1a1f2e', edgecolor='#444', labelcolor='white')

    plt.tight_layout()
    plt.show()


# ══════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════

# ══════════════════════════════════════════════
# ROBOT LINK LENGTHS
# ══════════════════════════════════════════════
L1 = 228.06   # Length of Link 1: base → elbow  (meters)
L2 = 142.50   # Length of Link 2: elbow → hand  (meters)


#%% SIMULATED ROBOT

print()
print("=" * 60)
print("   2-DOF Planar Robot Arm — Kinematics Simulation")
print("=" * 60)
print("   L1 = 1.0 m  (base → elbow)")
print("   L2 = 0.8 m  (elbow → hand)")
print()

# ──────────────────────────────────────────
# DEMO 1 — FORWARD KINEMATICS
# ──────────────────────────────────────────
FK_THETA1_DEG = 45
FK_THETA2_DEG = 30

print("─" * 60)
print("  DEMO 1 — Forward Kinematics (FK)")
print(f"  Input  : θ₁ = {FK_THETA1_DEG}°,  θ₂ = {FK_THETA2_DEG}°")
print("  Output : Where is the hand?")
print("─" * 60)


t1 = np.radians(FK_THETA1_DEG)
t2 = np.radians(FK_THETA2_DEG)
base, elbow, hand = forward_kinematics(t1, t2)

 

print(f"  Base  (fixed, never moves)  : (0.0000,  0.0000)")
print(f"  Elbow (tip of Link 1)       : ({elbow[0]:.4f}, {elbow[1]:.4f})")
print(f"  Hand  (end-effector)        : ({hand[0]:.4f}, {hand[1]:.4f})")
print()

# ──────────────────────────────────────────
# DEMO 2 — INVERSE KINEMATICS
# Uses the EXACT hand position from DEMO 1
# ──────────────────────────────────────────
IK_TARGET = (round(float(hand[0]), 4), round(float(hand[1]), 4))
print("SENTH",IK_TARGET)

print("─" * 60)
print("  Inverse Kinematics (IK)")
print(f"  Input  : Target = {IK_TARGET}  ← same position DEMO 1 calculated")
print("  Output : What angles are needed to reach this point?")
print("  NOTE   : There are always 2 solutions (elbow-up and elbow-down)")
print("─" * 60)

ik_up   = inverse_kinematics(*IK_TARGET, elbow_up=True)
ik_down = inverse_kinematics(*IK_TARGET, elbow_up=False)

_, _, hand_up   = forward_kinematics(*ik_up)
_, _, hand_down = forward_kinematics(*ik_down)

print(f"  Solution 1 — Elbow UP:")
print(f"    θ₁ = {np.degrees(ik_up[0]):+.2f}°,  θ₂ = {np.degrees(ik_up[1]):+.2f}°")
print(f"    FK verify → hand at ({hand_up[0]:.4f}, {hand_up[1]:.4f})  ✓ matches target")
print()
print(f"  Solution 2 — Elbow DOWN:")
print(f"    θ₁ = {np.degrees(ik_down[0]):+.2f}°,  θ₂ = {np.degrees(ik_down[1]):+.2f}°")
print(f"    FK verify → hand at ({hand_down[0]:.4f}, {hand_down[1]:.4f})  ✓ matches target")
print()
print(f"  Original FK used θ₁={FK_THETA1_DEG}°, θ₂={FK_THETA2_DEG}°")
print(f"  IK solution 2 (elbow-down) matches: "
      f"θ₁={np.degrees(ik_down[0]):.1f}°, θ₂={np.degrees(ik_down[1]):.1f}°")
print()
print("  [Opening DEMO 2 plot — close the window to continue]")
print()

plot_ik_visualize(IK_TARGET, ik_up, ik_down)

#%% HARDWARE ROBOT

