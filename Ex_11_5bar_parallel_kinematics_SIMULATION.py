# -*- coding: utf-8 -*-
"""
Created on Sun Nov  1 16:50:16 2026

@author: sen2d
"""

import math
import matplotlib.pyplot as plt
import numpy as np


def inverse_kinematics(x, y):
    # Angle from left shoulder to end effector
    beta1 = math.atan2( y, (l0 + x) )

    # Angle from right shoulder to end effector
    beta2 = math.atan2( y, (l0 - x) )

    # Alpha angle pre-calculations
    alpha1_calc = (l1**2 + ( (l0 + x)**2 + y**2 ) - l2**2) / (2*l1*math.sqrt( (l0 + x)**2 + y**2 ))
    alpha2_calc = (l1**2 + ( (l0 - x)**2 + y**2 ) - l2**2) / (2*l1*math.sqrt( (l0 - x)**2 + y**2 ))

    # If calculations > 1 or < -1, will fail acos function
    if alpha1_calc > 1 or alpha1_calc < -1 or alpha2_calc > 1 or alpha2_calc < -1:
        return None, None  # Indicate unreachable
    else:
        # Angle of left shoulder - beta1 and right shoulder - beta2
        alpha1 = math.acos(alpha1_calc)
        alpha2 = math.acos(alpha2_calc)

        # Angles of left and right shoulders
        shoulder1 = beta1 + alpha1
        shoulder2 = math.pi - beta2 - alpha2

        return(np.degrees(shoulder1), np.degrees(shoulder2))

def plot_arms(shoulder1, shoulder2, efx, efy):
    # Fixed points (motors)
    a1 = (-l0, 0)
    a2 = (l0, 0)

    # Passive joints (x, y) location
    p1 = ( -l0 + l1*math.cos(shoulder1), l1*math.sin(shoulder1)  )
    p2 = ( l0 + l1*math.cos(shoulder2), l1*math.sin(shoulder2)  )

    # End effector
    e = (efx, efy)

    # Unpack coordinates
    a1_x, a1_y = a1
    a2_x, a2_y = a2
    p1_x, p1_y = p1
    p2_x, p2_y = p2
    e_x, e_y = e

    fig, ax = plt.subplots(figsize=(3,3), facecolor='white')
    fig.suptitle('Inverse Kinematics of 5-Bar Parallel Robot', color='black', fontsize=12, fontweight='bold')
    
    # Stylize Axes and Grids to match the 2R theme
    ax.set_facecolor('white')
    ax.set_xlim(-200, 200)
    ax.set_ylim(0, 250)
    ax.set_aspect('equal')
    ax.tick_params(colors='#666')
    ax.spines[:].set_color('#333')
    ax.grid(True, color='#1e2530', linewidth=0.7, linestyle='--')
    ax.set_xlabel('X (mm)', color='#888')
    ax.set_ylabel('Y (mm)', color='#888')

    # Calculate precise dynamic linewidth mapping for a 60mm wide link 
    # (Since total X range spans 400mm across a 7-inch figure width)
    # Using 60mm * (7 inches / 400mm) * 72 points/inch ≈ 75 points
    lw_value = 6.0

    # Left arm (Blue/Orange combination matched to serial links)
    ax.plot([a1_x, p1_x], [a1_y, p1_y], '-', color='#00aaff', linewidth=lw_value, solid_capstyle='round', label=f'Link 1 (L={l1}mm)')
    ax.plot([p1_x, e_x], [p1_y, e_y], '-', color='#ff6600', linewidth=lw_value, solid_capstyle='round', label=f'Link 2 (L={l2}mm)')

    # Right arm (Reusing matching themes for symmetry)
    ax.plot([a2_x, p2_x], [a2_y, p2_y], '-', color='#00aaff', linewidth=lw_value, solid_capstyle='round')
    ax.plot([p2_x, e_x], [p2_y, e_y], '-', color='#ff6600', linewidth=lw_value, solid_capstyle='round')
    
    # Joints and End Effector Star
    ax.scatter([a1_x, a2_x, p1_x, p2_x], [a1_y, a2_y, p1_y, p2_y], color='white', edgecolor='#333', s=40, zorder=5)
    ax.scatter(e_x, e_y, marker='*', color='red', s=150, label=f'Target = ({efx:.1f}, {efy:.1f})', zorder=7)

    # Label text placements
    ax.text(a1_x - 10, a1_y - 10, "A1", color='#333', fontweight='bold')
    ax.text(a2_x + 5, a2_y - 10, "A2", color='#333', fontweight='bold')
    
    # Code Information Box styled exactly like the 2R arm overlay HUD
    ax.text(0.03, 0.97,
            f'θ₁ (Left)  = {math.degrees(shoulder1):+.1f}°\n'
            f'θ₂ (Right) = {math.degrees(shoulder2):+.1f}°\n'
            f'Hand position = ({e_x:.2f}, {e_y:.2f})',
            transform=ax.transAxes, color='white', fontsize=10,
            va='top', fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='#1a1f2e', edgecolor='#555'))

    ax.legend(loc='lower right', fontsize=8, facecolor='#1a1f2e', edgecolor='#444', labelcolor='white')
    plt.tight_layout()


def plot_plot(efx, efy, workspace_x, workspace_y, reach):
    # This contourf function handles plotting if points fail or when initializing workspace bounds
    s1, s2 = inverse_kinematics(efx, efy)
    s1 = np.radians(s1)
    s2 = np.radians(s2)
    if s1 is not None and s2 is not None:
        plot_arms(s1, s2, efx, efy)
        print(f"Coordinates Reachable: ({efx:.2f}, {efy:.2f})")
    else:
        # Styled fall-back plot window if coordinate is outside the workspace
        fig, ax = plt.subplots(figsize=(7, 7), facecolor='white')
        ax.set_facecolor('white')
        ax.set_xlim(-200, 200)
        ax.set_ylim(0, 250)
        ax.set_aspect('equal')
        ax.grid(True, color='#1e2530', linewidth=0.7, linestyle='--')
        ax.plot(efx, efy, 'rx', markersize=12, markeredgewidth=2, label='Unreachable Target')
        ax.legend(loc='lower right', facecolor='#1a1f2e', edgecolor='#444', labelcolor='white')
        
    plt.draw()
    plt.pause(0.1)

def forward_kinematics(theta_A1, theta_A2):
    """Forward kinematics of the 5R planar robot.
    Inputs : shoulder1, shoulder2 in radians (same convention as calc_angles).
    Returns: (efx, efy, phi), or (None, None, None) if the links cannot close.
    phi is the orientation in radians, taken as the angle of the p1 -> E link."""
    shoulder1 = np.radians(theta_A1)
    shoulder2 = np.radians(theta_A2)
    # Passive joints (same formulas as plot_arms)
    p1 = (-l0 + l1*math.cos(shoulder1), l1*math.sin(shoulder1))
    p2 = ( l0 + l1*math.cos(shoulder2), l1*math.sin(shoulder2))

    # Distance between the passive joints
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    d = math.hypot(dx, dy)

    # Two circles of radius l2 around p1 and p2 must intersect
    if d == 0 or d > 2*l2:
        return None, None, None

    # Midpoint of p1-p2 and distance from it to E
    mx = (p1[0] + p2[0]) / 2
    my = (p1[1] + p2[1]) / 2
    h = math.sqrt(l2**2 - (d/2)**2)

    # E lies along the perpendicular to p1->p2 (branch matching calc_angles)
    efx = mx + h * (-dy) / d
    efy = my + h * ( dx) / d

    # Orientation: angle of the link from p1 to E
    phi = math.atan2(efy - p1[1], efx - p1[0])

    return efx, efy, np.degrees(phi)

def compute_workspace_boundary():
    """
    Computes a grid over the approximate workspace region and determines
    which (x, y) points are reachable by the robot.

    The robot is reachable at a point (x,y) if the distance between (x,y)
    and each motor (located at (-l0, 0) and (l0, 0)) is in the range:
        [|l1 - l2|, l1+l2].
    Returns:
        X, Y: Meshgrid arrays.
        reach: A 2D array with 1 for reachable and 0 for unreachable.
    """
    x_vals = np.linspace(0, 1, 1)
    y_vals = np.linspace(180, 181, 1)
    X, Y = np.meshgrid(x_vals, y_vals)
    r1 = np.sqrt((X + l0)**2 + Y**2)
    r2 = np.sqrt((l0 - X)**2 + Y**2)
    # Reachable condition for each chain
    cond1 = np.logical_and(r1 >= abs(l1 - l2), r1 <= (l1 + l2))
    cond2 = np.logical_and(r2 >= abs(l1 - l2), r2 <= (l1 + l2))
    reach = np.logical_and(cond1, cond2).astype(float)
    print(reach)
    return X, Y, reach

#%%

# Manipulator link lengths
l0 = 148/2    # Distance from the origin to left/right motor centers
l1 = 120  # Length from motor to passive joint
l2 = 120    # Length from passive joint to end effector
  
#%%SIMULATED ROBOT
workspace_x, workspace_y, reach = compute_workspace_boundary()

a1_x, a1_y = -l0, 0
a2_x, a2_y = l0, 0
end_eff_x = -20
end_eff_y = 160
plot_plot(-20, 160, workspace_x, workspace_y, reach)
print('INVERSE KINEMATICS')
s1, s2 = inverse_kinematics(end_eff_x,end_eff_y)
print('For End-Effector Position X:{} and Y:{}, the Joint1 Angle is {} and Joint2 Angle is {}'.format(end_eff_x, end_eff_y, s1, s2))
print('FORWARD KINEMATICS')
x,y,phi = (forward_kinematics(s1, s2))
print('For  the Joint1 Angle: {} and Joint2 Angle: {}, the Effector X is {}, Y is {}'.format(s1, s2, x, y))

#%%HARDWARE ROBOT
