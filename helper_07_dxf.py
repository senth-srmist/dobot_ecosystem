import ezdxf
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os
import csv
# ============================================================
# USER SETTINGS
# ============================================================
def generate_csv_from_dxf():
    DXF_FILE = "random.dxf"

    # Tolerance used for SPLINE approximation
    # Smaller value = more points
    SPLINE_TOLERANCE = 0.5
    
    # Approximate point spacing used for ARC sampling
    ARC_SPACING = 0.5
    
    # CSV output
    CSV_FILE = "trajectory_3D.csv"
    
    
    # ============================================================
    # 1. LOAD DXF
    # ============================================================
    
    print("=" * 60)
    print("DXF → 3D TRAJECTORY EXTRACTION")
    print("=" * 60)
    
    print("\nLoading DXF...")
    
    doc = ezdxf.readfile(DXF_FILE)
    msp = doc.modelspace()
    
    print("DXF loaded successfully")
    
    
    # ============================================================
    # 2. CHECK DXF UNITS
    # ============================================================
    
    print("\n" + "=" * 60)
    print("DXF UNITS")
    print("=" * 60)
    
    units_code = doc.header.get("$INSUNITS")
    
    unit_names = {
        0: "Unitless",
        1: "Inches",
        2: "Feet",
        3: "Miles",
        4: "Millimeters",
        5: "Centimeters",
        6: "Meters",
        7: "Kilometers",
        8: "Microinches",
        9: "Mils",
        10: "Yards",
        11: "Angstroms",
        12: "Nanometers",
        13: "Microns",
        14: "Decimeters",
        15: "Decameters",
        16: "Hectometers",
        17: "Gigameters",
        18: "Astronomical Units",
        19: "Light Years",
        20: "Parsecs"
    }
    
    print("INSUNITS code:", units_code)
    print("DXF unit:", unit_names.get(units_code, "Unknown"))
    
    if units_code == 4:
        print("✓ DXF is specified in millimeters")
    elif units_code == 0:
        print("⚠ DXF is unitless.")
        print("  Verify dimensions against Fusion 360.")
    else:
        print("⚠ DXF is not specified as millimeters.")
        print("  Verify the scale before using the trajectory.")
    
    
    # ============================================================
    # 3. LIST ENTITIES
    # ============================================================
    
    print("\n" + "=" * 60)
    print("ENTITIES FOUND")
    print("=" * 60)
    
    entity_counts = {}
    
    for entity in msp:
    
        entity_type = entity.dxftype()
    
        if entity_type not in entity_counts:
            entity_counts[entity_type] = 0
    
        entity_counts[entity_type] += 1
    
    for entity_type, count in entity_counts.items():
    
        print(
            f"{entity_type:<15} : {count}"
        )
    
    
    # ============================================================
    # 4. EXTRACT 3D TRAJECTORY
    # ============================================================
    
    print("\n" + "=" * 60)
    print("EXTRACTING TRAJECTORY")
    print("=" * 60)
    
    trajectory = []
    
    entity_point_counts = {}
    
    
    for entity in msp:
    
        entity_type = entity.dxftype()
    
        print("Processing:", entity_type)
    
    
        # ========================================================
        # ARC
        # ========================================================
    
        if entity_type == "ARC":
    
            center = entity.dxf.center
    
            radius = entity.dxf.radius
    
            start_angle = np.radians(
                entity.dxf.start_angle
            )
    
            end_angle = np.radians(
                entity.dxf.end_angle
            )
    
            # DXF ARC normally goes counter-clockwise.
            if end_angle <= start_angle:
    
                end_angle += 2 * np.pi
    
    
            # Calculate arc length
    
            arc_length = radius * (
                end_angle - start_angle
            )
    
    
            # Number of points
    
            n_points = max(
                2,
                int(
                    np.ceil(
                        arc_length / ARC_SPACING
                    )
                ) + 1
            )
    
    
            angles = np.linspace(
                start_angle,
                end_angle,
                n_points
            )
    
    
            count = 0
    
    
            for angle in angles:
    
                x = (
                    center.x
                    + radius * np.cos(angle)
                )
    
                y = (
                    center.y
                    + radius * np.sin(angle)
                )
    
                z = center.z
    
    
                trajectory.append(
                    [x, y, z]
                )
    
                count += 1
    
    
            entity_point_counts["ARC"] = count
    
    
        # ========================================================
        # SPLINE
        # ========================================================
    
        elif entity_type == "SPLINE":
    
            # Important:
            # Use positional argument rather than
            # flattening(distance=...)
            #
            # This works with your ezdxf version.
    
            points = entity.flattening(
                SPLINE_TOLERANCE
            )
    
    
            count = 0
    
    
            for p in points:
    
                trajectory.append(
                    [
                        p.x,
                        p.y,
                        p.z
                    ]
                )
    
                count += 1
    
    
            entity_point_counts["SPLINE"] = count
    
    
        # ========================================================
        # LINE
        # ========================================================
    
        elif entity_type == "LINE":
    
            p1 = entity.dxf.start
    
            p2 = entity.dxf.end
    
    
            trajectory.append(
                [
                    p1.x,
                    p1.y,
                    p1.z
                ]
            )
    
    
            trajectory.append(
                [
                    p2.x,
                    p2.y,
                    p2.z
                ]
            )
    
    
            entity_point_counts["LINE"] = 2
    
    
        # ========================================================
        # 3D POLYLINE
        # ========================================================
    
        elif entity_type == "POLYLINE":
    
            count = 0
    
    
            for vertex in entity.vertices:
    
                p = vertex.dxf.location
    
    
                trajectory.append(
                    [
                        p.x,
                        p.y,
                        p.z
                    ]
                )
    
    
                count += 1
    
    
            entity_point_counts["POLYLINE"] = count
    
    
        # ========================================================
        # LWPOLYLINE
        # ========================================================
    
        elif entity_type == "LWPOLYLINE":
    
            elevation = entity.dxf.elevation
    
            count = 0
    
    
            for p in entity.get_points():
    
                x = p[0]
    
                y = p[1]
    
                z = elevation.z
    
    
                trajectory.append(
                    [
                        x,
                        y,
                        z
                    ]
                )
    
    
                count += 1
    
    
            entity_point_counts["LWPOLYLINE"] = count
    
    
        # ========================================================
        # CIRCLE
        # ========================================================
    
        elif entity_type == "CIRCLE":
    
            center = entity.dxf.center
    
            radius = entity.dxf.radius
    
    
            circumference = (
                2 * np.pi * radius
            )
    
    
            n_points = max(
                20,
                int(
                    np.ceil(
                        circumference / ARC_SPACING
                    )
                )
            )
    
    
            angles = np.linspace(
                0,
                2 * np.pi,
                n_points,
                endpoint=False
            )
    
    
            count = 0
    
    
            for angle in angles:
    
                x = (
                    center.x
                    + radius * np.cos(angle)
                )
    
                y = (
                    center.y
                    + radius * np.sin(angle)
                )
    
                z = center.z
    
    
                trajectory.append(
                    [
                        x,
                        y,
                        z
                    ]
                )
    
                count += 1
    
    
            entity_point_counts["CIRCLE"] = count
    
    
        # ========================================================
        # OTHER ENTITY
        # ========================================================
    
        else:
    
            print(
                "  Skipped:",
                entity_type
            )
    
    
    # ============================================================
    # 5. CHECK EXTRACTION
    # ============================================================
    
    print("\n" + "=" * 60)
    print("EXTRACTION SUMMARY")
    print("=" * 60)
    
    print(
        "Total extracted points:",
        len(trajectory)
    )
    
    if len(trajectory) == 0:
    
        print("\nERROR:")
        print("No supported geometry was found.")
        print("Check the DXF entity types.")
    
        exit()
    
    
    # ============================================================
    # 6. CREATE DATAFRAME
    # ============================================================
    
    df = pd.DataFrame(
        trajectory,
        columns=[
            "X",
            "Y",
            "Z"
        ]
    )
    
    
    # ============================================================
    # 7. REMOVE DUPLICATE CONSECUTIVE POINTS
    # ============================================================
    
    points = df[
        ["X", "Y", "Z"]
    ].values
    
    
    clean_points = [points[0]]
    
    
    for i in range(1, len(points)):
    
        distance = np.linalg.norm(
            points[i] - points[i - 1]
        )
    
    
        if distance > 1e-9:
    
            clean_points.append(
                points[i]
            )
    
    
    df = pd.DataFrame(
        clean_points,
        columns=[
            "X",
            "Y",
            "Z"
        ]
    )
    
    
    print(
        "Points after duplicate removal:",
        len(df)
    )
    
    
    # ============================================================
    # 8. DIMENSION / BOUNDING BOX
    # ============================================================
    
    print("\n" + "=" * 60)
    print("TRAJECTORY DIMENSIONS")
    print("=" * 60)
    
    
    xmin = df["X"].min()
    xmax = df["X"].max()
    
    ymin = df["Y"].min()
    ymax = df["Y"].max()
    
    zmin = df["Z"].min()
    zmax = df["Z"].max()
    
    
    x_size = xmax - xmin
    y_size = ymax - ymin
    z_size = zmax - zmin
    
    
    print(
        f"X range : {xmin:.3f} → {xmax:.3f}"
    )
    
    print(
        f"Y range : {ymin:.3f} → {ymax:.3f}"
    )
    
    print(
        f"Z range : {zmin:.3f} → {zmax:.3f}"
    )
    
    
    print("\nOverall dimensions:")
    
    print(
        f"X dimension = {x_size:.3f}"
    )
    
    print(
        f"Y dimension = {y_size:.3f}"
    )
    
    print(
        f"Z dimension = {z_size:.3f}"
    )
    
    
    # ============================================================
    # 9. CALCULATE TRAJECTORY LENGTH
    # ============================================================
    
    points = df[
        ["X", "Y", "Z"]
    ].values
    
    
    differences = np.diff(
        points,
        axis=0
    )
    
    
    distances = np.linalg.norm(
        differences,
        axis=1
    )
    
    
    total_length = np.sum(
        distances
    )
    
    
    print("\n" + "=" * 60)
    print("TRAJECTORY LENGTH")
    print("=" * 60)
    
    
    print(
        f"Total trajectory length = "
        f"{total_length:.3f}"
    )
    
    
    # ============================================================
    # 10. POINT SPACING
    # ============================================================
    
    print("\n" + "=" * 60)
    print("POINT SPACING")
    print("=" * 60)
    
    
    if len(distances) > 0:
    
        min_spacing = np.min(
            distances
        )
    
        max_spacing = np.max(
            distances
        )
    
        mean_spacing = np.mean(
            distances
        )
    
        median_spacing = np.median(
            distances
        )
    
    
        print(
            f"Minimum spacing = "
            f"{min_spacing:.4f}"
        )
    
        print(
            f"Maximum spacing = "
            f"{max_spacing:.4f}"
        )
    
        print(
            f"Average spacing = "
            f"{mean_spacing:.4f}"
        )
    
        print(
            f"Median spacing = "
            f"{median_spacing:.4f}"
        )
    
    
    # ============================================================
    # 11. PRINT FIRST 10 POINTS
    # ============================================================
    
    print("\n" + "=" * 60)
    print("FIRST 10 TRAJECTORY POINTS")
    print("=" * 60)
    
    print(
        df.head(10).to_string(
            index=True
        )
    )
    
    
    # ============================================================
    # 12. SAVE CSV
    # ============================================================
    
    df.to_csv(
        CSV_FILE,
        index=False
    )
    
    
    print("\n" + "=" * 60)
    print("CSV EXPORT")
    print("=" * 60)
    
    print(
        "Saved:",
        os.path.abspath(CSV_FILE)
    )
    
    
    # ============================================================
    # 13. 3D PLOT
    # ============================================================
    
    fig = plt.figure(
        figsize=(11, 8)
    )
    
    
    ax = fig.add_subplot(
        111,
        projection="3d"
    )
    
    
    ax.plot(
        df["X"],
        df["Y"],
        df["Z"],
        linewidth=2
    )
    
    
    ax.scatter(
        df["X"],
        df["Y"],
        df["Z"],
        s=8
    )
    
    
    ax.set_xlabel(
        "X (mm)"
    )
    
    ax.set_ylabel(
        "Y (mm)"
    )
    
    ax.set_zlabel(
        "Z (mm)"
    )
    
    
    ax.set_title(
        "3D Weld Seam Trajectory"
    )
    
    
    # ============================================================
    # 14. EQUAL 3D AXIS SCALE
    # ============================================================
    
    max_range = max(
        x_size,
        y_size,
        z_size
    ) / 2
    
    
    x_mid = (
        xmin + xmax
    ) / 2
    
    
    y_mid = (
        ymin + ymax
    ) / 2
    
    
    z_mid = (
        zmin + zmax
    ) / 2
    
    
    ax.set_xlim(
        x_mid - max_range,
        x_mid + max_range
    )
    
    ax.set_ylim(
        y_mid - max_range,
        y_mid + max_range
    )
    
    ax.set_zlim(
        z_mid - max_range,
        z_mid + max_range
    )
    
    
    plt.tight_layout()
    
    plt.show()
    
    
    # ============================================================
    # END
    # ============================================================
    
    print("\n" + "=" * 60)
    print("DONE")
    print("=" * 60)
#%%
def load_csv(filename):
    # Number of decimal places sent to robot
    COORDINATE_DECIMALS = 3

    # ------------------------------------------------------------
    # SAFETY TEST
    # ------------------------------------------------------------


    # Maximum allowed coordinates for an accidental bad CSV.
    # Adjust these to your already-established workspace.


    if not os.path.isfile(filename):
        raise FileNotFoundError(
            "CSV file not found:\n" + filename
        )

    points = []

    with open(filename, "r", newline="") as f:

        reader = csv.DictReader(f)

        # Check header
        if reader.fieldnames is None:
            raise ValueError("CSV has no header.")

        headers = [h.strip().upper() for h in reader.fieldnames]

        required = ["X", "Y", "Z"]

        for h in required:
            if h not in headers:
                raise ValueError(
                    "CSV must contain X,Y,Z columns."
                )

        for line_number, row in enumerate(reader, start=2):

            try:
                x = float(row["X"])
                y = float(row["Y"])
                z = float(row["Z"])
            except Exception:
                raise ValueError(
                    "Invalid coordinate at CSV line "
                    + str(line_number)
                )

            # Round only the value being sent to the robot.
            # Original CSV remains untouched.
            x = round(x, COORDINATE_DECIMALS)
            y = round(y, COORDINATE_DECIMALS)
            z = round(z, COORDINATE_DECIMALS)

            points.append([x, y, z])

    if len(points) < 2:
        raise ValueError(
            "CSV must contain at least two points."
        )

    return points


# ============================================================
# VALIDATE POINTS
# ============================================================

def validate_points(points):
    MAX_ABS_X = 500.0
    MAX_ABS_Y = 650.0
    MAX_ABS_Z = 500.0

    for i, (x, y, z) in enumerate(points):

        if abs(x) > MAX_ABS_X:
            raise ValueError(
                "Point {}: X={} exceeds safety limit.".format(
                    i + 1, x
                )
            )

        if abs(y) > MAX_ABS_Y:
            raise ValueError(
                "Point {}: Y={} exceeds safety limit.".format(
                    i + 1, y
                )
            )

        if abs(z) > MAX_ABS_Z:
            raise ValueError(
                "Point {}: Z={} exceeds safety limit.".format(
                    i + 1, z
                )
            )


# ============================================================
# PRINT PATH INFORMATION
# ============================================================

def print_path_information(points):

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]

    print("")
    print("============================================")
    print("CSV PATH INFORMATION")
    print("============================================")

    print("Number of points :", len(points))

    print("X range          :", min(xs), "to", max(xs))
    print("Y range          :", min(ys), "to", max(ys))
    print("Z range          :", min(zs), "to", max(zs))

    print("")
    print("First point:")
    print("X =", points[0][0])
    print("Y =", points[0][1])
    print("Z =", points[0][2])

    print("")
    print("Last point:")
    print("X =", points[-1][0])
    print("Y =", points[-1][1])
    print("Z =", points[-1][2])

    print("============================================")
    print("")
