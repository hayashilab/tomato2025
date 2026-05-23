import torch
from PIL import Image
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from ultralytics import YOLO
import random
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures
import pandas as pd
from sympy import symbols, solve

# Load YOLO model and image
model_path = "best.pt"
model = YOLO(model_path)

# testers
# image_path = "/mnt/UE2ROS/Tomato/src/tomato/dataset_Jan/train/IMG_0375_jpg.rf.f02e5921b1b20677455ece136359f8b3.jpg"  # Update the path
# image_path = "/mnt/UE2ROS/Tomato/src/tomato/dataset_Jan/train/16_JPG.rf.7341af50f7888fe4dbbbf85924db4274.jpg"  # Update the path

image_path = "/mnt/UE2ROS/Tomato/src/tomato/dataset_Jan/train/IMG_0375_jpg.rf.f02e5921b1b20677455ece136359f8b3.jpg"
# tester
# image = Image.open(image_path).rotate(180)
image = Image.open(image_path)

# Get model predictions (only one call needed)
results = model.predict(image)

# -----------------------------------------------------------------------------
# Functions
# -----------------------------------------------------------------------------
def get_tomatoes(prediction):
    """
    Extracts tomato detections from a YOLO prediction and returns a dictionary
    with each tomato's label, confidence, bounding box, and contour.
    """
    tomatoes = {}
    contour_points = prediction[0].masks.xy
    labels = prediction[0].boxes.cls
    confidences = prediction[0].boxes.conf
    for idx, label in enumerate(labels):
        if label != 3:  # Skip label 3 (assumed non-tomato, e.g. stem)
            contour = contour_points[idx].astype(int)
            x_min, x_max = int(np.min(contour[:, 0])), int(np.max(contour[:, 0]))
            y_min, y_max = int(np.min(contour[:, 1])), int(np.max(contour[:, 1]))
            tomatoes[len(tomatoes)] = {
                'label': label,
                'confidence': confidences[idx].item(),
                'img_box': (x_min, x_max, y_min, y_max),
                'contour': contour,
            }
    return tomatoes


def get_regions_contours(tomato, image):
    """
    Splits the tomato bounding box into a 3x3 grid and assigns contour points 
    to the corresponding region. Also displays the grid on the image.
    """
    x_min, x_max, y_min, y_max = tomato['img_box']
    grid_width = (x_max - x_min) // 3
    grid_height = (y_max - y_min) // 3

    regions = []
    for row in range(3):
        for col in range(3):
            rx_min = x_min + col * grid_width
            rx_max = x_min + (col + 1) * grid_width
            ry_min = y_min + row * grid_height
            ry_max = y_min + (row + 1) * grid_height
            regions.append((rx_min, rx_max, ry_min, ry_max))
    
    # Plot grid for visual verification
    fig, ax = plt.subplots()
    ax.imshow(image)
    for region in regions:
        rect = patches.Rectangle(
            (region[0], region[2]), region[1] - region[0], region[3] - region[2],
            linewidth=2, edgecolor='r', facecolor='none'
        )
        ax.add_patch(rect)
    plt.axis('off')
    plt.show()

    # Assign contour points to each region
    regions_with_contours = {i: {'region': reg, 'points': []} for i, reg in enumerate(regions)}
    for point in tomato['contour']:
        x, y = int(point[0]), int(point[1])
        for idx, reg in enumerate(regions):
            rx_min, rx_max, ry_min, ry_max = reg
            if rx_min <= x <= rx_max and ry_min <= y <= ry_max:
                regions_with_contours[idx]['points'].append((x, y))
    return regions_with_contours


def perform_regressions(regions, degrees=[1, 2, 3]):
    """
    For each region, performs regression for the specified degrees. 
    A degree of 1 is stored as 'linear'; higher orders are stored as 'polynomial{degree}'.
    """
    for region_id, data in regions.items():
        points = data['points']
        if len(points) < 2:
            print(f"Region {region_id}: Not enough points for regression.")
            continue
        X = np.array([p[0] for p in points]).reshape(-1, 1)
        y = np.array([p[1] for p in points])
        for deg in degrees:
            poly = PolynomialFeatures(degree=deg)
            X_poly = poly.fit_transform(X)
            model_lr = LinearRegression().fit(X_poly, y)
            coef = model_lr.coef_
            intercept = model_lr.intercept_
            if deg == 1:
                key = 'linear'
                # For degree 1, coef[1] is the slope.
                reg_data = {'coefficients': coef[1], 'intercept': intercept}
            else:
                key = f'polynomial{deg}'
                # Exclude the bias term; coefficients correspond to x, x^2, ...
                reg_data = {'coefficients': coef[1:].tolist(), 'intercept': intercept}
            data[key] = reg_data
    return regions


def get_local_min_max(regions, image, degree=2):
    """
    For degree-2 polynomials in each region, finds a critical point by solving the derivative.
    If the second derivative is negative the critical point is a maximum, otherwise a minimum.
    The function also plots the regression curves.
    """
    dict_l_min = {}
    dict_l_max = {}

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(image)
    for region_id, data in regions.items():
        rx_min, rx_max, ry_min, ry_max = data['region']
        poly_data = data.get('polynomial2')
        if not poly_data:
            print(f"Region {region_id}: No degree-2 polynomial available.")
            continue
        # Assume coefficients: [coef for x, coef for x^2]
        c1, c0 = poly_data['coefficients']
        intercept = poly_data['intercept']
        x_sym = symbols('x')
        f_expr = c0 * x_sym**2 + c1 * x_sym + intercept
        f_prime = f_expr.diff(x_sym)
        f_double = f_expr.diff(x_sym, 2)
        crit_points = solve(f_prime, x_sym)
        if crit_points:
            crit = float(crit_points[0])
            second_deriv = float(f_double.subs(x_sym, crit))
            y_val = float(f_expr.subs(x_sym, crit))
            if second_deriv < 0:
                dict_l_max[region_id] = (crit, y_val)
                ax.scatter(crit, y_val, color='yellow', s=70, label=f"Max {region_id}")
            else:
                dict_l_min[region_id] = (crit, y_val)
                ax.scatter(crit, y_val, color='red', s=70, label=f"Min {region_id}")
        # Plot the regression curve
        a_vals = np.linspace(rx_min, rx_max, 50)
        f_vals = [float(f_expr.subs(x_sym, val)) for val in a_vals]
        # Only keep values within the region bounds
        a_filtered = [a_vals[i] for i, val in enumerate(f_vals) if ry_min <= val <= ry_max]
        f_filtered = [val for val in f_vals if ry_min <= val <= ry_max]
        ax.plot(a_filtered, f_filtered, label=f"Region {region_id} Regression", linewidth=2, color="green")
    ax.legend()
    plt.show()
    return dict_l_min, dict_l_max


def generate_estimated_points(fct, lower_x, upper_x, lower_y, upper_y, n_points, max_attempts=10000):
    """
    Generates n_points from the function fct within the provided bounds.
    """
    points = []
    attempts = 0
    while len(points) < n_points and attempts < max_attempts:
        attempts += 1
        x_val = random.uniform(lower_x, upper_x)
        y_val = fct(x_val)
        if not np.isnan(y_val) and lower_y <= y_val <= upper_y:
            points.append((x_val, y_val))
    if len(points) < n_points:
        print(f"Warning: Only generated {len(points)} points after {attempts} attempts.")
    return points


def get_function(region_data, degree=2):
    """
    Returns a lambda function corresponding to the degree-2 polynomial fitted for the region.
    """
    poly_data = region_data.get(f'polynomial{degree}')
    if poly_data is None:
        return None
    if degree == 2:
        c1, c0 = poly_data['coefficients']
        intercept = poly_data['intercept']
        return lambda x: c0 * x**2 + c1 * x + intercept
    return None


def clean_of_occlusion(regions, dict_l_min, dict_l_max, n_points=30, occluded_points=50, degree=2):
    """
    Based on region ID and the existence of a local min/max, this function estimates new
    contour points to mitigate occlusions. The logic is specialized per region.
    """
    for region_id, data in regions.items():
        rx_min, rx_max, ry_min, ry_max = data['region']
        fct = get_function(data, degree)
        if fct is None:
            continue

        if region_id == 8:
            if region_id not in dict_l_max:
                # Estimate quarter circle in top-right corner
                m_x, m_y = rx_max, ry_max
                r = ry_max - ry_min
                theta = np.linspace(1.5 * np.pi, np.pi, 30)
                data[f"{n_points}_estimated"] = list(zip(m_x + r * np.cos(theta),
                                                         m_y + r * np.sin(theta)))
            else:
                max_x, _ = dict_l_max[8]
                new_rx_max = max_x if max_x < rx_max else rx_max
                data[f"{n_points}_estimated"] = generate_estimated_points(fct, rx_min, new_rx_max, ry_min, ry_max, n_points)

        elif region_id == 7:
            if region_id not in dict_l_max:
                est = generate_estimated_points(fct, rx_min, rx_max, ry_min, ry_max, occluded_points)
                data[f"{n_points}_estimated"] = [(x, 2 * ry_max - y) for x, y in est]
            else:
                data[f"{n_points}_estimated"] = generate_estimated_points(fct, rx_min, rx_max, ry_min, ry_max, n_points)

        elif region_id == 6:
            if region_id not in dict_l_max:
                m_x, m_y = rx_min, ry_max
                r = ry_max - ry_min
                theta = np.linspace(2 * np.pi, 1.5 * np.pi, 30)
                data[f"{n_points}_estimated"] = list(zip(m_x + r * np.cos(theta),
                                                         m_y + r * np.sin(theta)))
            else:
                data[f"{n_points}_estimated"] = generate_estimated_points(fct, rx_min, rx_max, ry_min, ry_max, n_points)

        elif region_id == 2:
            if region_id not in dict_l_min:
                m_x, m_y = rx_max, ry_min
                r = ry_max - ry_min
                theta = np.linspace(np.pi, 0.5 * np.pi, 30)
                data[f"{n_points}_estimated"] = list(zip(m_x + r * np.cos(theta),
                                                         m_y + r * np.sin(theta)))
            else:
                data[f"{n_points}_estimated"] = generate_estimated_points(fct, rx_min, rx_max, ry_min, ry_max, n_points)

        elif region_id == 1:
            if region_id not in dict_l_min:
                est = generate_estimated_points(fct, rx_min, rx_max, ry_min, ry_max, occluded_points)
                data[f"{n_points}_estimated"] = [(x, 2 * ry_max - y) for x, y in est]
            else:
                data[f"{n_points}_estimated"] = generate_estimated_points(fct, rx_min, rx_max, ry_min, ry_max, n_points)

        elif region_id == 0:
            if region_id not in dict_l_min:
                m_x, m_y = rx_min, ry_min
                r = ry_max - ry_min
                theta = np.linspace(2 * np.pi, 1.5 * np.pi, 30)
                data[f"{n_points}_estimated"] = list(zip(m_x + r * np.cos(theta),
                                                         m_y + r * np.sin(theta)))
            else:
                data[f"{n_points}_estimated"] = generate_estimated_points(fct, rx_min, rx_max, ry_min, ry_max, n_points)

        elif region_id in [5, 3]:
            data[f"{n_points}_estimated"] = generate_estimated_points(fct, rx_min, rx_max, ry_min, ry_max, n_points)
    return regions


def plot_estimated_points_on_image(image, regions):
    """
    Plots region bounding boxes and any estimated points (stored under "30_estimated")
    on the image.
    """
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(image)
    for data in regions.values():
        rx_min, rx_max, ry_min, ry_max = data['region']
        rect = patches.Rectangle(
            (rx_min, ry_min), rx_max - rx_min, ry_max - ry_min,
            linewidth=2, edgecolor='r', facecolor='none'
        )
        ax.add_patch(rect)
        key = "30_estimated"
        if key in data:
            for x, y in data[key]:
                ax.scatter(x, y, color='blue', s=10)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    plt.show()


def plot_estimated_points_and_ellipse_on_image(image, regions):
    """
    Aggregates estimated points from all regions, fits an ellipse to them using SVD,
    and plots both the points and ellipse on the image.
    """
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(image)
    all_points = []
    for data in regions.values():
        key = "30_estimated"
        if key in data:
            all_points.extend(data[key])
    if not all_points:
        print("No estimated points available.")
        return
    all_points = np.array(all_points)
    x_vals, y_vals = all_points[:, 0], all_points[:, 1]
    ax.scatter(x_vals, y_vals, color='blue', s=10, label="Estimated Points")

    # Fit an ellipse via SVD on the centered points
    x_mean, y_mean = x_vals.mean(), y_vals.mean()
    x_centered = x_vals - x_mean
    y_centered = y_vals - y_mean
    U, S, _ = np.linalg.svd(np.stack([x_centered, y_centered]))
    t = np.linspace(0, 2 * np.pi, 1000)
    circle = np.array([np.cos(t), np.sin(t)])
    transform = U @ np.diag(S) * np.sqrt(2 / len(all_points))
    ellipse = transform @ circle + np.array([[x_mean], [y_mean]])
    ax.plot(ellipse[0, :], ellipse[1, :], 'r', label="Fitted Ellipse")
    ax.set_aspect('equal')
    ax.legend()
    ax.set_title("Estimated Points and Fitted Ellipse")
    plt.show()


# -----------------------------------------------------------------------------
# Main Execution
# -----------------------------------------------------------------------------
tomatoes = get_tomatoes(results)
if not tomatoes:
    raise ValueError("No tomatoes detected.")

# Use the second tomato if available; otherwise, use the first detected tomato.
tomato = tomatoes.get(2, list(tomatoes.values())[0])

regions = get_regions_contours(tomato, image)
regions = perform_regressions(regions, degrees=[1, 2, 3])
dict_l_min, dict_l_max = get_local_min_max(regions, image, degree=2)
regions = clean_of_occlusion(regions, dict_l_min, dict_l_max, n_points=30)
plot_estimated_points_on_image(image, regions)
plot_estimated_points_and_ellipse_on_image(image, regions)
