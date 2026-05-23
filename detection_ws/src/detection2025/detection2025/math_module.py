# math_module.py

import numpy as np
import cv2
import random
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures
from sympy import symbols, solve

# Mapping from model labels to element names.
# (For example, label 0 is "bad tomato", label 1 is "good tomato", etc.)
LABEL_MAP = {
    0: "bad tomato",
    1: "good tomato",
    2: "green tomato",
    3: "stem"
}

# ---------------------------
# Extraction & Organization
# ---------------------------
def extract_detections(results):
    """
    Extract detection information from YOLO results.
    Each detection is a dict that includes:
      - label (numeric)
      - element (name from LABEL_MAP)
      - confidence (float)
      - img_box: (x_min, x_max, y_min, y_max)
      - contour: a numpy array of (x,y) points (if available)
    """
    detections = []
    if len(results) == 0:
        return detections

    boxes = results[0].boxes
    labels = boxes.cls
    confidences = boxes.conf
    # Use masks (if available) to get contour data; otherwise, contour remains None.
    if hasattr(results[0], 'masks') and results[0].masks is not None:
        contours = results[0].masks.xy
    else:
        contours = [None] * len(labels)
    
    for idx, label in enumerate(labels):
        conf = confidences[idx].item()
        contour = contours[idx]
        if contour is not None:
            contour = contour.astype(int)
            x_min = int(np.min(contour[:, 0]))
            x_max = int(np.max(contour[:, 0]))
            y_min = int(np.min(contour[:, 1]))
            y_max = int(np.max(contour[:, 1]))
        else:
            # Fallback: use bounding box coordinates from boxes.xyxy if available.
            box = boxes.xyxy[idx].tolist()  # Format: [x_min, y_min, x_max, y_max]
            x_min, y_min, x_max, y_max = map(int, box)
        detection = {
            "label": int(label),
            "element": LABEL_MAP.get(int(label), "unknown"),
            "confidence": conf,
            "img_box": (x_min, x_max, y_min, y_max),
            "contour": contour
        }
        detections.append(detection)
    return detections

def organize_detections(detections):
    """
    Organize detections into a dictionary keyed by element name.
    Detections in each list are sorted in descending order of confidence.
    """
    data = {"good tomato": [], "bad tomato": [], "green tomato": [], "stem": []}
    for det in detections:
        elem = det.get("element", "unknown")
        if elem in data:
            data[elem].append(det)
    for key in data:
        data[key].sort(key=lambda x: x.get("confidence", 0), reverse=True)
    return data

# ---------------------------
# Basic Math Functions
# ---------------------------
def compute_centroid(contour):
    """
    Compute the centroid (center of mass) of a contour.
    Returns (cx, cy) or None if computation fails.
    """
    if contour is None or len(contour) == 0:
        return None
    pts = np.array(contour)
    M = cv2.moments(pts)
    if M["m00"] == 0:
        return None
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return (cx, cy)

def compute_centroid_from_ellipse(ellipse):
    """
    Given an ellipse (tuple: ((x_center, y_center), (axis1, axis2), angle)),
    return its center as an integer tuple.
    """
    if ellipse is None:
        return None
    center, _, _ = ellipse
    return (int(center[0]), int(center[1]))

# ---------------------------
# Ellipse Recreation Pipeline
# ---------------------------
def divide_bbox_into_grid(detection, grid_rows=3, grid_cols=3):
    """
    Given a detection's img_box, divide it into a grid (default 3x3).
    Returns a list of region tuples: (rx_min, rx_max, ry_min, ry_max).
    """
    x_min, x_max, y_min, y_max = detection["img_box"]
    grid_width = max((x_max - x_min) // grid_cols, 1)
    grid_height = max((y_max - y_min) // grid_rows, 1)
    regions = []
    for row in range(grid_rows):
        for col in range(grid_cols):
            rx_min = x_min + col * grid_width
            rx_max = x_min + (col + 1) * grid_width
            ry_min = y_min + row * grid_height
            ry_max = y_min + (row + 1) * grid_height
            regions.append((rx_min, rx_max, ry_min, ry_max))
    return regions

def assign_contour_to_regions(detection, grid_regions):
    """
    Assigns each point in the detection's contour to one of the grid regions.
    Returns a dictionary mapping region index to a dict with keys:
      - 'region': (rx_min, rx_max, ry_min, ry_max)
      - 'points': list of (x,y) points falling into that region.
    """
    regions_data = {i: {'region': reg, 'points': []} for i, reg in enumerate(grid_regions)}
    contour = detection.get("contour")
    if contour is None:
        return regions_data
    for pt in contour:
        x, y = int(pt[0]), int(pt[1])
        for i, reg in enumerate(grid_regions):
            rx_min, rx_max, ry_min, ry_max = reg
            if rx_min <= x <= rx_max and ry_min <= y <= ry_max:
                regions_data[i]['points'].append((x, y))
                break
    return regions_data

def perform_regression_on_regions(regions_data, degrees=[1, 2, 3]):
    """
    For each region that has at least two points, perform polynomial regression.
    Stores the regression coefficients under keys:
       'linear' for degree 1, and 'poly{degree}' for higher degrees.
    """
    for region_id, data in regions_data.items():
        pts = data['points']
        if len(pts) < 2:
            data['regression'] = None
            continue
        pts_arr = np.array(pts)
        X = pts_arr[:, 0].reshape(-1, 1)
        y = pts_arr[:, 1]
        data['regression'] = {}
        for deg in degrees:
            poly = PolynomialFeatures(degree=deg)
            X_poly = poly.fit_transform(X)
            model_lr = LinearRegression().fit(X_poly, y)
            coef = model_lr.coef_
            intercept = model_lr.intercept_
            if deg == 1:
                data['regression']['linear'] = {'slope': coef[1], 'intercept': intercept}
            else:
                # For degree 2, we expect [1, coef_x, coef_x2]
                data['regression'][f'poly{deg}'] = {'coefficients': coef[1:].tolist(), 'intercept': intercept}
    return regions_data

def compute_local_extrema(regions_data, degree=2):
    """
    For regions with degree-2 regression data, solve f'(x)=0 to find a critical point.
    Returns two dictionaries: one for local minima and one for local maxima.
    """
    dict_l_min = {}
    dict_l_max = {}
    x_sym = symbols('x')
    for region_id, data in regions_data.items():
        reg_data = data.get('regression')
        if not reg_data:
            continue
        poly_data = reg_data.get(f'poly{degree}')
        if not poly_data:
            continue
        # For a degree-2 polynomial: f(x)= a*x^2 + b*x + c
        # Here, coefficients are stored as [b, a] and intercept=c.
        a = poly_data['coefficients'][1]  # coefficient for x^2
        b = poly_data['coefficients'][0]  # coefficient for x
        c = poly_data['intercept']
        if a == 0:
            continue
        # Solve derivative: f'(x)= 2*a*x + b = 0  -> x = -b/(2*a)
        x_crit = -b / (2 * a)
        rx_min, rx_max, ry_min, ry_max = data['region']
        if rx_min <= x_crit <= rx_max:
            y_crit = a*(x_crit**2) + b*x_crit + c
            if 2*a > 0:
                dict_l_min[region_id] = (x_crit, y_crit)
            else:
                dict_l_max[region_id] = (x_crit, y_crit)
    return dict_l_min, dict_l_max

def get_regression_function(region_data, degree=2):
    """
    Given regression data for a region, return a lambda function for the degree-2 polynomial.
    """
    poly_data = region_data.get(f'poly{degree}')
    if poly_data is None:
        return None
    a = poly_data['coefficients'][1]
    b = poly_data['coefficients'][0]
    c = poly_data['intercept']
    return lambda x: a*(x**2) + b*x + c

def generate_estimated_points(fct, rx_min, rx_max, ry_min, ry_max, n_points=30, max_attempts=10000):
    """
    Generates estimated points from the regression function fct within the given bounds.
    """
    points = []
    attempts = 0
    while len(points) < n_points and attempts < max_attempts:
        attempts += 1
        x_val = random.uniform(rx_min, rx_max)
        y_val = fct(x_val)
        if not np.isnan(y_val) and ry_min <= y_val <= ry_max:
            points.append((x_val, y_val))
    return points

def fill_occlusions(regions_data, dict_l_min, dict_l_max, n_points=30, occluded_points=50, degree=2):
    """
    For each region, based on the presence or absence of a local extremum,
    generate estimated points to fill in gaps due to occlusion.
    The logic is region-specific.
    """
    for region_id, data in regions_data.items():
        rx_min, rx_max, ry_min, ry_max = data['region']
        fct = get_regression_function(data, degree)
        if fct is None:
            continue
        # The logic below can be tailored per region. Here we provide example heuristics:
        if region_id == 8:
            if region_id not in dict_l_max:
                # Estimate a quarter circle in top-right corner.
                m_x, m_y = rx_max, ry_max
                r = ry_max - ry_min
                theta = np.linspace(1.5 * np.pi, np.pi, 30)
                data[f"{n_points}_estimated"] = list(zip(m_x + r*np.cos(theta),
                                                         m_y + r*np.sin(theta)))
            else:
                max_x, _ = dict_l_max.get(8)
                new_rx_max = min(max_x, rx_max)
                data[f"{n_points}_estimated"] = generate_estimated_points(fct, rx_min, new_rx_max, ry_min, ry_max, n_points)
        elif region_id in [7, 1]:
            # For these regions, if no local extremum, mirror points.
            if region_id not in dict_l_min:
                est = generate_estimated_points(fct, rx_min, rx_max, ry_min, ry_max, occluded_points)
                data[f"{n_points}_estimated"] = [(x, 2*ry_max - y) for x, y in est]
            else:
                data[f"{n_points}_estimated"] = generate_estimated_points(fct, rx_min, rx_max, ry_min, ry_max, n_points)
        else:
            # For other regions, simply generate estimated points.
            data[f"{n_points}_estimated"] = generate_estimated_points(fct, rx_min, rx_max, ry_min, ry_max, n_points)
    return regions_data

def aggregate_estimated_points(regions_data, key="30_estimated"):
    """
    Aggregate all estimated points from the regions (using the given key).
    Returns a numpy array of points.
    """
    all_points = []
    for data in regions_data.values():
        if key in data:
            all_points.extend(data[key])
    if len(all_points) == 0:
        return None
    return np.array(all_points)

def fit_ellipse_to_points(points, fallback_contour=None):
    """
    Fit an ellipse to a set of points using SVD.
    If points is None or has too few points (<5), fall back to cv2.fitEllipse on fallback_contour.
    Returns an ellipse in the format ((x_center, y_center), (axis1, axis2), angle) or None.
    """
    if points is None or len(points) < 5:
        if fallback_contour is not None and len(fallback_contour) >= 5:
            pts = np.array(fallback_contour).reshape(-1, 1, 2).astype(np.int32)
            try:
                ellipse = cv2.fitEllipse(pts)
                return ellipse
            except Exception as e:
                print("cv2.fitEllipse failed:", e)
                return None
        else:
            return None
    # Center the points.
    x_vals = points[:, 0]
    y_vals = points[:, 1]
    x_mean = x_vals.mean()
    y_mean = y_vals.mean()
    x_centered = x_vals - x_mean
    y_centered = y_vals - y_mean
    coords = np.vstack([x_centered, y_centered])
    U, S, _ = np.linalg.svd(coords)
    angle = np.arctan2(U[1, 0], U[0, 0]) * 180 / np.pi
    # Use singular values as semi-axis lengths (scaling factor can be adjusted).
    axes = (S[0], S[1])
    return ((x_mean, y_mean), axes, angle)

def recreate_ellipse(detection, image):
    """
    Recreates an ellipse for the given detection using a multi‐step pipeline:
      1. Divide the detection's bounding box into a 3×3 grid.
      2. Assign contour points to each grid region.
      3. Perform polynomial regression (degrees 1,2,3) on each region.
      4. Compute local extrema (minima/maxima) from the degree‑2 regressions.
      5. For each region, generate estimated points to handle occlusions.
      6. Aggregate all estimated points and fit an ellipse via SVD.
      7. If insufficient estimated points are available, fall back to cv2.fitEllipse on the raw contour.
    Returns the ellipse parameters (or None if all methods fail).
    """
    # Step 1: Divide bounding box into grid.
    grid_regions = divide_bbox_into_grid(detection, grid_rows=3, grid_cols=3)
    # Step 2: Assign contour points.
    regions_data = assign_contour_to_regions(detection, grid_regions)
    # Step 3: Perform regressions.
    regions_data = perform_regression_on_regions(regions_data, degrees=[1,2,3])
    # Step 4: Compute local extrema.
    dict_l_min, dict_l_max = compute_local_extrema(regions_data, degree=2)
    # Step 5: Generate estimated points to fill occlusions.
    regions_data = fill_occlusions(regions_data, dict_l_min, dict_l_max, n_points=30, occluded_points=50, degree=2)
    # Step 6: Aggregate estimated points.
    all_est_points = aggregate_estimated_points(regions_data, key="30_estimated")
    # Step 7: Fit ellipse using SVD; fallback to cv2.fitEllipse if necessary.
    ellipse = fit_ellipse_to_points(all_est_points, fallback_contour=detection.get("contour"))
    return ellipse

# ---------------------------
# Processing Pipeline
# ---------------------------
def process_detection(detection, image, element_type):
    """
    Processes a single detection according to its element type.
    
    - For bad tomato: compute detection & contour, then attempt full ellipse recreation.
      The centroid is computed from the ellipse (if available) or from the contour.
    - For good tomato and green tomato: compute detection, contour, and centroid from the contour.
    - For stem: store only detection and contour.
    
    Returns a dictionary with keys:
      detection, contour, ellipse (if applicable), centroid.
    """
    result = {"detection": detection, "contour": detection.get("contour")}
    if element_type == "bad tomato":
        ellipse = None
        if detection.get("contour") is not None and image is not None:
            ellipse = recreate_ellipse(detection, image)
        result["ellipse"] = ellipse
        if ellipse is not None:
            result["centroid"] = compute_centroid_from_ellipse(ellipse)
        else:
            result["centroid"] = compute_centroid(detection.get("contour"))
    elif element_type in ["good tomato", "green tomato"]:
        result["centroid"] = compute_centroid(detection.get("contour"))
        result["ellipse"] = None
    elif element_type == "stem":
        result["ellipse"] = None
        result["centroid"] = None
    return result

def process_all_detections(organized_detections, image, confidence_threshold=0.5):
    """
    Processes all detections in the organized dictionary, filtering out those
    with a confidence below the given threshold.
    
    Returns a new dictionary where keys are element types and values are lists
    of processed detection data (with ellipse recreation for bad tomato, etc.).
    
    :param organized_detections: dictionary with element names as keys and lists of detections.
    :param image: the image (as a numpy array) on which detections were made.
    :param confidence_threshold: float, only detections with confidence >= threshold are processed.
    """
    processed = {}
    for element, detections in organized_detections.items():
        processed[element] = []
        for det in detections:
            if det.get("confidence", 0) >= confidence_threshold:
                processed[element].append(process_detection(det, image, element))
    return processed

# ---------------------------
# Example Usage (for testing)
# ---------------------------
if __name__ == "__main__":
    # For testing, load a sample image and YOLO model.
    from ultralytics import YOLO
    from PIL import Image
    import matplotlib.pyplot as plt

    model = YOLO("best.pt")
    image_path = "tomato.jpg"
    image = Image.open(image_path).rotate(180)

    results = model.predict(image)
    detections = extract_detections(results)
    organized = organize_detections(detections)
    # Process all detections using the original image (PIL Image converted to numpy array if needed)
    # Here we assume the image is in a format cv2 can work with; if not, convert via np.array(image)
    cv_image = np.array(image)
    processed_data = process_all_detections(organized, cv_image)

    # For demonstration, print processed data for each element.
    for element, data in processed_data.items():
        print(f"{element}:")
        for idx, d in enumerate(data):
            print(f"  Detection {idx+1}: centroid={d.get('centroid')}, ellipse={d.get('ellipse')}")
