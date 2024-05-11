from PIL import Image, ImageFilter
import numpy as np
import cv2
from sklearn.cluster import KMeans


def image_analyse(image_path):
    # Load the image
    img = Image.open(image_path)
    vertices = extract_vertices(img)
    img.close()
    return vertices


def extract_vertices(image):
    width, height = image.size

    # Convert the image in W/B (1-bit) to simplify
    img_bw = image.convert('1')
    # Apply an erosion filter
    img_eroded = img_bw.filter(ImageFilter.MaxFilter(size=3))
    # Convert the image in RGB back
    image = img_eroded.convert('RGB')

    # Convert the image to grayscale for contour detection
    img_gray = image.convert('L')

    # Convert the grayscale image to a NumPy array of dtype uint8
    img_np = np.array(img_gray, dtype=np.uint8)

    _, binary_img = cv2.threshold(img_np, 128, 255, cv2.THRESH_BINARY)

    # Find contours in the image using OpenCV
    contours, _ = cv2.findContours(binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    def extract_corners_from_contour(contour):
        epsilon = 0.03 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        return approx.squeeze()

    all_corners = [extract_corners_from_contour(contour) for contour in contours]

    vertices_list = []
    for contour in all_corners:
        # Extract vertices from each contour
        for point in contour:
            x, y = point
            if 0 < x < width - 1 and 0 < y < height - 1:
                vertices_list.append((abs(x), abs(y - height)))  # keep the right orientation

    def merge_close_corners(corners, distance_threshold):
        # Use KMeans clustering to merge close corners
        kmeans = KMeans(n_clusters=len(corners), init=np.array(corners), n_init=1)
        kmeans.fit(np.array(corners))
        clustered_corners = kmeans.cluster_centers_

        # Filter out corners that are too close to each other
        merged_corners = []
        merged_indices = []
        for i, corner in enumerate(clustered_corners):
            if i not in merged_indices:
                merged_group = [corner]
                for j, other_corner in enumerate(clustered_corners[i + 1:], start=i + 1):
                    if np.linalg.norm(corner - other_corner) < distance_threshold:
                        merged_group.append(other_corner)
                        merged_indices.append(j)
                avg_corner = np.mean(np.array(merged_group), axis=0)
                merged_corners.append(avg_corner)

        return merged_corners

    # Merge close corners using KMeans clustering
    merged_corners = merge_close_corners(vertices_list, 10)

    return merged_corners

# Utilisation
# image_path = "images/Rec.png"
# vertices = image_analyse(image_path)
