from PIL import Image
import numpy as np
import cv2

def pil_to_cv2(pil_image):
    """Convert PIL Image to OpenCV Image (BGR)."""
    cv_image = np.array(pil_image)
    # Convert RGB to BGR
    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
    return cv_image

def cv2_to_pil(cv_image):
    """Convert OpenCV Image (BGR) to PIL Image (RGB)."""
    # Convert BGR to RGB
    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    pil_image = Image.fromarray(cv_image)
    return pil_image

def order_polygon(pts):
    """
    Order polygon points in clockwise order starting from the top-left point.
    This is essential for correct perspective transformations.
    """
    # Compute the centroid
    centroid = np.mean(pts, axis=0)
    # Compute angles from centroid to each point
    angles = np.arctan2(pts[:,1] - centroid[1], pts[:,0] - centroid[0])
    # Sort points based on angles
    sorted_indices = np.argsort(angles)
    ordered = pts[sorted_indices]

    # Ensure the first point is the top-left
    # Find the point with the smallest sum of coordinates
    sum_coords = ordered.sum(axis=1)
    top_left_index = np.argmin(sum_coords)
    ordered = np.roll(ordered, -top_left_index, axis=0)

    return ordered

def rotate_channel_section(cv_image, channel_index, polygon, angle):
    """
    Rotate a specific channel's polygonal section by a given angle and map it back.

    :param cv_image: OpenCV image (BGR).
    :param channel_index: Index of the channel to rotate (0=Blue, 1=Green, 2=Red).
    :param polygon: List of four (x, y) tuples defining the polygon's vertices in order.
    :param angle: Angle by which to rotate the channel (-90, -180 degrees).
    :return: Modified channel as a NumPy array.
    """
    # Extract the specified channel
    channel_img = cv_image[:, :, channel_index].copy()

    # Convert polygon to NumPy array of type float32
    polygon_np = np.array(polygon, dtype=np.float32)

    # Order polygon points consistently
    polygon_np = order_polygon(polygon_np)

    # Calculate width and height of the bounding rectangle
    width_a = np.linalg.norm(polygon_np[0] - polygon_np[1])
    width_b = np.linalg.norm(polygon_np[3] - polygon_np[2])
    max_width = max(int(width_a), int(width_b))

    height_a = np.linalg.norm(polygon_np[0] - polygon_np[3])
    height_b = np.linalg.norm(polygon_np[1] - polygon_np[2])
    max_height = max(int(height_a), int(height_b))

    # Define destination points for the perspective transform (unwarped rectangle)
    dst_rect = np.array([
        [0, 0],
        [max_width - 1, 0],
        [max_width - 1, max_height - 1],
        [0, max_height - 1]
    ], dtype=np.float32)

    # Compute the perspective transform matrix
    M = cv2.getPerspectiveTransform(polygon_np, dst_rect)

    # Warp the perspective to get the rectangular cropped image
    warped = cv2.warpPerspective(channel_img, M, (max_width, max_height))

    # Determine rotation flag based on angle
    # OpenCV's rotate function uses clockwise rotation
    angle = angle % 360
    if angle == 90 or angle == -270:
        rotate_flag = cv2.ROTATE_90_CLOCKWISE
    elif angle == 180 or angle == -180:
        rotate_flag = cv2.ROTATE_180
    elif angle == 270 or angle == -90:
        rotate_flag = cv2.ROTATE_90_COUNTERCLOCKWISE
    elif angle == 0:
        rotate_flag = None
    else:
        raise ValueError("Unsupported rotation angle. Use multiples of 90 degrees.")

    # Rotate the warped image
    if rotate_flag is not None:
        rotated = cv2.rotate(warped, rotate_flag)
    else:
        rotated = warped.copy()

    # After rotation, width and height may swap (for 90 and 270 degrees)
    if angle in [90, -270, 270, -90]:
        rotated_height, rotated_width = rotated.shape[:2]
    else:  # 0 or 180 degrees
        rotated_height, rotated_width = rotated.shape[:2]

    # Define new source points based on rotated image's size
    rotated_src_rect = np.array([
        [0, 0],
        [rotated_width - 1, 0],
        [rotated_width - 1, rotated_height - 1],
        [0, rotated_height - 1]
    ], dtype=np.float32)

    # Compute the inverse perspective transform matrix
    # Map the rotated image back to the original polygon
    M_inv = cv2.getPerspectiveTransform(rotated_src_rect, polygon_np)

    # Warp the rotated image back to the original polygon's perspective
    warped_back = cv2.warpPerspective(rotated, M_inv, (channel_img.shape[1], channel_img.shape[0]))

    # Create a mask of the polygon area
    mask = np.zeros((channel_img.shape[0], channel_img.shape[1]), dtype=np.uint8)
    cv2.fillConvexPoly(mask, polygon_np.astype(int), 255)

    # Invert mask for background
    mask_inv = cv2.bitwise_not(mask)

    # Black-out the area of the polygon in the original channel image
    image_bg = cv2.bitwise_and(channel_img, channel_img, mask=mask_inv)

    # Take only the warped_back region
    warped_foreground = cv2.bitwise_and(warped_back, warped_back, mask=mask)

    # Combine the background and warped foreground
    final_channel = cv2.add(image_bg, warped_foreground)

    return final_channel



def rotate_channels(polygon, cv_image):
    # Split the image into channels (B, G, R)
    channels = channels = list(cv2.split(cv_image))  # Convert to list to allow item assignment

    # Define rotations for specific channels
    # For example: Rotate Blue channel by -90 degrees and Green channel by -180 degrees
    # Note: In OpenCV, channel indices are 0=Blue, 1=Green, 2=Red
    rotations = {
        0: 180,    # Blue channel
        1: -90,   # Green channel
        # 2: 0,    # Red channel (no rotation)
    }

    # Process each specified channel
    for channel_index, angle in rotations.items():
        #print(f"Processing channel {channel_index} with rotation {angle} degrees")
        try:
            channels[channel_index] = rotate_channel_section(cv_image, channel_index, polygon, angle)
        except Exception as e:
            print(f"Error processing channel {channel_index}: {e}")

    # Merge the processed channels back into one image
    final_cv_image = cv2.merge(channels)
    return final_cv_image


if __name__ == "__main__":
    # Define the polygon points (x, y tuples) in order
    # Ensure they are ordered either clockwise or counterclockwise
    polygon = [(200, 600), (1100, 600), (900, 200), (400, 100)]

    # Path to your image
    image_path = 'beach.jpg'  # Replace with your actual image path

    # Load the image using PIL and convert to OpenCV format
    pil_image = Image.open(image_path).convert('RGB')
    cv_image = pil_to_cv2(pil_image)

    # Execute conversion
    final_cv_image = rotate_channels(cv_image, polygon)

    # Convert back to PIL Image and save/display
    final_pil = cv2_to_pil(final_cv_image)
    final_pil.save('rotated_section.png')
    final_pil.show()
    print("Rotated channels saved as 'rotated_section.png'.")

