import cv2
import numpy as np
from PIL import Image
from scipy.stats import mode

import cv2
import numpy as np
from scipy.stats import mode


def process_polygon(polygon, cv_image, join_method):
    """
    Processes the pixels within a given polygon on an image according to the join_method.

    Parameters:
        polygon (list of tuples): List of (x, y) coordinates defining the polygon.
        cv_image (numpy.ndarray): OpenCV image in which the polygon resides.
        join_method (str): Method to process pixels. Options are 'additive', 'binary', 'modal', 'average', 'median'.

    Returns:
        numpy.ndarray: Processed OpenCV image.
    """
    # Create a mask for the polygon area
    mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)
    cv2.fillPoly(mask, [np.array(polygon, dtype=np.int32)], 1)

    # Get indices of the pixels within the polygon
    indices = np.where(mask == 1)

    # Extract the pixels within the polygon
    masked_pixels = cv_image[indices]  # Shape: (num_pixels, 3)

    if join_method == 'none':
        return cv_image

    elif join_method == 'additive':
        # Sum the color channels for each pixel
        sum_pixels = np.sum(masked_pixels, axis=1)
        # Clip values to max 255 to prevent overflow
        sum_pixels = np.clip(sum_pixels, 0, 255).astype(np.uint8)
        # Create a grayscale version by stacking the sums into 3 channels
        new_pixel_values = np.stack([sum_pixels]*3, axis=1)
        # Update the pixel values in the image
        cv_image[indices] = new_pixel_values

    elif join_method == 'binary':
        # Convert the image to grayscale
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # Compute the mean intensity within the polygon
        mean_intensity = np.mean(gray_image[mask == 1])
        # Apply binary thresholding
        _, binary_mask = cv2.threshold(gray_image, mean_intensity, 255, cv2.THRESH_BINARY)
        # Update the polygon area with the binary values in all channels
        binary_values = binary_mask[indices].astype(np.uint8)
        new_pixel_values = np.stack([binary_values]*3, axis=1)
        cv_image[indices] = new_pixel_values

    elif join_method == 'modal':
        # Compute the mode of the color channels for each pixel
        # Since mode might not be meaningful for 3 unique values, we'll pick the value that appears most frequently
        # across the channels. If all are unique, we'll pick the first channel.
        modal_pixels = mode(masked_pixels, axis=1)[0].flatten()
        new_pixel_values = np.stack([modal_pixels]*3, axis=1)
        cv_image[indices] = new_pixel_values

    elif join_method == 'average':
        # Compute the average of the color channels for each pixel
        mean_pixels = np.mean(masked_pixels, axis=1).astype(np.uint8)
        # Create a grayscale version by stacking the means into 3 channels
        new_pixel_values = np.stack([mean_pixels]*3, axis=1)
        # Update the pixel values in the image
        cv_image[indices] = new_pixel_values

    elif join_method == 'median':
        # Compute the median of the color channels for each pixel
        median_pixels = np.median(masked_pixels, axis=1).astype(np.uint8)
        # Create a grayscale version by stacking the medians into 3 channels
        new_pixel_values = np.stack([median_pixels]*3, axis=1)
        # Update the pixel values in the image
        cv_image[indices] = new_pixel_values

    else:
        raise ValueError(f"Unknown join_method: {join_method}")

    return cv_image




def pil_to_cv2(pil_image):
    # Convert PIL Image to OpenCV image
    cv_image = np.array(pil_image)
    # Convert RGB to BGR
    cv_image = cv_image[:, :, ::-1].copy()
    return cv_image


if __name__ == "__main__":
    import time

    for join_method in ['additive', 'binary', 'modal', 'average', 'median']:
        start_time = time.time()

        # Define the polygon points (x, y tuples) in order
        # Ensure they are ordered either clockwise or counterclockwise
        polygon = [(200, 600), (1100, 600), (900, 200), (400, 100)]

        # Path to your image
        image_path = 'beach.jpg'  # Replace with your actual image path
        pil_image = Image.open(image_path).convert('RGB')
        cv_image = pil_to_cv2(pil_image)

        # Execute conversion
        cv_image = process_polygon(polygon, cv_image, join_method)
        final_cv_image = process_polygon(polygon, cv_image, 'binary')

        # Convert back to PIL Image and save/display
        final_pil = Image.fromarray(cv2.cvtColor(final_cv_image, cv2.COLOR_BGR2RGB))
        final_pil.save('processed_selection.jpg')
        final_pil.show()


        end_time = time.time()
        elapsed_time = end_time - start_time
        print(f"Elapsed time: {round(elapsed_time,3)} seconds for {join_method}")


