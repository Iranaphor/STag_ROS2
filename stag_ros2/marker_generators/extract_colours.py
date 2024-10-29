import sys
from PIL import Image
import numpy as np
from scipy.ndimage import binary_erosion

def extract_color_binary(image_path, target_color, threshold=60, structure=None):
    """
    Extracts pixels close to the target color from an image and creates a binary image,
    then applies morphological erosion to reduce artifacting.

    Parameters:
    - image_path (str): Path to the input image.
    - target_color (tuple): The RGB color to match (e.g., (255, 0, 0) for red).
    - threshold (int): Distance threshold to determine 'closeness' to the target color.
    - structure (ndarray): Structuring element for morphological operations.

    Returns:
    - PIL.Image: Binary image after morphological erosion.
    """
    # Open image and convert to RGB
    image = Image.open(image_path).convert('RGB')
    # Convert image to numpy array
    data = np.array(image)
    # Compute distance from target color
    distance = np.linalg.norm(data - target_color, axis=2)
    # Create binary mask where distance is less than threshold
    mask = distance < threshold
    # Apply morphological erosion to the mask
    eroded_mask = binary_erosion(mask, structure=structure)
    # Create binary image: white where mask is True, black elsewhere
    binary_image = np.zeros_like(data[:, :, 0], dtype=np.uint8)
    binary_image[eroded_mask] = 255
    # Convert to PIL Image and return
    return Image.fromarray(binary_image)

# Define target colors
colors = {
    'red': (255, 0, 0),
    'green': (0, 255, 0),
    'blue': (0, 0, 255),
    'cyan': (0, 255, 255),
    'yellow': (255, 255, 0),
    'purple': (255, 0, 255),
    'white': (255, 255, 255),
    'black': (0, 0, 0)
}

if __name__ == '__main__':
    # Get filepath
    if len(sys.argv) < 2:
        print("Usage: python script_name.py input_image.png")
        sys.exit(1)
    file = sys.argv[1]

    # Define the structuring element for morphological erosion
    # For example, a 3x3 square
    structuring_element = np.ones((3, 3), dtype=bool)

    # Process the image for each color
    input_image_path = file
    for color_name, color_value in colors.items():
        # Adjust threshold for white and black
        if color_name in ['white', 'black']:
            threshold = 80  # Higher threshold for white and black
        else:
            threshold = 60
        binary_image = extract_color_binary(
            input_image_path,
            color_value,
            threshold,
            structure=structuring_element
        )
        binary_image.save(f'binary_{color_name}.png')
