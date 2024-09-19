import os
from PIL import Image


def convert_to_binary(img, threshold=128):
    # Convert the image to grayscale to simplify thresholding
    grayscale = img.convert('L')

    # Apply the threshold to convert the image to binary in grayscale
    binary = grayscale.point(lambda x: 255 if x > threshold else 0, '1')

    # Convert the binary "1" mode image back to "L" mode (grayscale)
    binary_l = binary.convert('L')

    # Return the result
    return binary_l


def list_files(directory):
    # Get a list of all entries in the directory given
    entries = os.listdir(directory)

    # Filter out directories, only keep files
    files = [entry for entry in entries if os.path.isfile(os.path.join(directory, entry)) and entry.endswith('.png')]
    return sorted(files)

