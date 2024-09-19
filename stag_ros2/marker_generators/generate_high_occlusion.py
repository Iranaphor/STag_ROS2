import os
from PIL import Image


# File paths
#input_dir = "HD11/"
#output_dir = "HD11_HO/"

#input_dir = "HD13/"
#output_dir = "HD13_HO/"

#input_dir = "HD15/"
#output_dir = "HD15_HO/"

#input_dir = "HD17/"
#output_dir = "HD17_HO/"

#input_dir = "HD19/"
#output_dir = "HD19_HO/"

#input_dir = "HD21/"
#output_dir = "HD21_HO/"

input_dir = "HD23/"
output_dir = "HD23_HO/"

input_dir = "/home/jheselden/Pictures/HD23/HD23/"
output_dir = "/home/jheselden/Pictures/HD23/HD23_HO/"





def list_files(directory):
    # Get a list of all entries in the directory given
    entries = os.listdir(directory)

    # Filter out directories, only keep files
    files = [entry for entry in entries if os.path.isfile(os.path.join(directory, entry)) and entry.endswith('.png')]
    return files




def convert_to_binary(img, threshold=128):
    # Convert the image to grayscale to simplify thresholding
    grayscale = img.convert('L')

    # Apply the threshold to convert the image to binary in grayscale
    binary = grayscale.point(lambda x: 255 if x > threshold else 0, '1')

    # Convert the binary "1" mode image back to "L" mode (grayscale)
    binary_l = binary.convert('L')

    # Return the result
    return binary_l



def combine_rgb_channels(red_file, green_file, blue_file, output_file):
    # Load the images
    red_img = Image.open(red_file)
    green_img = Image.open(green_file)
    blue_img = Image.open(blue_file)

    # Convert to Binary
    r = convert_to_binary(red_img)
    g = convert_to_binary(green_img)
    b = convert_to_binary(blue_img)

    # Rotate images
    r = r.rotate(0, expand=True)
    g = g.rotate(-90, expand=True)
    b = b.rotate(-180, expand=True)

    # Create a new image from the R, G, B channels
    new_image = Image.merge("RGB", (r, g, b))
    new_image.save(output_file)  # Save the combined image


# Create output folder
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# Identify input files
m = input_dir.split('HD')[-1].split('/')[0]
files = list_files(input_dir)
print(files)

# Combine channels
for f in files:
    uuid = f.replace('.png','')
    input_file = f"{input_dir}{f}"
    output_file = f"{output_dir}HO{m}-ID{uuid}.png"
    combine_rgb_channels(input_file, input_file, input_file, output_file)


