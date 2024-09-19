import os
from pprint import pprint
from PIL import Image

# File paths
#HD = 11
#input_dir = "HD11/"
#output_dir = "HD11_HC/"

#HD = 13
#input_dir = "HD13/"
#output_dir = "HD13_HC/"

#HD = 15
#input_dir = "HD15/"
#output_dir = "HD15_HC/"

#HD = 17
#input_dir = "HD17/"
#output_dir = "HD17_HC/"

#HD = 19
#input_dir = "HD19/"
#output_dir = "HD19_HC/"

#HD = 21
#input_dir = "HD21/"
#output_dir = "HD21_HC/"

HD = 23
input_dir = "/home/james/Documents/STag-Markers/high-capacity/HD23/"
output_dir = "/home/james/Documents/STag-Markers/high-capacity/HD23_HC/"



if not os.path.exists(output_dir):
    os.makedirs(output_dir)


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

    # Create a new image from the R, G, B channels
    new_image = Image.merge("RGB", (r, g, b))
    #new_image.show()  # Show the combined image
    new_image.save(output_file)  # Save the combined image


# Test combination
#red_file = input_dir+"00001.png"
#green_file = input_dir+"00002.png"
#blue_file = input_dir+"00003.png"
#combine_rgb_channels(red_file, green_file, blue_file, output_dir)




def list_files(directory):
    # Get a list of all entries in the directory given
    entries = os.listdir(directory)

    # Filter out directories, only keep files
    files = [entry for entry in entries if os.path.isfile(os.path.join(directory, entry)) and entry.endswith('.png')]
    return files


files = sorted(list_files(input_dir))
print(files)

hd_r, hd_g, hd_b = len(files)**0, len(files)**1, len(files)**2

# Generate markers
for b_i, b_f in enumerate(files):
    for g_i, g_f in enumerate(files):
        for r_i, r_f in enumerate(files):
            #if not ((r_i == b_i) or (r_i == g_i) or (b_i == g_i)): continue

            # Calculate marker id
            uuid = (r_i*hd_r) + (g_i*hd_g) + (b_i*hd_b)
            print(f'M{uuid} | Format(R{r_i}-G{g_i}-B{b_i}))')

            # Combine channels
            output_file = f"{output_dir}HC{HD}-ID{uuid}.png"
            combine_rgb_channels(f"{input_dir}{r_f}", f"{input_dir}{g_f}", f"{input_dir}{b_f}", output_file)

print('\nGenerated markers.\n')
