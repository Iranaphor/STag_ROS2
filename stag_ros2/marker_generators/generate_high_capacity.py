import os
import sympy
from pprint import pprint
from PIL import Image


# File paths
input_dir = "HD11/"
output_dir = "HD11_HC/"
max_marker_id = (22335**3)+100

input_dir = "HD13/"
output_dir = "HD13_HC/"
max_marker_id = (2884**3)+100

#input_dir = "HD15/"
#output_dir = "HD15_HC/"
#max_marker_id = (766**3)+100

#input_dir = "HD17/"
#output_dir = "HD17_HC/"
#max_marker_id = (157**3)+100

#input_dir = "HD19/"
#output_dir = "HD19_HC/"
#max_marker_id = (38**3)+100

#input_dir = "HD21/"
#output_dir = "HD21_HC/"
#max_marker_id = (12**3)+100

#input_dir = "HD23/"
#output_dir = "HD23_HC/"
#max_marker_id = (6**3)+100


DONT_RENDER_OUTPUT_IMAGES=True


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


files = list_files(input_dir)
print(files)

HD = dict()
for f in files:
    idx = int(f.split('.png')[0])
    HD[idx] = {
        'filepath':f"{input_dir}{f}",
        'prime_red':sympy.prime((idx*3)+0) if idx != 0 else 1,
        'prime_green':sympy.prime((idx*3)+1),
        'prime_blue':sympy.prime((idx*3)+2)
    }
print(HD)


def prime_factors_sympy(n):
    # Using sympy.factorint() which returns a dictionary
    # where the keys are the prime factors and the values are their respective powers.
    factors_dict = sympy.factorint(n)

    # To list the factors in the form of base and exponent pairs
    factors_expanded = []
    for prime, exp in factors_dict.items():
        factors_expanded.extend([prime] * exp)

    return factors_expanded


count = 0
for m in range(1,max_marker_id+1):
    # Identify prime factors
    f = prime_factors_sympy(m)

    if len(f) == 0:
        f += [1,1,1]

    if len(f) == 1:
        f += [1,1]

    if len(f) == 2:
        f += [1]

    if len(f) > 3:
        print(f'M{m} | {len(f)} Factors, Too Many, Skipping')
        count += 1
        continue

    # Identify files for the factors
    if not all(i in HD for i in f):
        missing = [i for i in f if i not in HD]
        print(f'M{m} | Factors Missing: {missing}')
        count += 1
        continue

    if DONT_RENDER_OUTPUT_IMAGES:
        continue

    print(f'M{m} | {len(f)} Real Factors: {f}')
    r, g, b = HD[f[0]], HD[f[1]], HD[f[2]]

    # Combine channels
    uuid = r['prime_red'] * g['prime_green'] * b['prime_blue']
    output_file = f"{output_dir}HC{m}-ID{uuid}.png"
    combine_rgb_channels(r['filepath'], g['filepath'], b['filepath'], output_file)

print(f'\nOf {max_marker_id} attempts, {count} skipped leaving {max_marker_id-count}\n')
#pprint(HD)
