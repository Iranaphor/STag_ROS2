import os
from PIL import Image

import stag_ros2.marker_generators.utils as utils



def generate_greyscale_marker(input_file, output_file, base_shade, detail_shade):

    # Load the images
    img = Image.open(input_file)

    # Convert to Binary
    binary = utils.convert_to_binary(img)
    threshold = 128

    # Input Validation
    if not (0 <= base_shade <= 255):
        raise ValueError("base_shade must be an integer between 0 and 255.")
    if not (0 <= detail_shade <= 255):
        raise ValueError("detail_shade must be an integer between 0 and 255.")
    if not (0 <= threshold <= 255):
        raise ValueError("threshold must be an integer between 0 and 255.")

    # Define a lookup table to map each pixel
    # Pixels below threshold -> back_shade
    # Pixels equal to or above threshold -> front_shade
    lut = [base_shade if i < threshold else detail_shade for i in range(256)]

    # Apply the lookup table to the image
    new_image = binary.point(lut)

    # Save new image
    new_image.save(output_file)  # Save the combined image



def generate_hue_greyscale_markers(input_set, input_dir, output_dir):
    # Load directory
    files = utils.list_files(input_dir)
    print(files)

    # Get hamming distance sizes
    m = input_set.replace('HD','')

    # Shade Pairs
    s1 = sorted(list(range(0,101,10))+list(range(41,50,1))+list(range(51,60,1)))
    s2 = [100-s for s in s1]
    shades = [(s1[i],s2[i]) for i in range(len(s1))]

    # Generate markers
    for f in files:
        for i, s in enumerate(shades):
            print(f, i, s)

            # Calculate marker id
            total = 3
            shadetotal = len(str(len(shades)))
            base_shade, detail_shade = int((s[0]/100)*255), int((s[1]/100)*255)
            bs, ds = str(base_shade).zfill(total), str(detail_shade).zfill(total)
            uuid = f"{f.replace('.png','')}-{bs}-{ds}"

            # Combine channels
            output_file = f"{output_dir}HG{m}-{uuid}.png"
            generate_greyscale_marker(f"{input_dir}{f}", output_file, base_shade, detail_shade)



if __name__ == '__main__':
    import os
    input_set = 'HD23'
    input_dir = f"{os.getenv('HOME')}/STag-Markers/standard/HD23/"
    output_dir = f"{os.getenv('HOME')}/STag-Markers/hue-greyscale/HG23/"

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    generate_hue_greyscale_markers(input_set, input_dir, output_dir)

    print('\nGenerated markers.\n')



