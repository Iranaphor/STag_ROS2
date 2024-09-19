import os
from PIL import Image

import stag_ros2.marker_generators.utils as utils


def combine_rgb_channels(red_file, green_file, blue_file, output_file):
    # Load the images
    red_img = Image.open(red_file)
    green_img = Image.open(green_file)
    blue_img = Image.open(blue_file)

    # Convert to Binary
    r = utils.convert_to_binary(red_img)
    g = utils.convert_to_binary(green_img)
    b = utils.convert_to_binary(blue_img)

    # Rotate images
    r = r.rotate(0, expand=True)
    g = g.rotate(-90, expand=True)
    b = b.rotate(-180, expand=True)

    # Create a new image from the R, G, B channels
    new_image = Image.merge("RGB", (r, g, b))
    new_image.save(output_file)  # Save the combined image


def generate_high_occlusion_markers(input_set, input_dir, output_dir):

    # Identify input files
    m = input_set.replace('HD','')
    files = utils.list_files(input_dir)
    print(files)

    # Combine channels
    for f in files:
        total = len(str(len(files)))
        uuid = f.replace('.png','')
        input_file = f"{input_dir}{f}"

        output_file = f"{output_dir}HO{m}-ID{str(uuid).zfill(total)}.png"
        combine_rgb_channels(red_file=input_file, green_file=input_file, blue_file=input_file, output_file=output_file)


if __name__ == '__main__':
    import os
    input_set = 'HD23'
    input_dir = f"{os.getenv('HOME')}/STag-Markers/standard/HD23/"
    output_dir = f"{os.getenv('HOME')}/STag-Markers/high-occlusion/HC23/"

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    generate_high_occlusion_markers(input_set, input_dir, output_dir)

