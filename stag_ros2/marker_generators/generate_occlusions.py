import os
import shutil
import random
import numpy as np
from PIL import Image, ImageDraw

import stag_ros2.marker_generators.utils as utils


def generate_masks(masks_dir, total_masks=10, min_size=100, max_size=100, total_occlusions=1):

    # Remove contents of directory if it exists
    if os.path.exists(masks_dir):
        shutil.rmtree(masks_dir)

    # Make directory if it does not exist
    if not os.path.exists(masks_dir):
        os.makedirs(masks_dir)

    # Construst masks
    w, h = 1000, 1000
    for m in range(total_masks):
        q = total_occlusions

        mask = Image.new("L", (w, h), 0)

        for i in range(q):
            x = random.randint(0,w)
            y = random.randint(0,h)
            r = random.randint(min_size, max_size)

            fill_value = 255
            mask = utils.draw_occlusion(mask, x, y, r, fill_value)

        mt = str(m).zfill(len(str(total_masks)))
        xt = str(x).zfill(len(str(w)))
        yt = str(y).zfill(len(str(h)))
        rt = str(r).zfill(len(str(max_size)))
        mask_file = os.path.join(masks_dir, f"{mt}-x{xt}-y{yt}-r{rt}.png")
        mask.save(mask_file)



def apply_mask(image, mask, rgb_tuple):


    mask_bool = mask == 255  # Shape: (H, W)

    # Create a copy of the image to avoid modifying the original
    result = image.copy()

    # Get the indices where the mask is True
    indices = np.where(mask_bool)

    # Assign the RGB color to those indices for all color channels
    result[indices[0], indices[1], :] = rgb_tuple

    return result


if __name__ == '__main__':

    # 1. Generate masks and save to directory
    masks_dir = f"{os.getenv('HOME')}/STag-Markers/hue-greyscale/HG23_occluded/"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    generatemasks(masks_dir)

    # 2. Go through each input dir and mask each image with each mask
    input_dir = f"{os.getenv('HOME')}/STag-Markers/standard/HD23/"
    output_dir = f"{os.getenv('HOME')}/STag-Markers/occluded/HD23/"

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # 3. Apply masks for each file in input

















