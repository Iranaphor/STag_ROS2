import os
from PIL import Image
import stag_ros2.marker_generators.utils as utils



def generate_hue_marker(input_file, output_file, base_colour, detail_colour, threshold=128):

    # Load the image and convert to grayscale
    img = Image.open(input_file).convert("L")

    # Apply threshold to create a binary mask
    binary_mask = img.point(lambda x: 255 if x >= threshold else 0).convert('1')

    # Create solid color images for base and detail colors
    base_image = Image.new('RGB', img.size, base_colour)
    detail_image = Image.new('RGB', img.size, detail_colour)

    # Composite the images using the binary mask
    new_image = Image.composite(detail_image, base_image, binary_mask)

    # Save the new image
    new_image.save(output_file)



def generate_greyscale_marker(input_file, output_file, base_shade, detail_shade):

    # Load the images
    img = Image.open(input_file)

    # Convert to Binary
    binary = utils.convert_to_binary(img)
    threshold = 128

    # Define a lookup table to map each pixel
    # Pixels below threshold -> back_shade
    # Pixels equal to or above threshold -> front_shade
    lut = [base_shade if i < threshold else detail_shade for i in range(256)]

    # Apply the lookup table to the image
    new_image = binary.point(lut)

    # Save new image
    new_image.save(output_file)  # Save the combined image



def generate_hue_greyscale_markers(input_set, input_dir, output_dir, colour=None):
    if not colour: colour=[]

    # Load directory
    files = utils.list_files(input_dir)
    print(files)

    # Apply Chromatic Shades
    if len(colour) > 0:
        shades = colour #[(colour['base'],colour['detail']), (colour['detail'],colour['base'])]
        output_dir += "chromatic/"

    # Apply Shades Pairs
    else:
        s1 = sorted(list(range(0,101,10))+list(range(41,50,1))+list(range(51,60,1)))
        s2 = [100-s for s in s1]
        shades = [(int((s1[i]/100)*255),int((s2[i]/100)*255)) for i in range(len(s1))]

    # Generate markers
    total = 3
    print(str(shades))
    shadetotal = len(str(len(shades)))
    for f in files:
        for i, s in enumerate(shades):

            # Calculate marker id
            base_shade, detail_shade = s[0], s[1]
            if type(s[0]) == tuple:
                bs = f'{s[0][0]:02X}{s[0][1]:02X}{s[0][2]:02X}'
                ds = f'{s[1][0]:02X}{s[1][1]:02X}{s[1][2]:02X}'
            else:
                bs, ds = str(base_shade).zfill(total), str(detail_shade).zfill(total)
            uuid = f"{f.replace('.png','')}-{bs}-{ds}.png"
            print(f, i, s, uuid)

            # Combine channels
            output_file = f"{output_dir}HG{input_set}-{uuid}"
            if type(s[0]) == tuple:
                generate_hue_marker(f"{input_dir}{f}", output_file, base_shade, detail_shade)
                pass
            else:
                generate_greyscale_marker(f"{input_dir}{f}", output_file, base_shade, detail_shade)



if __name__ == '__main__':
    import os
    input_set = '23'
    input_dir = f"{os.getenv('HOME')}/STag-Markers/standard/HD{input_set}/"
    output_dir = f"{os.getenv('HOME')}/STag-Markers/hue-greyscale/HG{input_set}/"

    colour = [((255,0,0),(0,0,0)),
              ((255,222,0),(54,124,43)),
              ((0,69,124),(0,121,193))]
    #colour = None

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    if colour and not os.path.exists(output_dir+"chromatic/"):
        os.makedirs(output_dir+"chromatic/")
    generate_hue_greyscale_markers(input_set, input_dir, output_dir, colour)

    print('\nGenerated markers.\n')



