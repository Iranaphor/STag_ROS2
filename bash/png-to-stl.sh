#!/bin/bash

# USES:
# @techreport{trace2scad20150104,
#     author={Henry Gordon Dietz},
#     title={{Trace2SCAD: Converting Images Into OpenSCAD Models}},
#     month={January},
#     year={2015},
#     institution={University of Kentucky},
#     howpublished={Aggregate.Org online technical report},
#     URL={http://aggregate.org/MAKE/TRACE2SCAD/}
# }

# Default values for parameters
default_max_thickness=1
default_thickness_ratio=0.5
default_square_size=20

# Function to display help message
show_help() {
    echo "Usage: $0 <input_directory> [max_thickness] [thickness_ratio] [square_size]"
    echo "Converts PNG images into STL files using OpenSCAD."
    echo
    echo "Arguments:"
    echo "  input_directory     The directory containing PNG files to process."
    echo "  max_thickness       Height of model from back to face in mm (default: $default_max_thickness)."
    echo "  thickness_ratio     Percentage of height to use for details (default: $default_thickness_ratio)."
    echo "  square_size         Size of the square for the base in mm (default: $default_square_size)."
    echo
    exit 0
}

# Check if help is requested
if [ "$1" == "-h" ] || [ "$1" == "--help" ]; then
    show_help
fi

# Check if the input directory is provided
if [ -z "$1" ]; then
    echo "Error: Input directory is required."
    show_help
fi

# Define input directory and output directory
input_dir="$1"
output_dir="${input_dir}_stl"

# Assign default values for max_thickness, thickness_ratio, and square_size
max_thickness=${2:-$default_max_thickness}
thickness_ratio=${3:-$default_thickness_ratio}
square_size=${4:-$default_square_size}

# Create the output directory if it doesn't exist
mkdir -p "$output_dir"

# Process each .png file in the input directory
for img in "$input_dir"/*.png; do
    # Extract the base name (without extension)
    base_name=$(basename "$img" .png)

    # Convert PNG to BMP using ImageMagick
    bmp_file="$output_dir/B$base_name.bmp"
    convert "$img" "$bmp_file"

    scad_file="$output_dir/$base_name.scad"
    ./trace2scad.sh -f 0 -o $scad_file $bmp_file 

    # Create OpenSCAD script to extrude the DXF
    echo "max_thickness = $max_thickness;" >> "$scad_file"
    echo "thickness_ratio = $thickness_ratio;" >> "$scad_file"
    echo "square_size = $square_size;" >> "$scad_file"
    echo "difference(){" >> "$scad_file"
    echo "translate([0,0,0.5*max_thickness])" >> "$scad_file"
    echo "linear_extrude(max_thickness,center=true)" >> "$scad_file"
    echo "square(size=square_size, center = true);" >> "$scad_file"
    echo "translate([0,0,(1-thickness_ratio)*max_thickness])" >> "$scad_file"
    echo "scale([square_size,square_size,thickness_ratio*max_thickness]) B$base_name();" >> "$scad_file"
    echo "}" >> "$scad_file"

    # Generate the STL file using OpenSCAD
    stl_file="$output_dir/$base_name.stl"
    openscad -o "$stl_file" "$scad_file"

    # break

    # Remove intermediate files if they exist
    [ -f "$scad_file" ] && rm "$scad_file"
    [ -f "$bmp_file" ] && rm "$bmp_file"

    echo "Processed $img -> $stl_file"
done

echo "All PNG images have been processed and saved in $output_dir."
