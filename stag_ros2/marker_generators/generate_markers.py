import argparse
import os
import sys

from stag_ros2.marker_generators.high_capacity import generate_high_capacity_markers
from stag_ros2.marker_generators.hue_greyscale import generate_hue_greyscale_markers
from stag_ros2.marker_generators.high_occlusion import generate_high_occlusion_markers

def main():
    # Create the parser
    parser = argparse.ArgumentParser(description="Process input and output directories and sets.")

    # Add arguments
    parser.add_argument(
        '--input_dir',
        type=str,
        required=True,
        help='Path to the input directory.'
    )
    parser.add_argument(
        '--output_dir',
        type=str,
        required=True,
        help='Path to the output directory.'
    )
    parser.add_argument(
        '--input_set',
        type=str,
        required=True,
        help='Name of the input set.'
    )
    parser.add_argument(
        '--output_set',
        type=str,
        required=True,
        help='Name of the output set.'
    )
    parser.add_argument(
        '--sample_size',
        type=int,
        required=False,
        help='Number of markers to generate empty for All.'
    )

    # Parse the arguments
    args = parser.parse_args()

    # Check if input_dir exists and is a directory
    if not os.path.isdir(args.input_dir):
        print(f"Error: The input directory '{args.input_dir}' does not exist or is not a directory.", file=sys.stderr)
        sys.exit(1)

    # Check if output_dir exists; if not, create it
    if not os.path.exists(args.output_dir):
        try:
            os.makedirs(args.output_dir)
            print(f"Output directory '{args.output_dir}' created successfully.")
        except Exception as e:
            print(f"Error: Could not create output directory '{args.output_dir}'. {e}", file=sys.stderr)
            sys.exit(1)
    else:
        if not os.path.isdir(args.output_dir):
            print(f"Error: The output path '{args.output_dir}' exists but is not a directory.", file=sys.stderr)
            sys.exit(1)

    # Check the input set is valid
    valid_hd_sets = ['HD11', 'HD13', 'HD15', 'HD17', 'HD19', 'HD21', 'HD23']
    if args.input_set not in valid_hd_sets:
        print(f"Error: The input set '{args.input_set}' is not valid, use one of {str(valid_hd_sets)}.", file=sys.stderr)
        sys.exit(1)


    # Check output set is valid
    valid_prefixes = ('HC', 'HG', 'HO')
    if not any(args.output_set.startswith(prefix) for prefix in valid_prefixes):
        raise argparse.ArgumentTypeError(f"Invalid output_set '{args.output_set}'. It must start with one of {valid_prefixes}.")

    # Print the arguments
    print("Input Directory:", args.input_dir)
    print("Output Directory:", args.output_dir)
    print("Input Set:", args.input_set)
    print("Output Set:", args.output_set)

    # Process args
    if args.output_set.startswith('HC'):
        generate_high_capacity_markers(args.input_set, args.input_dir, args.output_dir)
    elif args.output_set.startswith('HG'):
        generate_hue_greyscale_markers(args.input_set, args.input_dir, args.output_dir)
    elif args.output_set.startswith('HO'):
        generate_high_occlusion_markers(args.input_set, args.input_dir, args.output_dir, sample_size=args.sample_size)

if __name__ == "__main__":
    #import os
    #marker_set = 'HC23'
    #input_dir = f"{os.getenv('HOME')}/STag-Markers/HD23/"
    #output_dir = f"{os.getenv('HOME')}/STag-Markers/HC23/"
    main()


















