
# Installation
The PNG to STL tool uses http://aggregate.org/MAKE/TRACE2SCAD/ to function. 
Please install all the necessary dependencies - including OpenSCAD:
```
sudo apt install imagemagick
sudo apt install potrace
sudo apt install openscad
```

Clone the repository containing the scripts:
```
git clone https://github.com/Iranaphor/stag_ros2.git
```

Download the marker files from the Google Drive Marker Folder.
Extraxt the zip folder into a directory of your choosing:
```
https://drive.google.com/drive/folders/0ByNTNYCAhWbIemJDbEpXTmRncnM?resourcekey=0-OpEpUjxCopVUlSSSZ8cOoA
```


# Execution
To run, first `cd` to the bash directory containing `./png-to-stl.sh`:
```
cd .../stag_ros2/bash/
```

Determine the directory containing the markers:
```
marker_dir=~/path/to/markers/directory/HD19/
```

Run the script, passing in the directory of the markers and any additional arguments.
```
./png-to-stl.sh $marker_dir
```

The tool will automatically convert the markers from PNG form to STL and place them in a directory named as:
```
${marker_dir}_stl
```


# Further Help
Run `./png-to-stl.sh --help` for information on how to make basic changes to the STL.
Intercepting one of the temporary OpenSCAD files and experimenting with it before using the script to generate STLs is advised.
