The PNG to STL tool uses http://aggregate.org/MAKE/TRACE2SCAD/ to work.
Please install all the necessary dependencies - including OpenSCAD.

Assuming you have STag markers in a specified directory, e.g., `HD19`, you can run

```
./png-to-stl.sh ./HD19
```

and the tool will automatically convert the markers from PNG form to STL and place them in `HD19_stl`.

Run `./png-to-stl.sh --help` for information on how to make basic changes to the STL.
Intercepting one of the temporary OpenSCAD files and experimenting with it before using the script to generate STLs is advised.