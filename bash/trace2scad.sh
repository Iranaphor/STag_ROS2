#!/bin/bash
# trace2scad
# 2015 by Hank Dietz, http://aggregate.org/hankd/
# This script is distributed as full source subject
# to the Creative Commons - Attribution constraints.
# You use it at your own risk, with no warranty, etc.
version=20150415
complex=4096
filter=2
gamma=1
pixerr=1.0
xres=2000
yres=2000
level=0.48
levels=$level
negate=""
turd=5
subres=5
align=0
verbose=""
prefix="no_prefix_set"
ppm='/tmp/trace2scad'$$'.ppm'
pgm='/tmp/trace2scad'$$'.pgm'
svg='/tmp/trace2scad'$$'.svg'
raw='/tmp/trace2scad'$$'.raw'
stmp='/tmp/trace2scad'$$'.scad'
scad=""
myname="$0"

if [[ $# < 1 ]]
then
	echo "Error: type $myname -h for usage" 1>&2
	exit 1
fi

while [[ $# > 0 ]]
do
	key="$1"
	shift
	case $key in
		-a|--align)
			align=1
			;;
		-c|--complex)
			complex="$1"
			shift
			;;
		-e|--pixerr)
			pixerr="$1"
			shift
			;;
		-f|--filter)
			filter="$1"
			shift
			;;
		-g|--gamma)
			gamma="$1"
			shift
			;;
		-x|--xres)
			xres="$1"
			shift
			;;
		-y|--yres)
			yres="$1"
			shift
			;;
		-s|--subres)
			subres="$1"
			shift
			;;
		-t|--turd)
			turd="$1"
			shift
			;;
		-o)
			scad="$1"
			shift
			;;
		-n)
			negate="-negate"
			;;
		-p|--prefix)
			prefix="${1%.*}" # allow use of filename, but strip .ext
			shift
			;;
		-l|--level|--levels)
			levels="$1"
			# if integer number of levels was given,
			# convert that to a list of thresholds
			# evenly spaced between 1 and 0
			if [[ ${levels#*.} = $levels ]]
			then
				levels=`awk -v levels=$levels '
BEGIN	{
		s = (levels-0.5)/levels
		for (i=levels-1; i>0; --i) {
			s = s "," ((i-0.5)/levels)
		}
		printf s
	}
' </dev/null`
			fi
			shift
			;;
		-v|--verbose)
			verbose="true"
			echo "verbose=true" 1>&2
			;;
		-h|--help)
			echo "Usage: $myname {options} inputimagefile" 1>&2
			echo "-a	force alignment based on image (not object bounds)" 1>&2
			echo "-c #	set target max complexity (default $complex)" 1>&2
			echo "-e #	set max line-combining error in pixels (default $pixerr)" 1>&2
			echo "-f #	set highpass filter radius (default $filter)" 1>&2
			echo "-g #	set gamma (default $gamma)" 1>&2
			echo "-h	show this help message" 1>&2
			echo "-l levs	set level threshold, thresholds, or number of layers (default $levels)" 1>&2
			echo "-n	negate the image" 1>&2
			echo "-o file	set OpenSCAD output filename to file (default from input filename)" 1>&2
			echo "-p pre	set prefix for OpenSCAD module names (default from input filename)" 1>&2
			echo "-s #	set subpixel potrace resolution (default $subres)" 1>&2
			echo "-t #	set potrace turd size (default $turd)" 1>&2
			echo "-v	enable verbose output, keep temp files" 1>&2
			echo "-x #	set X resolution (default $xres)" 1>&2
			echo "-y #	set Y resolution (default $yres)" 1>&2
			exit
			;;
		*)
			if [ -n "$infile" ]
			then
				echo "Error: can only have one input filename; type $myname -h for usage" 1>&2
				exit 1
			fi
			infile="$key"
			if [ $prefix = "no_prefix_set" ]
			then
				prefix="${key%.*}" # strip .ext
				prefix="${prefix##*/}" # strip leading path
				prefix=`echo $prefix |tr '-' '_' |tr -cd '[_a-zA-Z0-9]'`
			fi
			if [ -z "$scad" ]
			then
				scad="${key%.*}.scad" # strip .ext, add .scad
			fi
			shift
			;;
	esac
done

if [ -z "$infile" ]
then
	echo "Error: no input filename given; type $myname -h for usage" 1>&2
	exit 1
fi

# remember reference image size
refxres=$xres
refyres=$yres

# empty the output file
>$scad

# process multiple-level stuff
curlev=0
if [ -n "${levels#*,}" ]
then
	align=1		# force center based on image, not object bounds
fi
while [ -n "$levels" ]
do

level="${levels%%,*}"		# extract a level
levels="${levels#$level}"	# remove it from list
levels="${levels#,}"		# remove comma after it
curlev=$(( $curlev + 1 ))	# one more level

if [[ $align > 0 ]]
then
	echo "Creating layer $curlev with level $level ..."
fi

# loop trying to meet complexity constraint
xres=$refxres
yres=$refyres
echo -n "Adjusting complexity:"
while true; do

echo -n " $xres"'x'"$yres"

if [ "$verbose" = "true" ]
then
	echo ""
	echo "trace2scad version $version" 1>&2
	if [[ $align > 0 ]]
	then
		echo "--align" 1>&2
	fi
	echo "--complex $complex" 1>&2
	echo "--filter $filter" 1>&2
	echo "--gamma $gamma" 1>&2
	echo "--level $level" 1>&2
	if [[ -n $negate ]]
	then
		echo "--negate" 1>&2
	fi
	echo "--prefix $prefix" 1>&2
	echo "--subres $subres" 1>&2
	echo "--turd $turd" 1>&2
	echo "--xres $xres" 1>&2
	echo "--yres $yres" 1>&2
	echo "input filename is $infile" 1>&2
	echo "prefix is $prefix" 1>&2
	echo "OpenSCAD output filename is $scad" 1>&2
	echo "Converting image to" $pgm 1>&2
fi
convert $negate -gravity center -background white -gamma $gamma -adaptive-resize $xres'x'$yres $infile $ppm
if [ "$filter" != 0 ]
then
	mkbitmap -f $filter -s 2 -t $level <$ppm >$pgm
else
	mkbitmap -n -s 2 -t $level <$ppm >$pgm
fi

if [ "$verbose" = "true" ]
then
	echo "Running potrace to make" $svg 1>&2
fi
# note potrace version dependence:
# potrace 1.11 says -a 0 to disable smoothing
# potrace 1.10 says -a -1 (or any negative) to disable smoothing
# -a -1 seem to work on both versions, so we just do that...
potrace --opaque -a -1 -u $subres --group -sct $turd <$pgm >$svg

# compute max possible bounding box...
# based on the actual PGM file dimensions
xdim=$(( `convert $pgm -print "%w" /dev/null` * $subres ))
ydim=$(( `convert $pgm -print "%h" /dev/null` * $subres ))

if [ "$verbose" = "true" ]
then
	echo "Running sed to make" $raw 1>&2
fi
sed '1,/\/metadata/d
s/<g.*/<g>/
s/<path.*000000.*M/add /
s/<path.*ffffff.*M/sub /
/^fill/d
s/l//
s/z".*/ close/
s/<g>/begin/
s/<\/g>/end/
s/ /\
/g
/svg/d' <$svg >$raw

if [ "$verbose" = "true" ]
then
	echo "Running awk to make" $scad 1>&2
fi
awk -v align=$align -v xdim=$xdim -v ydim=$ydim -v curlev=$curlev -v version=$version -v prefix=$prefix -v level=$level -v complex=$complex -v pixerr=$pixerr '
function r(x) { return(0 + x) }
function indent(x) {
	for (xi=0; xi<x; ++xi) {
		printf " "
	}
}
function distfromline(x0, y0, x1, y1, x2, y2) {
	# compute distance of x1,y1 from the line through x0,y0 and x2,y2
	# as per http://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
	t = (y2-y0)*x1 - (x2-x0)*y1 + x2*y0 - y2*x0
	if (t < 0) t = -t
	b = sqrt((y2-y0)^2 + (x2-x0)^2)
	return(t / b)
}
function needed(x0, y0, x1, y1, x2, y2) {
	# are all 3 points needed
	if ((x0 == x1) && (x0 == x2)) return(0)
	if ((y0 == y1) && (y0 == y2)) return(0)
	if ((y0 != y1) && (y0 != y2)) {
		s01 = (x0 - x1 + 0.0) / (y0 - y1)
		s02 = (x0 - x2 + 0.0) / (y0 - y2)
		if (s01 == s02) return(0)
	}
	if ((x0 != x1) && (x0 != x2)) {
		s01 = (y0 - y1 + 0.0) / (x0 - x1)
		s02 = (y0 - y2 + 0.0) / (x0 - x2)
		if (s01 == s02) return(0)
	}
	if (distfromline(x0, y0, x1, y1, x2, y2) < pixerr) {
		# check for already-absorbed points getting too
		# far from the line
		for (i=0; i<shadow; ++i) {
			if (distfromline(x0, y0, shadx[i], shady[i], x2, y2) >= pixerr) {
				# too far away
				return(1)
			}
		}
		return(0)
	}
	return(1)
}

BEGIN	{
	objno = 0
	lev = 1
	big = 10000000
	minx = big
	miny = big
	maxx = -big
	maxy = -big
	nsp = 1
	byebye = 0
	shadow = 0
	}
/begin/	{
	++lev
	nrec[nsp] = "{"
	++nsp
	next
	}
/end/	{
	--lev
	if ((nrec[nsp-1] == "+") && (nrec[nsp-2] == "{")) {
		# optimize out {x} becomes x
		nrec[nsp-2] = nrec[nsp-1]
		--nsp
	} else {
		nrec[nsp] = "}"
		++nsp
	}
	next
	}
/add/	{
	++objno
	typ[objno] = lev
	nrec[nsp] = "+"
	++nsp
	pts = 0
	curx = 0
	cury = 0
	ncur = "x"
	next
	}
/sub/	{
	++objno
	typ[objno] = -lev
	nrec[nsp] = "-"
	++nsp
	pts = 0
	curx = 0
	cury = 0
	ncur = "x"
	next
	}
/close/	{
	# build string for polygon
	# compute approximate convexity as number of turns
	dx=0
	dy=0
	bufpts[objno] = pts
	buf[objno] = sprintf("polygon([[%d,%d]", curx, cury)
	convexity=2
	for (i=2; i<=pts; ++i) {
		buf[objno] = buf[objno] sprintf(",[%d,%d]", xpt[i], ypt[i])
		ndx=(xpt[i-1]>xpt[i])
		ndy=(ypt[i-1]>ypt[i])
		if ((dx != ndx) || (dy != ndy)) ++convexity
		dx=ndx
		dy=ndy
	}
	buf[objno] = buf[objno] sprintf("], convexity=%d);", convexity)
	ncur = ""
	next
	}
/[-0-9][0-9]*/	{
	if (ncur == "x") {
		prevx = curx
		curx += r($1)
		ncur = "y"
	} else if (ncur == "y") {
		prevy = cury
		cury += r($1)
		ncur = "x"
		if (pts == 0) {
			firstx = curx
			firsty = cury
			++pts
			xpt[pts] = curx
			ypt[pts] = cury
			++allpts
		} else if (((prevx != curx) || (prevy != cury)) &&
			   ((firstx != curx) || (firsty != cury))) {
			if ((pts > 2) &&
			    (needed(curx, cury, xpt[pts], ypt[pts], xpt[pts-1], ypt[pts-1]) < 1)) {
				# get rid of previous point
				shadx[shadow] = xpt[pts]
				shady[shadow] = ypt[pts]
				shadow = shadow + 1
				++byebye
				--pts
				--allpts
			} else {
				# no points shadowed now
				shadow = 0
			}
			++pts
			xpt[pts] = curx
			ypt[pts] = cury
			++allpts
			# update bounding box info
			if (minx > curx) minx = curx
			if (miny > cury) miny = cury
			if (maxx < curx) maxx = curx
			if (maxy < cury) maxy = cury
		} else {
			++byebye
		}

		# abort with no output if complexity exceeded
		if (allpts > (0+complex)) {
			exit
		}
	} else {
		# this should never happen
		print "Error parsing SVG generated by potrace."
	}
	next
	}
/^/	{
	next
	}
END	{
	if (allpts < 2) {
	# dummy output if no points
	print "module " prefix "_" curlev "()"
	print "{"
	print " /* Generated by trace2scad version " version
	print "    http://aggregate.org/MAKE/TRACE2SCAD/"
	print "    There ws no model content with level " level
	print " */"
	print "}"
	print ""
	} else if (allpts < (0+complex)) {
	# no output if complexity exceeded
	print "module " prefix "_" curlev "()"
	print "{"
	print " /* Generated by trace2scad version " version
	print "    http://aggregate.org/MAKE/TRACE2SCAD/"
	print "    Optimized model has " allpts "/" (allpts+byebye) " original points"
	print " */"
	print " color([" level ", " level ", " level "])"
	if (align > 0) {
		print " assign(minx=0) /* polygon minx=" minx " */"
		print " assign(miny=0) /* polygon miny=" miny " */"
		print " assign(maxx=" xdim ") /* polygon maxx=" maxx " */"
		print " assign(maxy=" ydim ") /* polygon maxy=" maxy " */"
	} else {
		print " assign(minx=" minx ")"
		print " assign(miny=" miny ")"
		print " assign(maxx=" maxx ")"
		print " assign(maxy=" maxy ")"
	}
	print " assign(dx=maxx-minx)"
	print " assign(dy=maxy-miny)"
	print " assign(maxd=((dx>dy)?dx:dy))"
	print " scale([1/maxd, 1/maxd, 1])"
	print " translate([-minx-dx/2, -miny-dy/2, 0])"
	print sprintf(" linear_extrude(height=1, convexity=%d)", allpts)
	print " union() {"
	i = 1
	j = 1
	k = 2
	while (j < nsp) {
		if (nrec[j] == "{") {
			indent(k)
			if (nrec[j+2] == "-") {
				print "difference() {"
			} else {
				print "union() {"
			}
			++k
		} else if (nrec[j] == "}") {
			--k
			indent(k)
			print "}"
		} else {
			indent(k)
			print buf[i]
			++i
		}
		++j
	}
	print " }"
	print "}"
	print ""
	}
	}
' <$raw >$stmp

if [ -s "$stmp" ]
then
	echo ""
	break
fi

# reduce resolution to simplify
xres=`expr $xres '*' 9 '/' 10`
yres=`expr $yres '*' 9 '/' 10`
if [ "$verbose" = "true" ]
then
	echo -n "Test complexity:"
fi

done

# add this layer to output file
cat $stmp >>$scad

done

# stick end on it all
awk -v curlev=$curlev -v version=$version -v prefix=$prefix -v complex=$complex '
BEGIN	{
	print "module " prefix "()"
	print "{"
	print " /* all layers combined, scaled to within a 1mm cube */"
	print " scale([1, 1, 1/" curlev "])"
	print " difference() {"
	print "  union() {"
	for (i=0; i<curlev; ++i) {
		print "   scale([1,1," 2*(i+1) "]) translate([0,0,-0.5]) " prefix "_" (i+1) "();"
	}
	print "  }"
	print "  translate([0,0,-" (curlev+1) "]) cube([2,2," (2*(curlev+1)) "],center=true);"
	print " }"
	print "}"
	}
' </dev/null >>$scad

rm -f "$stmp"
if [ "$verbose" = "true" ]
then
	echo "Leaving tmp files:" $ppm $pgm $svg $raw 1>&2
else
	rm -f $ppm $pgm $svg $raw
fi