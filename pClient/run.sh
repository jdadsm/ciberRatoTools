#!/bin/bash

challenge="1"
host="localhost"
robname="theAgent"
pos="0"
outfile="solution"

while getopts "c:h:r:p:f:" op
do
    case $op in
        "c")
            challenge=$OPTARG
            ;;
        "h")
            host=$OPTARG
            ;;
        "r")
            robname=$OPTARG
            ;;
        "p")
            pos=$OPTARG
            ;;
        "f")
            outfile=$OPTARG
            ;;
        default)
            echo "ERROR in parameters"
            ;;
    esac
done

shift $(($OPTIND-1))

case $challenge in
    1)
        # how to call agent for challenge 1
        python3 mainC1.py -h "$host" -p "$pos" -r "$robname"
        ;;
    2)
        # how to call agent for challenge 2
        java mainC2 -h "$host" -p "$pos" -r "$robname" -f "$outfile"
        mv map_c2.map $outfile.map             # if needed
        ;;
    3)
        # how to call agent for challenge 3
        ./mainC3 -h "$host" -p "$pos" -r "$robname" -f "$outfile"
        mv pathC3.path $outfile.path           # if needed
        ;;

    4)
        rm -f *.path *.map  # do not remove this line

        # how to call agent for challenge 4
        python3 mainC4.py -h "$host" -p "$pos" -r "$robname" -f "$outfile"
        mv map_c2.map $outfile.map
        mv pathC3.path $outfile.path
        ;;

        # how to call agent for challenge 4
        #java mainC4 -h "$host" -p "$pos" -r "$robname" 
        #mv your_mapfile $outfile.map
        #mv your_pathfile $outfile.path
        #;;

        # how to call agent for challenge 4
        #./mainC4 -h "$host" -p "$pos" -r "$robname"
        #mv your_mapfile $outfile.map
        #mv your_pathfile $outfile.path
        #;;
esac

