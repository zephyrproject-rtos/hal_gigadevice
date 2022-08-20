#!/usr/bin/env bash

if [ $# -ne 2 ]
  then
    echo "number of arguments must be 2"
    exit
fi

source_file=$2
cocci=$1

echo cocci: $cocci
echo source: $source_file

spatch -cocci_file $cocci $source_file --in-place
