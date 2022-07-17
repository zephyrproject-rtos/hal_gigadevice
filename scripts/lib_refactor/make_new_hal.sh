#!/usr/bin/env bash

if [ $# -ne 1 ]
  then
    echo "number of arguments must be 1"
    exit
fi

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
echo $script_dir
# set top (realpath $script_dir/../..)
top=$(realpath $script_dir/../..)
echo $top
target=$(realpath $top/$1)
echo work at $top , target: $target

# check lib folder file is correct
if [ ! -d $target/Firmware/CMSIS ]; then
    echo "file in folder is not correct!"
    exit
fi

# mkdirs
libbak=$target.bak
mv $target $libbak
mkdir -p $target

# copy files and drop unnessary files
cp -rv $libbak/Firmware/CMSIS $target/cmsis
rm $target/cmsis/*.h
rm -r $target/cmsis/GD/*/Source/ARM
rm -r $target/cmsis/GD/*/Source/IAR
cp -rv $libbak/Firmware/*_standard_peripheral $target/standard_peripheral
cp $libbak/Template/*_libopt.h $target/standard_peripheral/Include/

# change file to unix format
find $target type f -print0 | xargs -0 dos2unix -k

# patch the files
pushd $script_dir
for cocci in *.cocci; do
    find $target -name "*.h" -or -name "*.c" | xargs -P0 -n1 $script_dir/eval_patch.sh $cocci
done
popd
