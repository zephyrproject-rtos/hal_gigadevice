#!/usr/bin/env bash

function after_compress() {
    echo "no actions in after_compress hook"
}

function download() {
    wget -O $1 $2
    return $?
}

function checkfilehash() {
    echo "$1 $2"
    echo "$1 $2" | sha1sum -c -
    return $?
}

function getfile() {
    checkfilehash $3 $1
    if [ $? -ne 0 ]; then
        # download if not exist or hash not match
        download $1 $2
    fi
}

function uncompress_rar() {
    unrar x $1 $2
}

function uncompress() {
    rm -rvf $2
    mkdir -p $2

    ftpye=$(file "$1")
    case "$ftpye" in
    "$1: RAR archive"*)
        uncompress_rar $1 $2
        ;;
    *)
        echo unsupport file format: $ftpye
        ;;
    esac

    after_compress $1 $2 # call hook
}

function rename_to_lower() {
    mv -v $(dirname $1 | tr '[:upper:]' '[:lower:]')/$(basename $1) $(echo $1 | tr '[:upper:]' '[:lower:]')
}
export -f rename_to_lower

function drop_unnessary_files() {
    libbak=$1.bak
    rm -rf $libbak
    mv $1 $libbak
    mkdir -p $1

    cp -rv $libbak/Firmware/CMSIS $1/cmsis
    find $1/cmsis/GD/ -maxdepth 2 -type d | xargs -I {} bash -c 'rename_to_lower "{}"' \;
    rm $1/cmsis/*.h
    rm -r $1/cmsis/gd/*/source/ARM
    rm -r $1/cmsis/gd/*/source/IAR

    cp -rv $libbak/Firmware/*_standard_peripheral $1/standard_peripheral
    find $1/standard_peripheral -maxdepth 1 -type d | xargs -I {} bash -c 'rename_to_lower "{}"' \;
    cp -v $libbak/Template/*_libopt.h $1/standard_peripheral/include/
    rm -rv $libbak
}

function change_file_format() {
    find $1 type f -print0 | xargs -0 dos2unix -k
}

function patch_cocci() {
    script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
    # pushd $script_dir
    for cocci in $script_dir/../patches/*.cocci; do
        find $1 -name "*.h" -or -name "*.c" | xargs -P0 -n1 $script_dir/eval_patch.sh $cocci
    done
    # popd
}
