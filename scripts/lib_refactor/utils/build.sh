#!/usr/bin/env bash

# prepare env var
source utils/utils.sh
source $1

TMPDIR=tmp
DESTDIR=dest/${NAME}

echo Processing ${NAME}

# make workspace folder
mkdir -p ${TMPDIR}
mkdir -p ${DESTDIR}

# download files
getfile ${TMPDIR}/${FILENAME} ${URL} ${HASH}
checkfilehash ${HASH} ${TMPDIR}/${FILENAME}
if [ $? -ne 0 ]; then
    # abort if hash not match
    echo "file ${NAME}:${FILENAME} download failed!"
    exit 1
fi

uncompress ${TMPDIR}/${FILENAME} ${DESTDIR}

drop_unnessary_files ${DESTDIR}
change_file_format ${DESTDIR}
patch_cocci ${DESTDIR}
