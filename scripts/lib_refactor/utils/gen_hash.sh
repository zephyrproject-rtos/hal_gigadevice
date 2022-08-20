#!/usr/bin/env bash

source utils/utils.sh
source $1

TMPDIR=tmp

# make workspace folder
mkdir -p ${TMPDIR}

# force re-download
download ${TMPDIR}/${FILENAME} ${URL}
if [ $? -ne 0 ]; then
    # abort if hash not match
    echo "file ${NAME}:${FILENAME} hash update failed!"
    exit 1
fi

HASH=$(sha1sum ${TMPDIR}/${FILENAME} | cut -d ' ' -f 1)
echo "file ${NAME}:${FILENAME} hash update to ${HASH}"
sed -i -r "s/^HASH=.*$/HASH=${HASH}/" $1
