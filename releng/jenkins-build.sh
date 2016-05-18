#!/bin/bash
set -e

cd ${WORKSPACE}/TinyG2
make PLATFORM="${PLATFORM}"

FIRMWARE_BUILD=`cat tinyg2.h | grep '#define TINYG_FIRMWARE_BUILD' | sed -E 's:[^0-9]*([0-9]+.[0-9]+).*:\1:'`
if [ "${PLATFORM}" == "OthermillPro" ]; then
    FN_VERSION_SUFFIX="-pro"
fi
if [ "${RELEASE_TYPE}" != "release" ]; then
    GIT_SUFFIX="-${GIT_COMMIT:0:8}"
fi

cd bin/"${PLATFORM}"
for f in *; do mv "$f" "${f/${PLATFORM}/tinyg${FN_VERSION_SUFFIX}-${FIRMWARE_BUILD}${GIT_SUFFIX}}"; done