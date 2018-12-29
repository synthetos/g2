#! /bin/bash

MAKECMD='make CONFIG=PrintrbotSimple1608'

if [ "$1" == "clean" ]; then
    ${MAKECMD} clean
    exit 0
fi

${MAKECMD} clean
${MAKECMD}

