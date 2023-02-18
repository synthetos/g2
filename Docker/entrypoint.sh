#!/bin/bash

set -e


if [ "$1" == "--build" ]; then
    BOARD="$2"
    SETTINGS="$3"
    if [ -z "$2" ] || [ -z "$3" ]; then
        echo "Usage: $0 --build BOARD SETTING"
        exit 1
    fi

    if [ ! -f "/app/g2/g2core/settings/$SETTINGS" ]; then
        echo "Error: Settings file not found: /g2/g2core/settings/$SETTINGS"
        exit 1
    fi

    
   
 
    echo "Building g2core for board $BOARD with setting $SETTINGS"
    echo "Build started in 5 seconds..."
    sleep 5
    cd /app/g2/g2core
    make BOARD="$BOARD" SETTINGS_FILE="$SETTINGS"
fi

