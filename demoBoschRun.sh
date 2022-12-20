#!/bin/sh

MY_DIR="$( cd "$(dirname "$0")" ; pwd -P )"

PYTHONPATH="$MY_DIR/..":"$MY_DIR/../interpolation" python3 "$MY_DIR/demoBosch.py"
