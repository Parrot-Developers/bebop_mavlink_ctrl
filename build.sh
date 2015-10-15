#!/bin/bash - 

git submodule init && git submodule update
cd mavlink/ && python -m pymavlink.tools.mavgen --lang C -o ../out/ message_definitions/v1.0/common.xml ; cd ../
make all tests
