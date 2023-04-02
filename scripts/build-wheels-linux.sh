#!/bin/bash

set -euo pipefail
export LC_ALL=C

#Early check for build tools
cmake --version

cd $(dirname $(readlink -f "${BASH_SOURCE[0]}"))/..

mkdir -p ./build ./dist ./map_metrics
cp ./__init__.py ./map_metrics/__init__.py

cd ./build

NUMPROC=$(nproc)
echo "Running $NUMPROC parallel jobs"

LATEST=""

for PYBIN in /opt/python/cp3*/bin/python
do
    LATEST=${PYBIN}
    cmake -S .. -B . \
             -DPYTHON_EXECUTABLE:FILEPATH=${PYBIN} \
             -DCMAKE_BUILD_WITH_INSTALL_RPATH=TRUE \
             -DCMAKE_INSTALL_RPATH='$ORIGIN' \
             -DBOOST_ROOT=${BOOST_ROOT} \
             -DCMAKE_RUNTIME_OUTPUT_DIRECTORY=$PWD/../bin \
             -DCMAKE_LIBRARY_OUTPUT_DIRECTORY=$PWD/../map_metrics \
    && cmake --build .
done

cd ../
${LATEST} -m pip install $([[ -n "$VIRTUAL_ENV" ]] || echo "--user") -q build auditwheel
${LATEST} -m build --wheel --outdir ./dist/ .
auditwheel repair ./dist/*.whl
