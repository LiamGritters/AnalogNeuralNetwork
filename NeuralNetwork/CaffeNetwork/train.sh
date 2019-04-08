#!/usr/bin/env sh

rm -r snapshot_iter*

/home/liam/Caffe/caffe/build/tools/caffe train --solver=solver.prototxt
