rm -rf shape_lmdb_test shape_lmdb_train

export LD_LIBRARY_PATH=/home/liam/caffe/build/lib:${LD_LIBRARY_PATH}
./bin/DataGenerator --backend=lmdb --split=1 --shuffle=true --balance=true shape_lmdb ../DataCollector/AnalogRobotData.csv 
