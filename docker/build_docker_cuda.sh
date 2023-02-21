#!/usr/bin/env bash

ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
EXEC_PATH=$PWD

cd $ROOT_DIR

if [[ $1 = "--nvidia" ]] || [[ $1 = "-n" ]]
  then
    docker build -t ros-mir-img -f $ROOT_DIR/docker2/Dockerfile $ROOT_DIR \
                                  --network=host \
                                  --build-arg from=nvidia/cuda:11.4.2-cudnn8-runtime-ubuntu20.04

else
    echo "[!] If you use nvidia gpu, please rebuild with -n or --nvidia argument"
    docker build -t ros-mir-img -f $ROOT_DIR/docker2/Dockerfile $ROOT_DIR \
                                  --network=host \
                                  --build-arg from=ubuntu:18.04
fi

cd $EXEC_PATH
