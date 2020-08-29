#!/bin/bash

docker run --rm -it --gpus all -p 4567:4567 -p 8888:8888 -v `pwd`:/workspace mkolod/udacity_carnd_capstone
