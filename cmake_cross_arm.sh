#!/bin/bash
# PC上で ARM クロスコンパイラを使う時のスクリプト
# 2016-12-15 17:08:06

cmake -DCMAKE_C_COMPILER="/usr/bin/arm-linux-gnueabihf-gcc" \
      -DCMAKE_CXX_COMPILER="/usr/bin/arm-linux-gnueabihf-g++" \
      -DCMAKE_CROSS_ARM=1 \
       $*

