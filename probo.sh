#!/bin/bash
# probo お試し実行 2016-06-12 18:18:49
g++ -std=c++11 -fPIC -I/usr/include/lua5.2 -o probo pfamily.cpp probo.cpp -llua5.2
./probo
