SUBDIRS = ea1
bin_PROGRAMS = probo
probo_SOURCES = pfamily.cpp probo.cpp probolua.cpp ppca9685.cpp main.cpp
probo_CXXFLAGS = -Wall -std=c++11 -fPIC -I/usr/include/lua5.2 -I./ea1
probo_LDADD = -llua5.2 -L./ea1 -lea1
