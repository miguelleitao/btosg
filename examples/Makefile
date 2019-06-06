# btosg examples Makefile 
#
# Miguel Leitao, 2019

# Targets

EXAMPLES=ball carZ carY objects


BULLET_DIR?=/usr
OSG_DIR?=/usr
BTOSG_DIR?=..
INC_BULLET?=$(shell pkg-config --cflags-only-I bullet)
INC_OSG?=$(shell pkg-config --cflags-only-I openscenegraph-osg)
INC_BTOSG?=-I ${BTOSG_DIR} ${INC_BULLET} ${INC_OSG}
LIB_BULLET_DIR?=$(shell pkg-config --libs-only-L bullet)
LIB_OSG_DIR=$(shell pkg-config --libs-only-L openscenegraph-osg)
LIB_BTOSG_DIR=-L ${BTOSG_DIR} ${LIB_BULLET_DIR} ${LIB_OSG_DIR}

CXXFLAGS?=-std=c++11 -Wall -Wextra -O2 -Wno-uninitialized -Wno-unused-parameter
VERSION:=$(shell git describe --tags --long)

LD_B3OBJ_IMPORT=-L ${BTOSG_DIR}/loadOBJ -lloadOBJ -l Bullet3Common
B3_OBJ_LOADER=${BTOSG_DIR}/loadOBJ/libloadOBJ.a -l Bullet3Common

# Machine specific definitions can be used from local .config file
-include .config

ifeq ($(PREFIX),)
    PREFIX := /usr/local
endif


LD_BULLET=${LIB_BULLET_DIR} -l BulletDynamics -l BulletCollision -l LinearMath
LD_OSG=${LIB_OSG_DIR} -l osg -losgViewer -losgSim -losgDB -losgGA -losgShadow
LD_BTOSG=${LIB_BTOSG_DIR} -l btosg ${LD_BULLET} ${LD_OSG}

default: examples

examples: ${EXAMPLES}

# test targets
# test target are built during CI build process.
# CI build process uses BTOSG_LOAD_OBJ=NO.
# Applications using btosgExternalObject() should not be included in this phase.
test: ball carZ 

all: examples 

carZ: carZ.o 
	$(CXX) -O2 -o $@ $^  ${LD_BTOSG} -lm

carY: carY.o 
	$(CXX) -O2 -o $@ $^  ${LD_BTOSG} -lm

ball: ball.o 
	$(CXX) -O2 -o $@ $^  ${LD_BTOSG} -lm

objects: objects.o  
	$(CXX) -O2 -o $@ $^ ${LD_B3OBJ_IMPORT} ${LD_BTOSG}  -lm

hpr_test: hpr_test.o  ${B3_OBJ_LOADER}
	$(CXX) -O2 -o $@ $^ ${LD_B3OBJ_IMPORT} ${LD_BTOSG} -lm 

carZ.o: car.cpp
	$(CXX) ${CXXFLAGS} -c ${INC_BTOSG} $< -o $@ -DVERSION=${VERSION} 

carY.o: car.cpp 
	$(CXX) ${CXXFLAGS} -c ${INC_BTOSG} $< -o $@ -DVERSION=${VERSION} -D_UP_=0,1,0

ball.o: ball.cpp 
	$(CXX) ${CXXFLAGS} -c ${INC_BTOSG} -DVERSION=${VERSION} $<

objects.o: objects.cpp 
	$(CXX) ${CXXFLAGS} -c ${INC_BTOSG} -DVERSION=${VERSION} $< 

hpr_test.o: hpr_test.cpp 
	$(CXX) ${CXXFLAGS} -g -c ${INC_BTOSG} -DVERSION=${VERSION} $< 

clean:
	$(RM) *.o ${EXAMPLES} 

