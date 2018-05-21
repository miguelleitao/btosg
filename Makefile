
BTOSG=btosg.o
EXAMPLES=ball carZ carY objects

BULLET_DIR?=/usr
OSG_DIR?=/usr
INC_BULLET?=`pkg-config --cflags-only-I bullet`
INC_OSG?=`pkg-config --cflags-only-I openscenegraph-osg`
LIB_BULLET_DIR?=`pkg-config --libs-only-L bullet`
LIB_OSG_DIR=`pkg-config --libs-only-L openscenegraph-osg`
CFLAGS=-std=c++11 -Wall -O2 -Wno-uninitialized
VERSION:=$(shell git tag)

#LD_B3OBJ_IMPORT=-L loadOBJ -lloadOBJ
B3_OBJ_LOADER=loadOBJ/libloadOBJ.a

-include .config

LD_BULLET=${LIB_BULLET_DIR} -l BulletDynamics -l BulletCollision -l Bullet3Common -l LinearMath
LD_OSG=${LIB_OSG_DIR} -l osg -losgViewer -losgSim -losgDB -losgGA -losgShadow

default: ${BTOSG}

examples: ${EXAMPLES}

all: default examples

carZ: carZ.o btosg.o
	cc -O2 -o $@ $^ ${LD_BULLET} ${LD_OSG} -l stdc++ -lm

carY: carY.o btosg.o
	cc -O2 -o $@ $^ ${LD_BULLET} ${LD_OSG} -l stdc++ -lm

ball: ball.o btosg.o
	cc -O2 -o $@ $^ ${LD_BULLET} ${LD_OSG} -l stdc++ -lm

objects: objects.o btosg.o ${B3_OBJ_LOADER}
	cc -O2 -o $@ $^ ${LD_BULLET} ${LD_OSG} -l stdc++ -lm

carZ.o: car.cpp btosg.h btosgVehicle.h
	g++ ${CFLAGS} -c ${INC_BULLET} ${INC_OSG} $< -o $@ -DVERSION=${VERSION} 

carY.o: car.cpp btosg.h btosgVehicle.h
	g++ ${CFLAGS} -c ${INC_BULLET} ${INC_OSG} $< -o $@ -DVERSION=${VERSION} -D_UP_=0,1,0

ball.o: ball.cpp btosg.h 
	g++ ${CFLAGS} -c ${INC_BULLET} ${INC_OSG} -DVERSION=${VERSION} $<

objects.o: objects.cpp btosg.h
	g++ ${CFLAGS} -c ${INC_BULLET} ${INC_OSG} -DVERSION=${VERSION} $<

#-DBTOSG_SHADOW $<

btosg.o: btosg.cpp btosg.h
	g++ ${CFLAGS} -c ${INC_BULLET} ${INC_OSG} -DVERSION=${VERSION} $<

${B3_OBJ_LOADER}:
	make -C loadOBJ BULLET_DIR=${BULLET_DIR} OSG_DIR=${OSG_DIR}

clean:
	rm -f *.o ${EXAMPLES}
	make INC_BULLET=${INC_BULLET} -C loadOBJ clean

push: *.cpp *.h Makefile README.md loadOBJ obj .gitignore .travis.yml
	git add $^
	git commit -m "update"
	git push
