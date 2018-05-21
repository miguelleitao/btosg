
BTOSG=btosg.o
EXAMPLES=ball carZ carY

BULLET_DIR?=/usr
OSG_DIR?=/usr
INC_BULLET?=-I${BULLET_DIR}/include/bullet 
INC_OSG?=-I${OSG_DIR}/include/osg
LD_OSG=-losg -losgViewer -losgSim -losgDB -losgGA -losgShadow
LD_BULLET=-l BulletDynamics -l BulletCollision -l Bullet3Common -l LinearMath
CFLAGS=-std=c++11 -Wall -O2 -Wno-uninitialized
VERSION:=$(shell git tag)

#LD_B3OBJ_IMPORT=-L loadOBJ -lloadOBJ
B3_OBJ_LOADER=loadOBJ/libloadOBJ.a

default: ${BTOSG}

examples: ${EXAMPLES}

all: default examples

carZ: carZ.o btosg.o
	cc -O2 -o $@ $^ ${LD_BULLET} ${LD_OSG} -l stdc++ -lm

carY: carY.o btosg.o
	cc -O2 -o $@ $^ ${LD_BULLET} ${LD_OSG} -l stdc++ -lm

ball: ball.o btosg.o ${B3_OBJ_LOADER}
	cc -O2 -o $@ $^ ${LD_BULLET} ${LD_OSG} -l stdc++ -lm

carZ.o: car.cpp btosg.h btosgVehicle.h
	g++ ${CFLAGS} -c ${INC_BULLET} ${INC_OSG} $< -o $@ -DVERSION=${VERSION} 

carY.o: car.cpp btosg.h btosgVehicle.h
	g++ ${CFLAGS} -c ${INC_BULLET} ${INC_OSG} $< -o $@ -DVERSION=${VERSION} -D_UP_=0,1,0

ball.o: ball.cpp btosg.h 
	g++ ${CFLAGS} -c ${INC_BULLET} ${INC_OSG} -DVERSION=${VERSION} $<

#-DBTOSG_SHADOW $<

btosg.o: btosg.cpp btosg.h
	g++ ${CFLAGS} -c ${INC_BULLET} ${INC_OSG} -DVERSION=${VERSION} $<

${B3_OBJ_LOADER}:
	make -C loadOBJ

clean:
	rm -f *.o ${EXAMPLES}
	make INC_BULLET=${INC_BULLET} -C loadOBJ clean

push: *.cpp *.h Makefile README.md loadOBJ .gitignore .travis.yml
	git add $^
	git commit -m "update"
	git push
