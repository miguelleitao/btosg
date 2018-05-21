
BTOSG=btosg.o
EXAMPLES=ball carZ carY

INC_BULLET=-I/usr/include/bullet 
#INC_BULLET=-I ../bullet3/src
INC_OSG=-I/usr/include/osg
LD_OSG=-losg -losgViewer -losgSim -losgDB -losgGA -losgShadow
#LD_BULLET=-L /home/jml/bullet3/src/BulletDynamics
LD_BULLET=-l BulletDynamics -l BulletCollision -l Bullet3Common -l LinearMath
CFLAGS=-std=c++11 -Wall -O2 -Wno-uninitialized
VERSION:=$(shell git tag)

#OBJ_B3OBJ_IMPORT=LoadMeshFromObj.o ImportObjExample.o Wavefront2GLInstanceGraphicsShape.o tiny_obj_loader.o
#LD_B3OBJ_IMPORT=-L /home/jml/bullet3/src/Bullet3Common -l Bullet3Common b3ResourcePath.o stb_image.o
OBJ_B3OBJ_IMPORT=-L loadOBJ -lloadOBJ

default: ${BTOSG}

examples: ${EXAMPLES}

all: default examples

carZ: carZ.o btosg.o
	cc -O2 -o $@ $^ ${LD_BULLET} ${LD_OSG} -l stdc++ -lm

carY: carY.o btosg.o
	cc -O2 -o $@ $^ ${LD_BULLET} ${LD_OSG} -l stdc++ -lm

ball: ball.o btosg.o 
	cc -O2 -o $@ $^ ${OBJ_B3OBJ_IMPORT} ${LD_BULLET} ${LD_B3OBJ_IMPORT} ${LD_OSG} -l stdc++ -lm


#cc -O2 -o ball ball.o btosg.o LoadMeshFromObj.o ImportObjExample.o Wavefront2GLInstanceGraphicsShape.o tiny_obj_loader.o -L /home/jml/bullet3/src/BulletDynamics -L /home/jml/bullet3/src/Bullet3Common -l BulletDynamics -l BulletCollision -l LinearMath -l Bullet3Common -losg -losgViewer -losgSim -losgDB -losgGA -losgShadow -l stdc++ -lm b3ResourcePath.o stb_image.o

	
carZ.o: car.cpp btosg.h btosgVehicle.h
	g++ ${CFLAGS} -c ${INC_BULLET} ${INC_OSG} $< -o -DVERSION=${VERSION} $@

carY.o: car.cpp btosg.h btosgVehicle.h
	g++ ${CFLAGS} -c ${INC_BULLET} ${INC_OSG} $< -o $@ -DVERSION=${VERSION} -D_UP_=0,1,0

ball.o: ball.cpp btosg.h 
	g++ ${CFLAGS} -c ${INC_BULLET} ${INC_OSG} -DVERSION=${VERSION} $<

#-DBTOSG_SHADOW $<

btosg.o: btosg.cpp btosg.h
	g++ ${CFLAGS} -c ${INC_BULLET} ${INC_OSG} -DVERSION=${VERSION} $<

clean:
	rm -f *.o ${EXAMPLES}

push: *.cpp *.h Makefile README.md loadOBJ .gitignore .travis.yml
	git add $^
	git commit -m "update"
	git push
