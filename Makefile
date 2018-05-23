# btosg Makefile 

BTOSG=libbtosg.a
EXAMPLES=ball carZ carY objects

BULLET_DIR?=/usr
OSG_DIR?=/usr
INC_BULLET?=$(shell pkg-config --cflags-only-I bullet)
INC_OSG?=$(shell pkg-config --cflags-only-I openscenegraph-osg)
LIB_BULLET_DIR?=$(shell pkg-config --libs-only-L bullet)
LIB_OSG_DIR=$(shell pkg-config --libs-only-L openscenegraph-osg)
CXXFLAGS?=-std=c++11 -Wall -O2 -Wno-uninitialized
VERSION:=$(shell git tag)

#LD_B3OBJ_IMPORT=-L loadOBJ -lloadOBJ
B3_OBJ_LOADER=loadOBJ/libloadOBJ.a
BTOSG_PC=btosg.pc

-include .config

ifeq ($(PREFIX),)
    PREFIX := /usr/local
endif

LD_BULLET=${LIB_BULLET_DIR} -l BulletDynamics -l BulletCollision -l Bullet3Common -l LinearMath
LD_OSG=${LIB_OSG_DIR} -l osg -losgViewer -losgSim -losgDB -losgGA -losgShadow

default: ${BTOSG}

examples: ${EXAMPLES}

pc:
	rm -f ${BTOSG_PC}
	@make ${BTOSG_PC}

btosg: ${BTOSG}
	@echo -n ""

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
	$(CXX) ${CXXFLAGS} -c ${INC_BULLET} ${INC_OSG} $< -o $@ -DVERSION=${VERSION} 

carY.o: car.cpp btosg.h btosgVehicle.h
	$(CXX) ${CXXFLAGS} -c ${INC_BULLET} ${INC_OSG} $< -o $@ -DVERSION=${VERSION} -D_UP_=0,1,0

ball.o: ball.cpp btosg.h 
	$(CXX) ${CXXFLAGS} -c ${INC_BULLET} ${INC_OSG} -DVERSION=${VERSION} $<

objects.o: objects.cpp btosg.h
	$(CXX) ${CXXFLAGS} -c ${INC_BULLET} ${INC_OSG} -DVERSION=${VERSION} $<

#-DBTOSG_SHADOW $<

btosg.o: btosg.cpp btosg.h
	$(CXX) ${CXXFLAGS} -c ${INC_BULLET} ${INC_OSG} -DVERSION=${VERSION} $<

${BTOSG}: btosg.o
	$(AR) cr $@ $^

${B3_OBJ_LOADER}:
	make -C loadOBJ BULLET_DIR=${BULLET_DIR} OSG_DIR=${OSG_DIR}

${BTOSG_PC}:
	@echo "prefix=/usr" 				 > $@
	@echo "exec_prefix=\$${prefix}"			>> $@
	@echo "includedir=\$${prefix}/include"		>> $@
	@echo "libdir=\$${exec_prefix}/lib"		>> $@
	@echo "os=$(shell uname)"			>> $@
	@echo "machtype=$(shell gcc -dumpmachine)"	>> $@
	@echo ""					>> $@
	@echo "Name: btosg"				>> $@
	@echo "Description: A visual simulation library integrating Bullet and OpenSceneGraph"	>> $@
	@echo "Version: ${VERSION}"			>> $@
	@echo "URL: https://github.com/miguelleitao/btosg"	>> $@
	@echo "Requires: bullet openscenegraph-osg openscenegraph-osgViewer openscenegraph-osgSim openscenegraph-osgDB openscenegraph-osgGA openscenegraph-osgShadow"	>> $@
	@echo "Cflags: -I\$${includedir}"		>> $@
	@echo "Libs: -L\$${libdir} -losg"		>> $@

install: ${BTOSG} ${BTOSGPC}
	install -d $(DESTDIR)$(PREFIX)/lib/
	install -m 755 ${BTOSG} $(DESTDIR)$(PREFIX)/lib/
	install -m 755 ${B3_OBJ_LOADER} $(DESTDIR)$(PREFIX)/lib/
	install -d $(DESTDIR)$(PREFIX)/include/
	install -m 644 btosg.h $(DESTDIR)$(PREFIX)/include/
	install -m 644 btosgVehicle.h $(DESTDIR)$(PREFIX)/include/
	install -d $(DESTDIR)$(PREFIX)/lib/pkgconfig/
	install -m 644 ${BTOSG_PC} $(DESTDIR)$(PREFIX)/lib/pkgconfig/

clean:
	$(RM) *.o ${EXAMPLES}
	make INC_BULLET=${INC_BULLET} -C loadOBJ clean

push: *.cpp *.h Makefile README.md loadOBJ obj .gitignore .travis.yml
	git add $^
	git commit -m "update"
	git push
