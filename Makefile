# btosg Makefile 
#
# Miguel Leitao, 2016

# Targets
BTOSG=libbtosg.a libbtosg.so
BTOSG_PC=btosg.pc

BULLET_DIR?=/usr/local
OSG_DIR?=/usr
INC_BULLET?=$(shell pkg-config --silence-errors --cflags-only-I bullet) -I /usr/local/include/bullet
INC_OSG?=$(shell pkg-config --silence-errors --cflags-only-I openscenegraph-osg)
LIB_BULLET_DIR?=$(shell pkg-config --silence-errors --libs-only-L bullet)
LIB_OSG_DIR=$(shell pkg-config --silence-errors --libs-only-L openscenegraph-osg)
BTOSG_LOAD_OBJ?=YES
CXXFLAGS?=-std=c++11 -Wall -Wextra -O2 -Wno-uninitialized -Wno-unused-parameter -DBTOSG_LOAD_OBJ=${BTOSG_LOAD_OBJ}
VERSION:=$(shell git describe --tags --long)

#LD_B3OBJ_IMPORT=-L loadOBJ -lloadOBJ
B3_OBJ_LOADER=loadOBJ/libloadOBJ.a

# Machine specific definitions can be used from local .config file
-include .config

ifeq ($(PREFIX),)
    PREFIX := /usr/local
endif

#LD_BULLET=${LIB_BULLET_DIR} -l BulletDynamics -l BulletCollision -l Bullet3Common -l LinearMath
LD_BULLET=${LIB_BULLET_DIR} -l BulletDynamics -l BulletCollision -l LinearMath
LD_OSG=${LIB_OSG_DIR} -l osg -losgViewer -losgSim -losgDB -losgGA -losgShadow

default: ${BTOSG} loadOBJ

Examples:
	make -C examples BULLET_DIR=${BULLET_DIR} OSG_DIR=${OSG_DIR} all

# test targets
# test target are built during CI build process.
# CI build process uses BTOSG_LOAD_OBJ=NO.
# Applications using btosgExternalObject() should not be included in this phase.
test: ${BTOSG} pc 
	@make -C examples carZ ball

pc:
	rm -f ${BTOSG_PC}
	@make ${BTOSG_PC}

btosg: ${BTOSG}
	@echo -n ""

all: default Examples docs pc

All:
	make clean
	make all

manual:
	${RM} -rf documentation/html documentation/latex
	@make documentation/html

documentation/html:
	doxygen btosg.doxygen

#-DBTOSG_SHADOW $<

btosg.o: btosg.cpp btosg.h
	$(CXX) ${CXXFLAGS} -g -c ${INC_BULLET} ${INC_OSG} -DVERSION=${VERSION} -fPIC $<

libbtosg.a: btosg.o
	$(AR) cr $@ $^

libbtosg.so: btosg.o
	$(CXX) -shared $^ -o $@

loadOBJ: ${B3_OBJ_LOADER}

${B3_OBJ_LOADER}:
	make -C loadOBJ BULLET_DIR=${BULLET_DIR} OSG_DIR=${OSG_DIR} all

.PRECIOUS: ${BTOSG_PC}
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

install: ${BTOSG} ${BTOSG_PC}
	install -d $(DESTDIR)$(PREFIX)/lib/
	install -m 755 ${BTOSG} $(DESTDIR)$(PREFIX)/lib/
	install -m 755 ${B3_OBJ_LOADER} $(DESTDIR)$(PREFIX)/lib/
	install -d $(DESTDIR)$(PREFIX)/include/
	install -m 644 btosg.h $(DESTDIR)$(PREFIX)/include/
	install -m 644 btosgVehicle.h $(DESTDIR)$(PREFIX)/include/
	install -d $(DESTDIR)$(PREFIX)/lib/pkgconfig/
	install -m 644 ${BTOSG_PC} $(DESTDIR)$(PREFIX)/lib/pkgconfig/

clean:
	$(RM) *.o ${EXAMPLES} ${BTOSG} *.pc
	make -C loadOBJ clean
	make -C examples clean

push: *.cpp *.h Makefile *.md loadOBJ img obj .gitignore .travis.yml docs btosg.doxygen .gitignore examples
	git add $^
	git commit -m "update"
	git push origin HEAD
