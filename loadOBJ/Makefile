

TARGET=libloadOBJ.a
OBJS=LoadMeshFromObj.o Wavefront2GLInstanceGraphicsShape.o tiny_obj_loader.o Wavefront2GLInstanceGraphicsShape.o

CXXFLAGS=-Wall -I include -I include/extra -I include/extra/extra ${INC_BULLET} -Wno-unused-variable


BULLET_DIR?=/usr
INC_BULLET?:=$(shell pkg-config --silence-errors --cflags-only-I bullet)
INC_BULLET?=-I /usr/local/include/bullet
LIB_BULLET_DIR?=$(shell pkg-config --silence-errors --libs-only-L bullet)
VERSION:=$(shell git tag)

all: ${TARGET}

${TARGET}: ${OBJS}
	$(AR) cr $@ $^

clean:
	$(RM) ${TARGET} *.o

