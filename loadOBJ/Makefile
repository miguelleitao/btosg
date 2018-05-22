

TARGET=libloadOBJ.a
OBJS=b3ResourcePath.o LoadMeshFromObj.o Wavefront2GLInstanceGraphicsShape.cpp tiny_obj_loader.o Wavefront2GLInstanceGraphicsShape.o
#OBJS=b3ResourcePath.o LoadMeshFromObj.o stb_image.o Wavefront2GLInstanceGraphicsShape.cpp tiny_obj_loader.o Wavefront2GLInstanceGraphicsShape.o

CXXFLAGS=-Wall -I include -I include/extra -I include/extra/extra ${INC_BULLET} -Wno-unused-variable


BULLET_DIR?=/usr
INC_BULLET?=$(shell pkg-config --cflags-only-I bullet)
LIB_BULLET_DIR?=$(shell pkg-config --libs-only-L bullet)
VERSION:=$(shell git tag)



${TARGET}: ${OBJS}
	ar cr $@ $^

clean:
	$(RM) ${TARGET} *.o

