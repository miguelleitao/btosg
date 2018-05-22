
BULLET_DIR?=/usr
INC_BULLET?=-I${BULLET_DIR}/include/bullet

TARGET=libloadOBJ.a
OBJS=b3ResourcePath.o LoadMeshFromObj.o stb_image.o Wavefront2GLInstanceGraphicsShape.cpp tiny_obj_loader.o Wavefront2GLInstanceGraphicsShape.o

CXXFLAGS=-I include -I include/extra -I include/extra/extra ${INC_BULLET}

${TARGET}: ${OBJS}
	ar cr $@ $^

clean:
	rm -f ${TARGET} *.o

