
TARGET=libloadOBJ.a
OBJS=b3ResourcePath.o ImportObjExample.o LoadMeshFromObj.o stb_image.o Wavefront2GLInstanceGraphicsShape.cpp tiny_obj_loader.o Wavefront2GLInstanceGraphicsShape.o


CXXFLAGS=-I include -I include/extra -I include/extra/extra -I /usr/include/bullet

${TARGET}: ${OBJS}
	ar cr $@ $^

clean:
	rm -f ${TARGET} *.o

