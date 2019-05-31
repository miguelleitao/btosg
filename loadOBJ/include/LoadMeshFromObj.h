#ifndef LOAD_MESH_FROM_OBJ_H
#define LOAD_MESH_FROM_OBJ_H


struct GLInstanceGraphicsShape;


GLInstanceGraphicsShape* LoadMeshFromObj(const char* relativeFileName, const char* materialPrefixPath);
void btgDeleteGraphicsShape(GLInstanceGraphicsShape* shape);

#endif //LOAD_MESH_FROM_OBJ_H

