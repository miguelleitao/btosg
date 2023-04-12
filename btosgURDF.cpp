/*
 *	btosgUrdf.cpp
 *
 *	(c) Miguel Leitao, 2023
 *
 */

#include <libxml/parser.h>
#include <libxml/tree.h>
#include "btosg.h"

int debug = 0;


void
getXmlPropFloat(xmlNode *cur_node, const char *vname, float *v) {
    xmlChar *v_str = xmlGetProp(cur_node, (const xmlChar *)vname);
    if ( v_str ) {
	*v = atof((const char*)v_str);
	xmlFree(v_str);
   }
}

/**
 * getUrdfLinkVisualGeometry:
 * @a_node: the initial xml node to consider.
 *
 * Reads the visual block of the link element
 * from the given xml node.
 */
void
btosgObject::getUrdfLinkVisualGeometry(xmlNode * a_node) {
    xmlNode *cur_node = NULL;
    for (cur_node = a_node->children; cur_node; cur_node = cur_node->next) {
        if ( cur_node->type != XML_ELEMENT_NODE ) continue;
        if ( !xmlStrcmp(cur_node->name, (const xmlChar *)"box") ) {
                osg::Vec3 size = osg::Vec3(1., 1., 1.);
                xmlChar *size_str = xmlGetProp(cur_node, (const xmlChar *)"size");
		sscanf((char *)size_str, "%f %f %f", &(size[0]), &(size[1]), &(size[2]));
		xmlFree(size_str);
	        printf("                box\n");
	        setVisualGeometry( new osg::Box( osg::Vec3(0.,0.,0.), size[0], size[1], size[2] ) );
	        continue;
	}
        if ( !xmlStrcmp(cur_node->name, (const xmlChar *)"cylinder") ) {
	        printf("                cylinder\n");
                float len = 1;
                float rad = 1;
		getXmlPropFloat(cur_node, "lenght", &len);
		getXmlPropFloat(cur_node, "radius", &rad);
	        setVisualGeometry(new osg::Cylinder( osg::Vec3(0.,0.,0.), rad, len));
	        continue;
	}
	if ( !xmlStrcmp(cur_node->name, (const xmlChar *)"sphere") ) {
	        printf("                sphere\n");
	        float rad = 1;
	        getXmlPropFloat(cur_node, "radius", &rad);
	        setVisualGeometry(new osg::Sphere( osg::Vec3(0.,0.,0.), rad));
	        continue;
	}
	if ( !xmlStrcmp(cur_node->name, (const xmlChar *)"mesh") ) {
	        printf("                mesh\n");
	        continue;
	}
	printf("        Invalid node in URDF::link::Visual::Geometry: '%s'\n", cur_node->name);
    }
}


/**
 * getUrdfLinkVisual:
 * @a_node: the initial xml node to consider.
 *
 * Reads the visual block of the link element
 * from the given xml node.
 */
void btosgObject::getUrdfLinkVisual(xmlNode * a_node) {
    xmlNode *cur_node = NULL;
    for (cur_node = a_node->children; cur_node; cur_node = cur_node->next) {
        if ( cur_node->type != XML_ELEMENT_NODE ) continue;
        if ( !xmlStrcmp(cur_node->name, (const xmlChar *)"geometry") ) {
	        printf("            geometry\n");
	        getUrdfLinkVisualGeometry(cur_node);
	        continue;
	}
        if ( !xmlStrcmp(cur_node->name, (const xmlChar *)"origin") ) {
	        printf("            origin\n");
                osg::Vec3 pos, hpr;
                xmlChar *pos_str = xmlGetProp(cur_node, (const xmlChar *)"xyz");
		sscanf((char *)pos_str, "%f %f %f", &(pos[0]), &(pos[1]), &(pos[2]));
		xmlFree(pos_str);
                xmlChar *hpr_str = xmlGetProp(cur_node, (const xmlChar *)"xyz");
		sscanf((char *)hpr_str, "%f %f %f", &(hpr[0]), &(hpr[1]), &(hpr[2]));
		xmlFree(hpr_str);
		setVisualOrigin(pos, hpr);
	        continue;
	}
        if ( !xmlStrcmp(cur_node->name, (const xmlChar *)"material") ) {
	        printf("            material\n");
	        continue;
	}
	printf("        Invalid node in URDF::link::Visual: '%s'\n", cur_node->name);
    }
}


/**
 * getUrdfLink:
 * @a_node: the initial xml node to consider.
 *
 * Reads the link element
 * from the given xml node.
 */
void btosgWorld::getUrdfLink(xmlNode * a_node)
{
    btosgObject *n_obj = new btosgObject;
    xmlNode *cur_node = NULL;
    for (cur_node = a_node->children; cur_node; cur_node = cur_node->next) {
        if ( cur_node->type != XML_ELEMENT_NODE) continue;
        if ( !xmlStrcmp(cur_node->name, (const xmlChar *)"visual") ) {
	        printf("        visual\n");
	        n_obj->getUrdfLinkVisual(cur_node);
	        continue;
	}
        if ( !xmlStrcmp(cur_node->name, (const xmlChar *)"inertial") ) {
	        printf("        inertial\n");
	        continue;
	}
        if ( !xmlStrcmp(cur_node->name, (const xmlChar *)"collision") ) {
	        printf("        collision\n");
	        continue;
	}
	printf("        Invalid node in URDF::link: '%s'\n", cur_node->name);
    }
    addObject(n_obj);
}
/**
 * getUrdfElement:
 * @a_node: the initial xml node to consider.
 *
 * Reads all the xml elements
 * that are siblings or children of a given xml node.
 */
void btosgWorld::getUrdfElement(xmlNode * a_node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = a_node; cur_node; cur_node = cur_node->next) {
        if (cur_node->type == XML_ELEMENT_NODE) {
            
            printf("node type: Element, name: %s\n", cur_node->name);
            if (debug) { //(!xmlStrcmp(cur_node->name, (const xmlChar *)"name"))) {
                    xmlChar *name = xmlGetProp(cur_node, (const xmlChar *)"name");
		    printf("name: %s\n", name);
		    xmlFree(name);
	    }
	    if ( !xmlStrcmp(cur_node->name, (const xmlChar *)"link") ) {
	        printf("link\n");
	        getUrdfLink(cur_node);
	        continue;
	    }
	    if ( !xmlStrcmp(cur_node->name, (const xmlChar *)"joint") ) {
	        printf("joint\n");
	        //getUrdfJoint(cur_node);
	        continue;
	    }
	    
	    if ( !xmlStrcmp(cur_node->name, (const xmlChar *)"robot") ) {
        	getUrdfElement(cur_node->children);
	    }
	    
        }
        //print_element_names(cur_node->children);
    }
}
 
int btosgWorld::loadUrdf(const char *fname) {
    /// Loads an URDF hierarchy from fname file.
    
    if ( ! fname || ! *fname ) {
        fprintf(stderr, "Invalid URDF model filename\n");
        return 0;
    }
    /*
     * this initialize the library and check potential ABI mismatches
     * between the version it was compiled for and the actual shared
     * library used.
     */
    LIBXML_TEST_VERSION
    
    xmlDocPtr doc; /* the resulting document tree */

    doc = xmlReadFile(fname, NULL, 0);
    if (doc == NULL) {
        fprintf(stderr, "Failed to parse %s\n", fname);
	return 0;
    }
    
    /*Get the root element node */
    xmlNode *root_element = xmlDocGetRootElement(doc);

    getUrdfElement(root_element);
    
    xmlFreeDoc(doc);
    /*
     * Cleanup function for the XML library.
     */
    xmlCleanupParser();
    /*
     * this is to debug memory for regression tests
     */
    xmlMemoryDump();
  /*  
    osg::Node* loadedModel = osgDB::readNodeFile(fname);
    if ( ! loadedModel ) {
        fprintf(stderr, "Error reading URDF model from file '%s'\n", fname);
    }

    

    setName(fname);
    if ( ! model )	model = new osg::PositionAttitudeTransform;
    osg::PositionAttitudeTransform* obj_rot = new osg::PositionAttitudeTransform;
    obj_rot->setAttitude(osg::Quat(-osg::PI/2.,osg::Vec3(1.,0.,0.)));
    obj_rot->addChild(loadedModel);
    model->addChild(obj_rot);
*/
   return 0;
}
