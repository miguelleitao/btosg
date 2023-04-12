
/*
	btosHeightfield.cpp
	Miguel Leitao, 2021
*/

#include "btosg.h"

//btosgHeightfield::btosgHeightfield(float x_size, float y_size, float z_size) {
void btosgHeightfield::sizeSetup(float x_size, float y_size, float z_size) {
        /// Constructs a physical and visual highfield with dimensions dx,dy,dz.
        /// Minimum dimension selects physical plane orientation.
        /// Physichal shape implemented from btHeightfieldTerrainShape and
        /// Graphical model implemented from osg::HeightField.
        /// HeightField is created as axis oriented.
        xSize = max(x_size, 0.001);
        ySize = max(y_size, 0.001);
        zSize = max(z_size, 0.001);
         
        maxHeight =  zSize/2.;
        minHeight = -zSize/2.;
}

void btosgHeightfield::graphicSetup() {
	// Graphics
        geode = new osg::Geode();
        if ( geode ) {
            //osg::Shape *sp = new osg::Box( osg::Vec3(0.,0.,0.), dx, dy, dz );
            hField = new osg::HeightField();
            if ( hField ) {
                hField->allocate(xSteps, ySteps);
                hField->setOrigin(osg::Vec3(-xSize/2., -ySize/2., 0));
    		hField->setXInterval(xInterval);
    		hField->setYInterval(yInterval);
    		hField->setSkirtHeight(1.0f);
    		/*
                osg::ShapeDrawable *sd = new osg::ShapeDrawable(hField);
                if ( sd )
                    geode->addDrawable(sd);
                else fprintf(stderr, "Error creating osg::ShapeDrawable\n");
                */
            } else fprintf(stderr, "Error creating osg::HeightField\n");
        } else fprintf(stderr, "Error creating Geode\n");
        if ( !model )	model = new osg::PositionAttitudeTransform;
        model->addChild(geode);
        model->setNodeMask(ReceivesShadowTraversalMask);
}
void btosgHeightfield::graphicSetup2() {
        osg::ShapeDrawable *sd = new osg::ShapeDrawable(hField);
                if ( sd )
                    geode->addDrawable(sd);
                else fprintf(stderr, "Error creating osg::ShapeDrawable\n");
}

void btosgHeightfield::physicSetup() {
        // Physics
        btHeightfieldTerrainShape *hfShape; 
        hfShape = new btHeightfieldTerrainShape(xSteps, ySteps, data, heightScale, minHeight, maxHeight, upAxis, heightDataType, false);
        if ( !hfShape ) fprintf(stderr, "Error creating btShape\n");
        hfShape->setUseDiamondSubdivision(true);
        hfShape->setLocalScaling(btVector3(xInterval, yInterval, 1.));
        shape = hfShape;
    
        createRigidBody();
}

/** HeightField constructor
 *     @param x_size X dimension
 *     @param y_size Y dimension
 *     @param z_size Z dimension
 *     @param x_steps Number of samples (heights) in X direction
 *     @param y_steps Number of samples (heights) in Y direction
 */
btosgHeightfield::btosgHeightfield(float x_size, float y_size, float z_size, int x_steps, int y_steps) {
   // example https://snipplr.com/view/30974/osg-height-field-example 
   
   	//btosgHeightfield(x_size, y_size, z_size)
	sizeSetup(x_size, y_size, z_size);
	
        xSteps = x_steps;
        ySteps = y_steps;
        
        xInterval = xSize/(xSteps-1);
        yInterval = ySize/(ySteps-1);
        
        data = new float[xSteps*ySteps]; 
        for( int y=0 ; y<ySteps ; y++ )
            for( int x=0 ; x<xSteps ; x++ ) 
                data[y*xSteps+x] = 0.;
	
	// Graphics
	graphicSetup();
        
        // Physics
        physicSetup();
}


/** HeightField constructor from external imagemap file
 * 	@param x_size X dimension
 * 	@param y_size Y dimension
 * 	@param z_size Z dimension
 * 	@param fname ImageMap filename
 */
btosgHeightfield::btosgHeightfield(float x_size, float y_size, float z_size, const char *fname) {
   printf("constructor\n");
   printf("    fname='%s'\n", fname);
   	//btosgHeightfield(x_size, y_size, z_size)
	sizeSetup(x_size, y_size, z_size);
	
	osg::Image* heightMap = osgDB::readImageFile(fname);
	if ( ! heightMap ) {
	    fprintf(stderr, "Could not load heightMap file '%s'\n", fname);
	    exit(1);
	}
	
        xSteps = heightMap->s();
        ySteps = heightMap->t();
        
        xInterval = xSize/(xSteps-1);
        yInterval = ySize/(ySteps-1);
        
        data = new float[xSteps*ySteps];
	
	// Graphics
	graphicSetup();
        
        // Physics
        physicSetup();

        setHeightsImage(heightMap);
}
    
    

void btosgHeightfield::printAABB() {
	/// Print HeightFiled Axis Aligned Bounding Boxes
        btTransform trans;
        trans.setIdentity();
        btVector3 aabbMin, aabbMax;
        shape->getAabb(trans, aabbMin, aabbMax);
        printf("aabbMin %f %f %f\n", aabbMin[0], aabbMin[1], aabbMin[2]);
        printf("aabbMax %f %f %f\n", aabbMax[0], aabbMax[1], aabbMax[2]);
        
        osg::BoundingBox bb = geode->getBoundingBox();
        printf("osgbb Min %f %f %f\n", bb.xMin(), bb.yMin(), bb.zMin());
        printf("osgbb Max %f %f %f\n", bb.xMax(), bb.yMax(), bb.zMax());
}
    
void btosgHeightfield::setHeight(int x, int y, double height) {
    /// Set Height of single sample in Heightfield.
    /// Height is bounded to `[minHeight, maxHeight]`
    
    if ( height>maxHeight ) {
        fprintf(stderr, "btosgHeightfield::setHeight(%d,%d,%f): height>%f\n",
        	x, y, height, maxHeight);
        height = maxHeight;
    }
    if ( height<minHeight )  {
        fprintf(stderr, "btosgHeightfield::setHeight(%d,%d,%f): height<%f\n",
        	x, y, height, minHeight);
        height = minHeight;
    }
    if ( ! hField ) {
        fprintf(stderr, "Invalid hField\n");
        return;
    }
    hField->setHeight(x, y, height);
    data[y*xSteps + x] = (float)height;
}

/** Populate HeightField using a second order bi-polynomial function.
 *	@param ax Second order `x` coeficient. Coeficient to x²
 *	@param ay Second order `y` coeficient. Coeficient to y²
 *	@param bx First order  `x` coeficient. Coeficient to x
 *	@param by First order  `y` coeficient. Coeficient to y
 *	@param c Order zero coeficient. Constant term
 *
 *  Each height [h(x,y)] is calculated through expression:
 *       `h(x,y)=ax*x²+ay*y²+bx*x+by*y+c`
 */
int btosgHeightfield::setHeightsParabola(float ax, float ay, float bx, float by, float c) {
    // printf("Setting Heightfield data\n");
    for( int y=0 ; y<ySteps ; y++ )
        for( int x=0 ; x<xSteps ; x++ ) {
	    double xo = (double)x/(double)xSteps - 0.5;
	    double yo = (double)y/(double)ySteps - 0.5;
	    double hi = ax*xo*xo + ay*yo*yo + bx*xo + by*yo + c;
	    setHeight(x, y, hi);
        }
    // printf("Heightfield data defined\n");    
    graphicSetup2();
    return 0;    
}

/** Populate HeightField from a loaded HeightMap.
 *	@param heightMap Pointer to already loaded osg::Image containing the HeightMap
 *
 *  This function can only be used when sizes (numbers os samples) of HeightField and HeightMap matches.  
 */
int  btosgHeightfield::setHeightsImage(osg::Image* heightMap) {
    if ( ! heightMap ) return 1;
    //printf("Setting Heightfield data from file(%d,%d)\n", xSteps, ySteps);
    for( int y=0; y<ySteps ; y++) {
        for( int x=0; x<xSteps ; x++) {       
            unsigned char v = *(heightMap->data(x, y));
            setHeight(x, y, v / 255.f * zSize + minHeight);
        }
    }
    printf("Heightfield data defined\n"); 
    graphicSetup2();
    return 0;
}

/** Populate HeightField from an HeightMap file.
 *	@param fname ImageMap filename
 *
 *  This function can only be used when sizes (numbers of samples) of HeightField and HeighMap matches.
 *  Similar to osgDB::readRefHieghtFieldFile(), bu also keep heights in private data array.
 */
int  btosgHeightfield::loadImageHeights(const char *fname) {
    osg::Image* heightMap = osgDB::readImageFile(fname);
    // consider using osgDB::readRefImageFile() instead.
    if ( ! heightMap ) {
        fprintf(stderr, "Image '%s' could not be loaded.\n", fname);
        return 1;
    }   
    return setHeightsImage(heightMap);
}

