

#include <osgUtil/Optimizer>

#include <osgDB/ReadFile>
#include <osgDB/Registry>

#include <osg/Geode>
#include <osg/Projection>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>

#include <osgText/Font>
#include <osgText/Text>


class btosgHUD : public osg::Projection {
    private:
 	// A geometry node for the HUD
	osg::Geode* geode; 
    public:
	void setBackground() {
	   // Set up geometry for the HUD and add it to the HUD
	   osg::Geometry* HUDBackgroundGeometry = new osg::Geometry();

	   osg::Vec3Array* HUDBackgroundVertices = new osg::Vec3Array;
	   HUDBackgroundVertices->push_back( osg::Vec3( 0,    0,-1) );
	   HUDBackgroundVertices->push_back( osg::Vec3(1024,  0,-1) );
	   HUDBackgroundVertices->push_back( osg::Vec3(1024,150,-1) );
	   HUDBackgroundVertices->push_back( osg::Vec3(   0,150,-1) );

	   osg::DrawElementsUInt* HUDBackgroundIndices =
	      new osg::DrawElementsUInt(osg::PrimitiveSet::POLYGON, 0);
	   HUDBackgroundIndices->push_back(0);
	   HUDBackgroundIndices->push_back(1);
	   HUDBackgroundIndices->push_back(2);
	   HUDBackgroundIndices->push_back(3);

	   osg::Vec4Array* HUDcolors = new osg::Vec4Array;
	   HUDcolors->push_back(osg::Vec4(0.8f,0.8f,0.8f,0.8f));

	   osg::Vec2Array* texcoords = new osg::Vec2Array(4);
	   (*texcoords)[0].set(0.0f,0.0f);
	   (*texcoords)[1].set(1.0f,0.0f);
	   (*texcoords)[2].set(1.0f,1.0f);
	   (*texcoords)[3].set(0.0f,1.0f);

	   HUDBackgroundGeometry->setTexCoordArray(0,texcoords);
	   osg::Texture2D* HUDTexture = new osg::Texture2D;
	   HUDTexture->setDataVariance(osg::Object::DYNAMIC);
	   osg::Image* hudImage;
	   hudImage = osgDB::readImageFile("metal4.png");
	   HUDTexture->setImage(hudImage);
	   osg::Vec3Array* HUDnormals = new osg::Vec3Array;
	   HUDnormals->push_back(osg::Vec3(0.0f,0.0f,1.0f));
	   HUDBackgroundGeometry->setNormalArray(HUDnormals);
	   HUDBackgroundGeometry->setNormalBinding(osg::Geometry::BIND_OVERALL);
	   HUDBackgroundGeometry->addPrimitiveSet(HUDBackgroundIndices);
	   HUDBackgroundGeometry->setVertexArray(HUDBackgroundVertices);
	   HUDBackgroundGeometry->setColorArray(HUDcolors);
	   HUDBackgroundGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);

	   geode->addDrawable(HUDBackgroundGeometry);

	   // Create and set up a state set using the texture:
	   osg::StateSet* HUDStateSet = new osg::StateSet();
	   geode->setStateSet(HUDStateSet);
	   HUDStateSet->
	      setTextureAttributeAndModes(0,HUDTexture,osg::StateAttribute::ON);

	   // For this state set, turn blending on (so alpha texture looks right)
	   HUDStateSet->setMode(GL_BLEND,osg::StateAttribute::ON);

	   // Disable depth testing so geometry is always drawn.
	   HUDStateSet->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
	   HUDStateSet->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );

	   // Need to make sure this geometry is draw last. Use RenderBin 11.
	   HUDStateSet->setRenderBinDetails( 11, "RenderBin");
	}
    	btosgHUD() {
	   // Initialize the projection matrix for viewing everything we
	   // will add as descendants of this node. Use screen coordinates
	   // to define the horizontal and vertical extent of the projection
	   // matrix. Positions described under this node will equate to
	   // pixel coordinates.
	   setMatrix(osg::Matrix::ortho2D(0,1024,0,768));
	  
	   // For the HUD model view matrix use an identity matrix:
	   osg::MatrixTransform* HUDModelViewMatrix = new osg::MatrixTransform;
	   HUDModelViewMatrix->setMatrix(osg::Matrix::identity());

	   // Make sure the model view matrix is not affected by any transforms
	   // above it in the scene graph:
	   HUDModelViewMatrix->setReferenceFrame(osg::Transform::ABSOLUTE_RF);

	   // Add the HUD model view matrix as a child.
	   // Anything under this node will be viewed using this projection matrix
	   // and positioned with this model view matrix.
	   addChild(HUDModelViewMatrix);

	   // Create and add the Geometry node to contain HUD geometry as a child of the
	   // HUD model view matrix.
	   geode = new osg::Geode();
	   HUDModelViewMatrix->addChild( geode );
    	}
	virtual bool addDrawable( osg::Drawable *td ) {
	   return geode->addDrawable(td);	   
	}
};

/*	Text example
 *
   // Text instance to show up in the HUD:
   osgText::Text* textOne = new osgText::Text();
   textOne->setCharacterSize(25);
   textOne->setFont("arial.ttf");
   textOne->setText("btosg text example");
   textOne->setAxisAlignment(osgText::Text::SCREEN);
   textOne->setPosition( osg::Vec3(360., 100., -1) );
   textOne->setColor( osg::Vec4(1., 1., 0., 1.) );

   // Add the text to the HUD:
   myHUD->addDrawable( textOne );
  
*/ 


