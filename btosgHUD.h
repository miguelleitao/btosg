

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


/// Head Up Display
class btosgHUD : public osg::Projection {
private:
    // A geometry node for the HUD
    osg::Geode* geode;
protected:
    int x1 = 0;  	///< left 
    int y1 = 0;		///< bottom
    int x2 = 1024;	///< right
    int y2 = 150;	///< top
    osg::Vec4 back_color = osg::Vec4(1.f,1.f,1.f,1.0f);		///< Bachground color
    osg::Texture2D* backgroundTexture = NULL;			///< Background Texture
    osg::StateSet* HUDStateSet;					///< StateSet for HUD visual properties
public:
    void setPosition( int x1, int y1, int x2, int y2) {
        /// Positions the HUD in the display window.
        /// x1, x2, y1, y2 are interpreted as pixel coordinates.
        this->x1 = x1;
        this->y1 = y1;
        this->x2 = x2;
        this->y2 = y2;
        // Initialize an orthographic projection matrix.
        setMatrix(osg::Matrix::ortho2D(0,x2-x1,0,y2-y1));
    }
    void setFullScreen(osg::Viewport *vp) {	
        /// Fill the whole viewport
        setPosition( vp->x(), vp->y(), vp->x()+vp->width(), vp->y()+vp->height() );
    }
    void setColor( osg::Vec4 &c ) {
        /// Define the background color
        back_color = c;
    }
    void setBackground(const char* back_fname) {
        /// Load the image from file back_fname to be used as background
        osg::Image* hudImage;
        hudImage = osgDB::readImageFile(back_fname);
        setBackground(hudImage);
    }
    void clearBackground() {
    	/// Clears the defined background image
        if ( backgroundTexture ) {
            HUDStateSet->
                setTextureAttributeAndModes(0,backgroundTexture,osg::StateAttribute::OFF);
        }
    }    
    void setBackground(osg::Image* hudImage) {
        /// Use hudImage as background image
        if ( backgroundTexture ) {
            backgroundTexture->setImage(hudImage);    
            return;
        }
        backgroundTexture = new osg::Texture2D;
        backgroundTexture->setDataVariance(osg::Object::DYNAMIC);
        backgroundTexture->setImage(hudImage);
        
        /// Set up geometry for the HUD and add it to the HUD
        osg::Geometry* HUDBackgroundGeometry = new osg::Geometry();

        osg::Vec3Array* HUDBackgroundVertices = new osg::Vec3Array;
        HUDBackgroundVertices->push_back( osg::Vec3( x1, y1, -1) );
        HUDBackgroundVertices->push_back( osg::Vec3( x2, y1, -1) );
        HUDBackgroundVertices->push_back( osg::Vec3( x2, y2, -1) );
        HUDBackgroundVertices->push_back( osg::Vec3( x1, y2, -1) );

        osg::DrawElementsUInt* HUDBackgroundIndices =
            new osg::DrawElementsUInt(osg::PrimitiveSet::POLYGON, 0);
        HUDBackgroundIndices->push_back(0);
        HUDBackgroundIndices->push_back(1);
        HUDBackgroundIndices->push_back(2);
        HUDBackgroundIndices->push_back(3);

        osg::Vec4Array* HUDcolors = new osg::Vec4Array;
        HUDcolors->push_back(back_color);

        osg::Vec2Array* texcoords = new osg::Vec2Array(4);
        (*texcoords)[0].set(0.0f,0.0f);
        (*texcoords)[1].set(1.0f,0.0f);
        (*texcoords)[2].set(1.0f,1.0f);
        (*texcoords)[3].set(0.0f,1.0f);
        HUDBackgroundGeometry->setTexCoordArray(0,texcoords);

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
        HUDStateSet = new osg::StateSet();
        geode->setStateSet(HUDStateSet);
        HUDStateSet->
            setTextureAttributeAndModes(0,backgroundTexture,osg::StateAttribute::ON);

        // For this state set, turn blending on (so alpha texture looks right)
        HUDStateSet->setMode(GL_BLEND,osg::StateAttribute::ON);

        // Disable depth testing so geometry is always drawn.
        HUDStateSet->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
        //HUDStateSet->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );

        // Need to make sure this geometry is draw last. Use RenderBin 11.
        HUDStateSet->setRenderBinDetails( 11, "RenderBin");
    }
    void setBackground() {
        /// Selects default background image
    	setBackground("metal.png");
    }
    btosgHUD() {
        /// Constructs an Head Up Display.

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
    bool addDrawable( osg::Drawable *td ) {
        /// Add a visual element to the HUD.
        return geode->addDrawable(td);
    }
    virtual bool removeDrawable( osg::Drawable *td ) {
        /// Removes Drawable element from HUD
        return geode->removeDrawable(td);
    }
    void setStateSet(osg::StateSet *sSet) {
        /// Use StateSet to define visual properties for HUD 
        geode->setStateSet(sSet);
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


