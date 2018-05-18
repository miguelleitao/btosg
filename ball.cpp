/*
	ball.cpp 
	Miguel Leitao, 2012
*/


#include <osgViewer/Viewer> 
#include <osg/Material>

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

#include <osg/Material>
#include <osg/Texture2D>

#include "btosg.h"

#define _DEBUG_ (0)


int ResetFlag=0;
double frame_time = 0.;

// Create World
btosgWorld myWorld;

btosgSphere *myBall;




class BlockGreen : public btosgBox {
    public:
        BlockGreen(float x, float y, float z) : btosgBox( osg::Vec3(1.,1.,1.)*0.2, 10. ) {
            setPosition(btVector3(x,y,z));
            osg::ref_ptr<osg::Material> mat = new osg::Material;
            mat->setAmbient (osg::Material::FRONT_AND_BACK, osg::Vec4(0., 0., 0., 1.0));
            mat->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(0.1, 0.5, 0.1, 1.0));
            mat->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 1.0));
            mat->setShininess(osg::Material::FRONT_AND_BACK, 64);
            model->getOrCreateStateSet()->
                setAttributeAndModes(mat, osg::StateAttribute::ON);
        }
        BlockGreen(float x, float z) : BlockGreen(x,1.,z) {};
};

class BlockRed : public btosgBox {
    public:
        BlockRed(float x, float y, float z) : btosgBox( osg::Vec3(1.,1.,1.)*0.2, 100. ) {
            setPosition(btVector3(x,y,z));
            osg::ref_ptr<osg::Material> mat = new osg::Material;
            mat->setAmbient (osg::Material::FRONT_AND_BACK, osg::Vec4(0., 0., 0., 1.0));
            mat->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(0.6, 0.1, 0.1, 1.0));
            mat->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 1.0));
            mat->setShininess(osg::Material::FRONT_AND_BACK, 64);
            model->getOrCreateStateSet()->
                setAttributeAndModes(mat, osg::StateAttribute::ON);
        }
        BlockRed(float x, float z) : BlockRed(x,1.,z) {};
};

class BlockBlue : public btosgBox {
    public:
        BlockBlue(float x, float y, float z) : btosgBox( osg::Vec3(1.,0.25,1.)*0.2, 100. ) {
            setPosition(btVector3(x,y,z));
            osg::ref_ptr<osg::Material> mat = new osg::Material;
            mat->setAmbient (osg::Material::FRONT_AND_BACK, osg::Vec4(0., 0., 0., 1.0));
            mat->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(0.1, 0.1, 0.5, 1.0));
            mat->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 1.0));
            mat->setShininess(osg::Material::FRONT_AND_BACK, 64);
            model->getOrCreateStateSet()->
                setAttributeAndModes(mat, osg::StateAttribute::ON);
        }
        BlockBlue(float x, float z) : BlockBlue(x,1.,z) {};
};


// class to handle events
class EventHandler : public osgGA::GUIEventHandler
{
    public:
    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {
        osgViewer::Viewer* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);
        if (!viewer) return false;
        switch(ea.getEventType())
        {
            case(osgGA::GUIEventAdapter::KEYDOWN):
                switch ( ea.getKey() ) {
                    case osgGA::GUIEventAdapter::KEY_Down:
                        return false;
                    case osgGA::GUIEventAdapter::KEY_Up:
                        return false;
                    case osgGA::GUIEventAdapter::KEY_Left:
                        return false;
                    case osgGA::GUIEventAdapter::KEY_Right:
                        return false;
                    case 'b':
                    case '0':
                    case osgGA::GUIEventAdapter::KEY_Control_R:
                        return false;
                }
                break;
	case(osgGA::GUIEventAdapter::KEYUP):
		switch ( ea.getKey() ) {
                    case osgGA::GUIEventAdapter::KEY_Down:
                        myBall->body->activate(true);
                        myBall->body->applyCentralImpulse(btVector3(0.,-1.,0.));
                        return false;
                    case osgGA::GUIEventAdapter::KEY_Up:
                        myBall->body->activate(true);
                        myBall->body->applyCentralImpulse(btVector3(0.,2.,0.));
                        return false;
                    case osgGA::GUIEventAdapter::KEY_Left:
                        myBall->body->activate(true);
                        myBall->body->applyCentralImpulse(btVector3(-1.,0.,0.));
                        return false;
                    case osgGA::GUIEventAdapter::KEY_Right:
                        myBall->body->activate(true);
                        myBall->body->applyCentralImpulse(btVector3(1.,0.,0.));
                        return false;
                    case '0':
                    case 'b':
                    case osgGA::GUIEventAdapter::KEY_Control_R:
                        return false;
                    case 'S':
			std::cout << "tecla S" << std::endl;
			return false;
                    case 'i':
                        
                        break;
                    case 'F':
                            std::cout << "adding Force" << std::endl;

                            myBall->body->activate(true);
                            myBall->body->applyCentralImpulse(btVector3(100.,0.,0.));
                           
                            // handled = true;
                            return false;
                    case 'R':
		    case 'r':
			   ResetFlag = 1;
			   std::cout << "tecla R" << std::endl;
			   break;
		}
		case(osgGA::GUIEventAdapter::MOVE):
			//std::cout << "mouse move " << ea.getX() << " " << ea.getY() << std::endl;
			return false;
		default:
			return false;
	    }
	    return true;
	}
};


int main()
{
    osg::Vec3 up(0., 0., 1.);
    myWorld.dynamic->setGravity(osg2bt_Vec3(up)*-9.8);

    // Ball
    myBall = new btosgSphere(0.2);
    myBall->setMass(0.01);
    myBall->setTexture("beachball.png");
    myBall->setPosition(0.,-1.,2.);
    myWorld.addObject( myBall );


    // Ball
    myBall = new btosgSphere(0.1085);
    myBall->setPosition(0.0, -9., 5.);
    myBall->setTexture("ball.png");
    myBall->setMass(6.5);
    myBall->setName("BowlingBall");
    myWorld.addObject(myBall);




    // Pins
    btosgBowlingPin *myPin[10];
    int x,y,p = 0;
    float space = 12 * 0.0254;  // 12 inches
    for( y=0 ; y<4 ; y++ ) {
        for( x=0 ; x<=y ; x++ ) {
            myPin[p] = new btosgBowlingPin();
            myPin[p]->setName("pin");
            myPin[p]->setPosition(0.+(x-y/2.)*space, -1.4+(y)*space, 0.20);
            myWorld.addObject( myPin[p] );
            p += 1;
        }
    }



    {
	    BlockGreen *myBlock;
	    myBlock = new BlockGreen(4.,-4.);
	    myWorld.addObject(myBlock);
	    myBlock = new BlockGreen(6.,-5.);
	    myWorld.addObject(myBlock);
	    myBlock = new BlockGreen(0.,0.);
	    myWorld.addObject(myBlock);
	    myBlock = new BlockGreen(9.,-5.);
	    myWorld.addObject(myBlock);
	    myBlock = new BlockGreen(1.,1.);
	    myWorld.addObject(myBlock);
	    myBlock = new BlockGreen(-1.,-6.);
	    myWorld.addObject(myBlock);
    }
    {
	    BlockRed *myBlock;
	    myBlock = new BlockRed(4.,4.);
	    myWorld.addObject(myBlock);
	    myBlock = new BlockRed(7.,5.);
	    myWorld.addObject(myBlock);
	    myBlock = new BlockRed(-8.,5.);
	    myWorld.addObject(myBlock);
	    myBlock = new BlockRed(9.,-5.);
	    myWorld.addObject(myBlock);
	    myBlock = new BlockRed(1.,-6.);
	    myWorld.addObject(myBlock);
	    myBlock = new BlockRed(-2.,6.);
	    myWorld.addObject(myBlock);
    }

    {
	    BlockBlue *myBlock;
	    myBlock = new BlockBlue(4.,-4.);
	    myWorld.addObject(myBlock);
	    myBlock = new BlockBlue(7.5,6.);
	    myWorld.addObject(myBlock);
	    myBlock = new BlockBlue(-8.,-5.);
	    myWorld.addObject(myBlock);
	    myBlock = new BlockBlue(9.,5.);
	    myWorld.addObject(myBlock);
	    myBlock = new BlockBlue(1.,-7.);
	    myWorld.addObject(myBlock);
	    myBlock = new BlockBlue(-3.,7.);
	    myWorld.addObject(myBlock);
    }




    // Plane
    btosgPlane *myRamp = new btosgPlane();
    myRamp->setRotation(osg::Quat(0.,osg::Vec3(1.,0.,0.)));
    myRamp->setPosition(0.,-5.,0.);
    myWorld.addObject( myRamp );
    myRamp->setName("Ramp");
    myRamp->body->setFriction(100.);
    osg::ref_ptr<osg::Material> matRamp = new osg::Material;
    matRamp->setAmbient (osg::Material::FRONT_AND_BACK, osg::Vec4(0., 0., 0., 1.0));
    matRamp->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(0.7, 0.8, 0.0, 1.0));
    matRamp->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 1.0));
    matRamp->setShininess(osg::Material::FRONT_AND_BACK, 64);
    myRamp->model->getOrCreateStateSet()->
        setAttributeAndModes(matRamp, osg::StateAttribute::ON);

        

    btosgBox *myBox = new btosgBox(0.04,10.,0.2);
    myBox->setPosition(1.5,-5.,0.1);
    myBox->setMass(0.);
    myWorld.addObject( myBox );

    myBox = new btosgBox(0.04,10.,0.2);
    myBox->setPosition(-1.5,-5.,0.1);
    myBox->setMass(0.);
    myWorld.addObject( myBox );

/*
    myRamp = new btosgPlane();
    myRamp->setRotation(osg::Quat(osg::PI/4.,osg::Vec3(1.,0.,0.)));
    myRamp->setPosition(0.,2.,0.);
    myWorld.addObject( myRamp );
    myRamp->setName("Ramp");
    myRamp->body->setFriction(100.);
    myRamp->model->getOrCreateStateSet()->
        setAttributeAndModes(matRamp, osg::StateAttribute::ON);


    myRamp = new btosgPlane();
    myRamp->setRotation(osg::Quat(osg::PI/4.,osg::Vec3(0.,1.,0.)));
    myRamp->setPosition(-2.,0.,0.);
    myWorld.addObject( myRamp );
    myRamp->setName("Ramp");
    myRamp->body->setFriction(100.);
    myRamp->model->getOrCreateStateSet()->
        setAttributeAndModes(matRamp, osg::StateAttribute::ON);

   myRamp = new btosgPlane();
    myRamp->setRotation(osg::Quat(-osg::PI/4.,osg::Vec3(0.,1.,0.)));
    myRamp->setPosition(2.,0.,0.);
    myWorld.addObject( myRamp );
    myRamp->setName("Ramp");
    myRamp->body->setFriction(100.);
    myRamp->model->getOrCreateStateSet()->
        setAttributeAndModes(matRamp, osg::StateAttribute::ON);
*/

        // Creating the viewer
	osgViewer::Viewer viewer ;

	// Setup camera
	osg::Matrix matrix;
	matrix.makeLookAt( osg::Vec3(0.,8.,5.), osg::Vec3(0.,0.,1.), up );
	viewer.getCamera()->setViewMatrix(matrix);

	// add the Event handler
	viewer.addEventHandler(new EventHandler());

        // Light
        osg::ref_ptr<osg::LightSource> ls = new osg::LightSource;
        ls->getLight()->setPosition(osg::Vec4(2.5,-10+30*up[1],-10+30.*up[2],1.)); 
        ls->getLight()->setAmbient(osg::Vec4(0.1, 0.1, 0.1, 1.0));
        ls->getLight()->setDiffuse(osg::Vec4(1.0, 1.0, 1.0, 1.0));
        ls->getLight()->setSpecular(osg::Vec4(0.2, 0.2, 0.2, 1.0));
        myWorld.scene->addChild(ls.get());

        viewer.setSceneData( myWorld.scene );

	viewer.getCamera()->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR ); 
        
        osg::ref_ptr<osgGA::TrackballManipulator> manipulator = new osgGA::TrackballManipulator;
        viewer.setCameraManipulator( manipulator );

        // Set the desired home coordinates for the manipulator
        osg::Vec3d eye(osg::Vec3(0., -5., -5.)+up*20.);
        osg::Vec3d center(0., 0., 0.);

        // Make sure that OSG is not overriding our home position
        manipulator->setAutoComputeHomePosition(false);
        // Set the desired home position of the Trackball Manipulator
        manipulator->setHomePosition(eye, center, up);
        // Force the camera to move to the home position
        manipulator->home(0.0);
        
	// record the timer tick at the start of rendering.
	osg::Timer myTimer;
	double timenow = myTimer.time_s();
	double last_time = timenow;
	frame_time = 0.;
    
        //suspBL->enableFeedback(1);

        while( !viewer.done() )
	{
	 	myWorld.stepSimulation(frame_time,10);
                
	  	viewer.frame();
	  	timenow = myTimer.time_s();
	  	frame_time = timenow - last_time;
	  	last_time = timenow;
        
		if (ResetFlag>0) {
		    //Reset(); 
                    myWorld.reset();
		    ResetFlag = 0;
		}
	}
}

