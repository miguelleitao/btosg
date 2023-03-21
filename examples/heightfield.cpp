/*
	heightfield.cpp
	Miguel Leitao, 2019
*/

#include <osgViewer/Viewer>
#include <osg/Material>
#include "btosg.h"

#define _DEBUG_ (0)


double frame_time = 0.;

// Create World
btosgWorld myWorld;

int main(int argc, char **argv)
{
    const btosgVec3 up(0., 0., 1.);
    const btosgVec3 gravity = up*-9.8;
    myWorld.dynamic->setGravity(gravity);

    // Balls
    for( int y=-10 ; y<=10 ; y++ )
        for( int x=-3 ; x<=3 ; x++ ) {
            btosgSphere *myBall = new btosgSphere(1.);
            char oname[20];
            sprintf(oname, "Ball_%02d%02d", y, x);
            myBall->setName(oname);
            myBall->setMass(100);
            myBall->setTexture("img/beachball.png");
            myBall->setPosition((float)x*2, (float)y*2, 15.);
            myBall->body->setRestitution(0.5);
            myWorld.addObject( myBall );
        }

    // Material for base surface
    osg::ref_ptr<osg::Material> mat = new osg::Material;
    mat->setAmbient (osg::Material::FRONT_AND_BACK, osg::Vec4(0., 0., 0., 1.0));
    mat->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(0.7, 0.8, 0.0, 1.0));
    mat->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 1.0));
    mat->setShininess(osg::Material::FRONT_AND_BACK, 64);

    // Curved surface defined from an HeightField
    btosgHeightfield *myHfield;    
    if ( argc==1 ) {
    	// No HeightField Map.
    	// Generate bi-quadratic map with 100x100 samples. 
        myHfield = new btosgHeightfield(40., 50., 60., 100, 100);
        myHfield->setHeightsParabola(10., 75.);
        
    }
    else {
    	// Build HeightField from given Image Map. 
        myHfield = new btosgHeightfield(100., 100., 40., argv[1]);
    }
    myHfield->setPosition(0., 0., 0.);
    myHfield->setName("HeightField");
    myHfield->printAABB();
    myHfield->body->setFriction(10.);
    myHfield->setMaterial(mat);
    myHfield->setTexture("img/beachball.png");
    myWorld.addObject( myHfield );

    // Creating the viewer
    osgViewer::Viewer viewer ;
    viewer.setSceneData( myWorld.scene );
    
    // Setup camera
    osg::Matrix matrix;
    matrix.makeLookAt( osg::Vec3(0.,8.,5.), osg::Vec3(0.,0.,1.), up );
    viewer.getCamera()->setViewMatrix(matrix);
    viewer.getCamera()->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );

    // Light
    osg::ref_ptr<osg::LightSource> ls = new osg::LightSource;
    ls->getLight()->setPosition(osg::Vec4(2.5,-10+30*up[1],-10+30.*up[2],1.));
    ls->getLight()->setAmbient(osg::Vec4(0.1, 0.1, 0.1, 1.0));
    ls->getLight()->setDiffuse(osg::Vec4(1.0, 1.0, 1.0, 1.0));
    ls->getLight()->setSpecular(osg::Vec4(0.2, 0.2, 0.2, 1.0));
    myWorld.scene->addChild(ls.get());

    // Manipulator
    osg::ref_ptr<osgGA::TrackballManipulator> manipulator = new osgGA::TrackballManipulator;
    viewer.setCameraManipulator( manipulator );
    // Set the desired home coordinates for the manipulator
    osg::Vec3d eye(osg::Vec3(-65., 0., 0.)+up*40.);
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
    
    //myWorld.listObjects();

    while( !viewer.done() )
    {
        myWorld.stepSimulation(frame_time,100);

        viewer.frame();
        timenow = myTimer.time_s();
        frame_time = timenow - last_time;
        last_time = timenow;
    }
    printf("main loop exited\n");
}

