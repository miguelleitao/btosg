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

int main()
{
    btosgVec3 up(0., 0., 1.);
    btosgVec3 gravity = up*-9.8;
    //myWorld.dynamic->setGravity(osg2bt_Vec3(up)*-9.8);
    myWorld.dynamic->setGravity(gravity);

    // Balls
    for( int y=-10 ; y<=10 ; y++ )
        for( int x=-3 ; x<=3 ; x++ ) {
            btosgSphere *myBall = new btosgSphere(0.5);
            char oname[20];
            sprintf(oname,"Ball_%02d%02d", y, x);
            myBall->setName(oname);
            myBall->setMass(0.01);
            myBall->setTexture("img/beachball.png");
            myBall->setPosition((float)x*2, (float)y*2, 15.);
            myBall->body->setRestitution(0.5);
            myWorld.addObject( myBall );
        }

    // Material for base plans
    osg::ref_ptr<osg::Material> matRamp = new osg::Material;
    matRamp->setAmbient (osg::Material::FRONT_AND_BACK, osg::Vec4(0., 0., 0., 1.0));
    matRamp->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(0.7, 0.8, 0.0, 1.0));
    matRamp->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 1.0));
    matRamp->setShininess(osg::Material::FRONT_AND_BACK, 64);
/*
    // Plane 1
    btosgPlane *myRamp;
    myRamp = new btosgPlane();
    myRamp->setRotation(osg::Quat(-osg::PI/8.,osg::Vec3(1.,0.,0.)));
    myRamp->setPosition(0.,-5.,0.);
    myRamp->setName("Ramp1");
    myRamp->body->setFriction(100.);
    myRamp->setMaterial(matRamp);
//    myWorld.addObject( myRamp );
*/
    // Curved surface defined from an HeightField
    btosgHeightfield *myHfield;
    myHfield = new btosgHeightfield(50., 50., 0.1);
    //myHfield->setRotation(osg::Quat(osg::PI/8.,osg::Vec3(1.,0.,0.)));
    myHfield->setPosition(0.,0.,0.);
    myHfield->setName("HeightField");
    myHfield->body->setFriction(100.);
    myHfield->setMaterial(matRamp);
    myWorld.addObject( myHfield );

    // Creating the viewer
    osgViewer::Viewer viewer ;

    // Setup camera
    osg::Matrix matrix;
    matrix.makeLookAt( osg::Vec3(0.,8.,5.), osg::Vec3(0.,0.,1.), up );
    viewer.getCamera()->setViewMatrix(matrix);

    // Light
    osg::ref_ptr<osg::LightSource> ls = new osg::LightSource;
    ls->getLight()->setPosition(osg::Vec4(2.5,-10+30*up[1],-10+30.*up[2],1.));
    ls->getLight()->setAmbient(osg::Vec4(0.1, 0.1, 0.1, 1.0));
    ls->getLight()->setDiffuse(osg::Vec4(1.0, 1.0, 1.0, 1.0));
    ls->getLight()->setSpecular(osg::Vec4(0.2, 0.2, 0.2, 1.0));
    myWorld.scene->addChild(ls.get());

    viewer.setSceneData( myWorld.scene );

    viewer.getCamera()->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );

    // Manipulator
    osg::ref_ptr<osgGA::TrackballManipulator> manipulator = new osgGA::TrackballManipulator;
    viewer.setCameraManipulator( manipulator );
    // Set the desired home coordinates for the manipulator
    osg::Vec3d eye(osg::Vec3(-45., 0., 0.)+up*30.);
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

    while( !viewer.done() )
    {
        myWorld.stepSimulation(frame_time,100);

        viewer.frame();
        timenow = myTimer.time_s();
        frame_time = timenow - last_time;
        last_time = timenow;
	//myWorld.listObjects();
    }
    printf("main loop exited\n");
    //delete manipulator;
    
    printf("mmanipulator deleted\n");

}

