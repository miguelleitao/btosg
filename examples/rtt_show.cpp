/*
	objects.cpp
	Miguel Leitao, 2012
*/

#define BTOSG_LOAD_OBJ YES

#include <osgViewer/Viewer>
#include <osg/Material>
#include "btosg.h"

#define _DEBUG_ (0)


double frame_time = 0.;

// Create World
btosgWorld myWorld;

btosgSphere *myBall;


int main()
{
    btosgVec3 up(0., 0., 1.);
    myWorld.dynamic->setGravity(btVector3(0.,0.,0.));

    // Beach Ball
    myBall = new btosgSphere(0.2);
    myBall->setMass(0.01);
    myBall->setTexture("img/beachball.png");
    myBall->setPosition(0.,-4.,5.);
    myWorld.addObject( myBall );


    btosgExternalObject *myObj;
    myObj = new btosgExternalObject("obj/cone.obj");
    myObj->setMass(1.6);
    myObj->setRotation(btQuaternion(btVector3(1.,0.,0.),osg::PI/2.));
    myObj->setPosition(0.5,-4.5,3);
    myWorld.addObject( myObj );
    myObj = new btosgExternalObject("obj/pyramid.obj");
    myObj->setMass(1.6);
    myObj->setRotation(btQuaternion(btVector3(1.,0.,0.),osg::PI));
    myObj->setPosition(0.5,-4.5,3);
    myWorld.addObject( myObj );
    myObj = new btosgExternalObject("obj/cone.obj");
    myObj->setMass(1.6);
    myObj->setRotation(btQuaternion(btVector3(1.,0.,0.),-osg::PI/2.));
    myObj->setPosition(1.5,-3.5,3);
    myWorld.addObject( myObj );
    myObj = new btosgExternalObject("obj/torus.obj");
    myObj->setMass(1.6);
    myObj->setRotation(btQuaternion(btVector3(1.,0.,0.),-0.*osg::PI/2.));
    myObj->setPosition(1.5,-4.5,6.);
    myWorld.addObject( myObj );
    myObj = new btosgExternalObject("obj/octahedron.obj");
    myObj->setMass(0.8);
    myObj->setPosition(1.5,-4.5,3.);
    myWorld.addObject( myObj );

    // Material for base plans
    osg::ref_ptr<osg::Material> matRamp = new osg::Material;
    matRamp->setAmbient (osg::Material::FRONT_AND_BACK, osg::Vec4(0., 0., 0., 1.0));
    matRamp->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(0.7, 0.8, 0.0, 1.0));
    matRamp->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 1.0));
    matRamp->setShininess(osg::Material::FRONT_AND_BACK, 64);

    // Plane 1
    btosgPlane *myRamp;
    myRamp = new btosgPlane();
    myRamp->setRotation(osg::Quat(-0.*osg::PI/8.,osg::Vec3(1.,0.,0.)));
    myRamp->setPosition(0.,-5.,0.);
    myRamp->setName("Ramp1");
    myRamp->body->setFriction(100.);
    myRamp->setMaterial(matRamp);
    myWorld.addObject( myRamp );

    // Plane 2
    myRamp = new btosgPlane();
    myRamp->setRotation(osg::Quat(osg::PI/8.,osg::Vec3(1.,0.,0.)));
    myRamp->setPosition(0.,0.,0.);
    myRamp->setName("Ramp2");
    myRamp->body->setFriction(100.);
    myRamp->setMaterial(matRamp);
    myWorld.addObject( myRamp );

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
    osg::Vec3d eye(osg::Vec3(-15., 0., -5.)+up*20.);
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
        myWorld.stepSimulation(frame_time,10);

        viewer.frame();
        timenow = myTimer.time_s();
        frame_time = timenow - last_time;
        last_time = timenow;
    }
}

