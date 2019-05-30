/*
	hpr_test.cpp
	Miguel Leitao, 2019
*/

#include <osgViewer/Viewer>
#include <osg/Material>
#include "btosg.h"

#define _DEBUG_ (0)

double frame_time = 0.;

// Create World
btosgWorld myWorld;

btosgQuat Euler2Quat(btosgVec3 v) {

    //return      osg::Quat::makeRotate(v[0],v[1],v[2]);
    return 	btQuaternion(btVector3(0.,1.,0.),v[2]) * 
    		btQuaternion(btVector3(1.,0.,0.),v[1]) *
		btQuaternion(btVector3(0.,0.,1.),v[0]);
}

int main()
{
    btosgVec3 up(0., 0., 1.);
    myWorld.dynamic->setGravity(btVector3(up)*-9.8);

    btosgExternalObject *myObj1, *myObj2;
    myObj1 = new btosgExternalObject("obj/plane.obj");
    myObj1->setMass(0);
    myObj1->setRotation(btQuaternion(btVector3(1.,0.,0.),osg::PI/2.));
    myObj1->setPosition(-2.,-2.,1.);
    myWorld.addObject( myObj1 );
    myObj2 = new btosgExternalObject("obj/plane.obj");
    myObj2->setMass(0);
    myObj2->setRotation(btQuaternion(btVector3(1.,0.,0.),osg::PI/2.));
    myObj2->setPosition(2.,-2.,1.);
    myWorld.addObject( myObj2 );
    

    // Material for base plans
    osg::ref_ptr<osg::Material> matRamp = new osg::Material;
    matRamp->setAmbient (osg::Material::FRONT_AND_BACK, osg::Vec4(0., 0., 0., 1.0));
    matRamp->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(0.7, 0.8, 0.0, 1.0));
    matRamp->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 1.0));
    matRamp->setShininess(osg::Material::FRONT_AND_BACK, 64);

    // Plane 1
    btosgPlane *myRamp;
    myRamp = new btosgPlane();
    myRamp->setRotation(osg::Quat(0.,osg::Vec3(1.,0.,0.)));
    myRamp->setPosition(0.,-5.,0.);
    myRamp->setName("Ramp1");
    myRamp->setMaterial(matRamp);
    myWorld.addObject( myRamp );

    // Plane 2
    myRamp = new btosgPlane();
    myRamp->setRotation(osg::Quat(osg::PI/2.,osg::Vec3(1.,0.,0.)));
    myRamp->setPosition(0.,0.,0.);
    myRamp->setName("Ramp2");
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
    osg::Vec3d eye(osg::Vec3(0., -15., -5.)+up*20.);
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

	btosgQuat quat = btQuaternion(btVector3(1.,0.,0.),timenow);
        myObj1->setRotation(quat);
	btosgVec3 euler = quat.toEuler();
	printf("quat %f %f %f %f : ",quat[0],quat[1],quat[2],quat[3]);
	printf("euler %f,%f,%f\n", euler[0],euler[1],euler[2]);
        myObj2->setRotation(Euler2Quat(euler));


        viewer.frame();
        timenow = myTimer.time_s();
        frame_time = timenow - last_time;
        last_time = timenow;
    }
}

