/*
	bowling.cpp 
	Miguel Leitao, ISEP, 2008
*/

///usr/include/bullet/BulletDynamics/Vehicle/btRaycastVehicle.h
#include <osgViewer/Viewer> 
#include <osg/Material>

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <BulletDynamics/Vehicle/btRaycastVehicle.h>

#include "btosg.h"


#define _DEBUG_ (1)



int ResetFlag=0;

// Create World
btosgWorld myWorld;

btosgBox *myBox;



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
			case(osgGA::GUIEventAdapter::KEYUP):
				switch ( ea.getKey() ) {
					case 'S':
						std::cout << "tecla S" << std::endl;
						return false;
                                        case 'f':
                                                std::cout << "adding force" << std::endl;
                                                myBox->body->activate(true);
                                                myBox->body->applyCentralImpulse(btVector3(100.,0.,0.));
                                                return false;
                                        case 'F':
                                                std::cout << "adding force" << std::endl;
                                                myBox->body->activate(true);
                                                myBox->body->applyCentralImpulse(btVector3(-200.,0.,0.));
                                                return false;
                                        case 'R':
					case 'r':
						ResetFlag = 1;
						std::cout << "tecla R" << std::endl;
						break;
						
				}
			case(osgGA::GUIEventAdapter::MOVE):
				std::cout << "mouse move" << ea.getX()<< " " << ea.getY()<< std::endl;
				return false;
			default:
				return false;
		}
	}
};



class btosgWheel : public btosgCylinder {
    public:
        btosgWheel(btVector3 pos, double ang) : btosgCylinder(0.4, 0.2) {
            
            setPosition(pos);
            setRotation(osg::Quat(ang,osg::Vec3(1.,0.,0.)));
            setTexture("beachball.png");
            body->setFriction(100.);
        }
        virtual void update() {
        if (body) {
            btTransform wTrans;
            body->getMotionState()->getWorldTransform(wTrans);
            
            if ( model ) {
                model->setAttitude(bt2osg_Quat(wTrans.getRotation()));
                model->setPosition(bt2osg_Vec3(wTrans.getOrigin()));
            }
            //logPosition();
        }
    }
};


class btosgVehicle: public btosgObject {
    public:
	float dx,dy,dz;
	btosgVehicle(osg::Vec3 dim = osg::Vec3(1.6,4,1.), double m=800. ) {
            dx = dim[0];
            dy = dim[1];
            dz = dim[2];
            osg::Geode *geo = new osg::Geode();
            if ( geo ) {
                osg::Shape *sp = new osg::Box( osg::Vec3(0.,0.,0.), dim[0], dim[1], dim[2] );
                if ( sp) {
                    osg::ShapeDrawable *sd = new osg::ShapeDrawable(sp);
                    if ( sd ) 
                        geo->addDrawable(sd);
                    else fprintf(stderr,"Error creating osg::Shape\n");
                } else fprintf(stderr,"Error creating osg::Shape\n");
            } else fprintf(stderr,"Error creating Geode\n");
            if (  !model)	model = new osg::PositionAttitudeTransform;
            model->addChild(geo);
            model->setNodeMask(CastsShadowTraversalMask);
            mass = m;
            shape = new btBoxShape( osg2bt_Vec3(dim/2.) );
            if ( !shape ) fprintf(stderr,"Error creating btShape\n");
            
            createRigidBody();
            
            btDefaultVehicleRaycaster *rayCaster = new btDefaultVehicleRaycaster(myWorld.dynamic);
            
            btRaycastVehicle::btVehicleTuning tuning;
            btRaycastVehicle *vehicle = new btRaycastVehicle(tuning, body, rayCaster);
    
            
            
            
            printf("vehicle body created\n");
            
            float connectionHeight = 1.2f;
            bool isFrontWheel = true;
            int rightIndex = 0;
            int upIndex = 1;
            int forwardIndex = 2;
            vehicle->setCoordinateSystem(rightIndex, upIndex, forwardIndex); // 0, 1, 2

            // add wheels
            
                    float wheelWidth = 0.2;
        float wheelRadius = 0.5;
        
        btVector3 wheelDirectionCS0(0,-1,0);
        
        btVector3 wheelAxleCS(-1,0,0); 
        
            btVector3 connectionPointCS0;                                
   float suspensionRestLength = 0.6f;//0.6
   
           btCollisionShape *wheelShape = new btCylinderShapeX(btVector3(wheelWidth, wheelRadius, wheelRadius));
            printf("creating wheels\n");

         
            // front left
            connectionPointCS0 = btVector3(3.-(0.3*wheelWidth), 3*1-wheelRadius, connectionHeight);
            printf("definiu connection point 1\n");
            vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, isFrontWheel);
            printf("adicionou roda 1\n");
            
            // front right
            connectionPointCS0 = btVector3(-3+(0.3*wheelWidth),  3*1-wheelRadius, connectionHeight);
            vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, isFrontWheel);
            
            isFrontWheel = false;
            // rear right
            connectionPointCS0 = btVector3(-3+(0.3*wheelWidth),  -3*1+wheelRadius, connectionHeight);
            vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, isFrontWheel);
            // rear left
            connectionPointCS0 = btVector3(3-(0.3*wheelWidth),  -3*1+wheelRadius, connectionHeight);
            vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, isFrontWheel);
            
            printf("wheels created\n");

        
        for (int i = 0; i < vehicle->getNumWheels(); i++)
        {
            btWheelInfo& wheel = vehicle->getWheelInfo(i);
            wheel.m_suspensionStiffness = 14.0;
            wheel.m_wheelsDampingRelaxation = 0.2;
            wheel.m_wheelsDampingCompression = 0.2;
            wheel.m_frictionSlip = 1000.;
            wheel.m_rollInfluence = 0.01;
        }
            printf("vehicle created\n");
            
            vehicle->setSteeringValue(0.,0);
            vehicle->setSteeringValue(0.,1);
	}
  
    
    /*
    
    wheelShape = new btCylinderShapeX(btVector3(wheelWidth, wheelRadius, wheelRadius));
    {
        vehicleRayCaster = new btDefaultVehicleRaycaster(dynamicsWorld);
        vehicle = new btRaycastVehicle(vehicleTuning, vehicleRigidBody, vehicleRayCaster);

        // never deactivate vehicle
        vehicleRigidBody->setActivationState(DISABLE_DEACTIVATION);
        dynamicsWorld->addVehicle(vehicle);

        float connectionHeight = 1.2f;
        bool isFrontWheel = true;

        rightIndex = , upIndex, forwardIndex)
        vehicle->setCoordinateSystem(rightIndex, upIndex, forwardIndex); // 0, 1, 2

        // add wheels
        // front left
        btVector3 connectionPointCS0;
        connectionPointCS0 = byVector3(CUBE_HALF_EXTENT-(0.3*wheelWidth), connectionHeight, 2*CUBE_HALF_EXTENT-wheelRadius);
        vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, vehicleTuning, isFrontWheel);
        // front right
        connectionPointCS0 = btVector3(-CUBE_HALF_EXTENT+(0.3*wheelWidth), connectionHeight, 2*CUBE_HALF_EXTENT-wheelRadius);
        vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, vehicleTuning, isFrontWheel);
        isFrontWheel = false;
        // rear right
        connectionPointCS0 = btVector3(-CUBE_HALF_EXTENT+(0.3*wheelWidth), connectionHeight, -2*CUBE_HALF_EXTENT+wheelRadius);
        vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, vehicleTuning, isFrontWheel);
        // rear left
        connectionPointCS0 = btVector3(CUBE_HALF_EXTENT-(0.3*wheelWidth), connectionHeight, -2*CUBE_HALF_EXTENT+wheelRadius);
        vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, vehicleTuning, isFrontWheel);

        for (int i = 0; i < vehicle->getNumWheels(); i++)
        {
            btWheelInfo& wheel = vehicle->getWheelInfo(i);
            wheel.m_suspensionStiffness = suspensionStiffness;
            wheel.m_wheelsDampingRelaxation = suspensionDamping;
            wheel.m_wheelsDampingCompression = suspensionCompression;
            wheel.m_frictionSlip = wheelFriction;
            wheel.m_rollInfluence = rollInfluence;
        }


    }

    ///////////////////////////////////////////////////////////////////////

    // Orientation and Position of Falling body
    fallMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(-1, 5, 0)));
    btScalar mass = 1;
    btVector3 fallInertia(0, 0, 0);
    fallShape->calculateLocalInertia(mass, fallInertia);
    btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState, fallShape, fallInertia);
    fallRigidBody = new btRigidBody(fallRigidBodyCI);
    dynamicsWorld->addRigidBody(fallRigidBody);

        }
        */
};


int main()
{
	double z_bola = 10.;
	double v_bola = 0.;
	osg::Matrix myMatrix;
        
    myWorld.dynamic->setGravity(btVector3(0., -9.8, 0.));

     // Box
    btVector3 posBody = btVector3(-2.,0.,3.);
    
    btosgVehicle *myVehicle;
    myVehicle = new btosgVehicle();
    myVehicle->setPosition(btVector3(2.,0.,2.));
    myVehicle->setName("Vehicle");
    myWorld.addObject( myVehicle );
    

    myBox = new btosgBox(osg::Vec3(4.,1.6,1.),800.);
    myBox->setPosition(posBody);
    myBox->setName("Body");
    myWorld.addObject( myBox );
        
        
    // Rodas
    osg::Material* matCylinder = new osg::Material;
    matCylinder->setAmbient (osg::Material::FRONT_AND_BACK, osg::Vec4(0.0, 0.,  0.,  1.0));
    matCylinder->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(0.6, 0.4, 0.1, 1.0));
    matCylinder->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0.,  0.,  0.,  1.0));
    matCylinder->setShininess(osg::Material::FRONT_AND_BACK, 64);
 /*   
    btosgCylinder *wheelBR = new btosgCylinder(0.4,0.2);
    wheelBR->setPosition(posBody+btVector3(-1.5,-1.,1.));
    wheelBR->setRotation(osg::Quat(M_PI/2.,osg::Vec3(1.,0.,0.)));
    wheelBR->setName("CylinderBR");
    wheelBR->setTexture("beachball.png");
    myWorld.addObject( wheelBR );
    
    btosgCylinder *wheelBL = new btosgCylinder(0.4,0.2);
    wheelBL->setPosition(posBody+btVector3(-1.5,1.,1.));
    wheelBL->setRotation(osg::Quat(M_PI/2.,osg::Vec3(1.,0.,0.)));
    wheelBL->setName("CylinderBR");
    wheelBL->setTexture("beachball.png");
    myWorld.addObject( wheelBL );
    
    btosgCylinder *wheelFR = new btosgCylinder(0.4,0.2);
    wheelFR->setPosition(posBody+btVector3(.5,-1.,1.));
    wheelFR->setRotation(osg::Quat(M_PI/2.,osg::Vec3(1.,0.,0.)));
    wheelFR->setName("CylinderFR");
    wheelFR->setTexture("beachball.png");
    myWorld.addObject( wheelFR );
    
    btosgCylinder *wheelFL = new btosgCylinder(0.4,0.2);
    wheelFL->setPosition(posBody+btVector3(.5,1.,1.));
    wheelFL->setRotation(osg::Quat(M_PI/2.,osg::Vec3(1.,0.,0.)));
    wheelFL->setName("CylinderFR");
    wheelFL->setTexture("beachball.png");
    myWorld.addObject( wheelFL );
    */
 
    btosgWheel *wheelBR = new btosgWheel(posBody+btVector3(-1.5,-1.,1.), -M_PI/2.);
    wheelBR->setName("WheelBR");
    myWorld.addObject( wheelBR );
    
    btosgWheel *wheelBL = new btosgWheel(posBody+btVector3(-1.5,1.,1.), M_PI/2.);
    wheelBL->setName("WheelBR");
    myWorld.addObject( wheelBL );
    
    btosgWheel *wheelFR = new btosgWheel(posBody+btVector3(1.5,-1.,1.), -M_PI/2.);
    wheelFR->setName("WheelFR");
    myWorld.addObject( wheelFR );
    
    btosgWheel *wheelFL = new btosgWheel(posBody+btVector3(1.5,1.,1.), M_PI/2.);
    wheelFL->setName("WheelFR");
    myWorld.addObject( wheelFL );
    wheelBL->model->getOrCreateStateSet()->
        setAttributeAndModes(matCylinder, osg::StateAttribute::ON);
    wheelBR->model->getOrCreateStateSet()->
        setAttributeAndModes(matCylinder, osg::StateAttribute::ON);   
    wheelFL->model->getOrCreateStateSet()->
        setAttributeAndModes(matCylinder, osg::StateAttribute::ON);
    wheelFR->model->getOrCreateStateSet()->
        setAttributeAndModes(matCylinder, osg::StateAttribute::ON); 
    
    btQuaternion ori;
    btVector3 pos;
    
    // Left side
    ori = btQuaternion( btVector3(0.,1.,0.), M_PI/2.);
    pos = btVector3(-1.5,1.,0.);
    btTransform *sliderBLb = new btTransform(ori,pos);
    
    ori = btQuaternion( btVector3(1.,0.,0.), -M_PI/2.);
    pos = btVector3(0.,0.,0.);
    btTransform *sliderLw = new btTransform(ori,pos);
    
    btSliderConstraint *suspBL = new btSliderConstraint(*(myBox->body), *(wheelBL->body),
        *sliderBLb, *sliderLw, true);

    ori = btQuaternion( btVector3(0.,1.,0.), M_PI/2.);
    pos = btVector3(1.5,1.,0.);
    btTransform *sliderFLb = new btTransform(ori,pos);
    
    btSliderConstraint *suspFL = new btSliderConstraint(*(myBox->body), *(wheelFL->body),
        *sliderFLb, *sliderLw, true);
    
    // Right side
    ori = btQuaternion( btVector3(0.,1.,0.), M_PI/2.);
    pos = btVector3(-1.5,-1.,0.);
    btTransform *sliderBRb = new btTransform(ori,pos);
    
    ori = btQuaternion( btVector3(1.,0.,0.), M_PI/2.);
    pos = btVector3(0.,0.,0.);
    btTransform *sliderRw = new btTransform(ori,pos);
    
    btSliderConstraint *suspBR = new btSliderConstraint(*(myBox->body), *(wheelBR->body),
        *sliderBRb, *sliderRw, true);

    ori = btQuaternion( btVector3(0.,1.,0.), M_PI/2.);
    pos = btVector3(1.5,-1.,0.);
    btTransform *sliderFRb = new btTransform(ori,pos);
    
    btSliderConstraint *suspFR = new btSliderConstraint(*(myBox->body), *(wheelFR->body),
        *sliderFRb, *sliderRw, true);
    
suspBL->setLowerAngLimit(0.0f);
suspBL->setUpperAngLimit(0.0f);
suspBR->setLowerAngLimit(0.0f);
suspBR->setUpperAngLimit(0.0f);
suspFL->setLowerAngLimit(0.0f);
suspFL->setUpperAngLimit(0.0f);
suspFR->setLowerAngLimit(0.0f);
suspFR->setUpperAngLimit(0.0f);

float minSuspZ = 0.7f;
float maxSuspZ = 0.9f;
suspBL->setLowerLinLimit(minSuspZ);
suspBL->setUpperLinLimit(maxSuspZ);
suspBR->setLowerLinLimit(minSuspZ);
suspBR->setUpperLinLimit(maxSuspZ);
suspFL->setLowerLinLimit(minSuspZ);
suspFL->setUpperLinLimit(maxSuspZ);
suspFR->setLowerLinLimit(minSuspZ);
suspFR->setUpperLinLimit(maxSuspZ);

//suspFR->setDampoing
    // Susps
    myWorld.dynamic->addConstraint(suspBL);
    myWorld.dynamic->addConstraint(suspFL);
    
    myWorld.dynamic->addConstraint(suspBR);
    myWorld.dynamic->addConstraint(suspFR);
    
    // Plane 1
    btosgPlane *myPlane = new btosgPlane();
    myPlane->setName("Plane");
    myWorld.addObject( myPlane );

    osg::Material* mat = new osg::Material;
    mat->setAmbient (osg::Material::FRONT_AND_BACK, osg::Vec4(0.01, 0.01, 0.01, 0.));
    mat->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(0.1, 0.5, 0.1, 1.0));
    mat->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(1.0, 1.0, 1.0, 1.0));
    mat->setShininess(osg::Material::FRONT_AND_BACK, 64);
    myPlane->model->getOrCreateStateSet()->
        setAttributeAndModes(mat, osg::StateAttribute::ON);
    

    // Plane 2
    btosgPlane *myRamp = new btosgPlane();
    myRamp->setRotation(osg::Quat(-osg::PI/2,osg::Vec3(1.,0.,0.)));
    myRamp->setPosition(0.,-10.,0.);
    myWorld.addObject( myRamp );
    myRamp->setName("Ramp");
    myRamp->body->setFriction(100.);
    osg::Material* matRamp = new osg::Material;
    matRamp->setAmbient (osg::Material::FRONT_AND_BACK, osg::Vec4(0., 0., 0., 1.0));
    matRamp->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(0.4, 0.1, 0.1, 1.0));
    matRamp->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 1.0));
    matRamp->setShininess(osg::Material::FRONT_AND_BACK, 64);
    myRamp->model->getOrCreateStateSet()->
        setAttributeAndModes(matRamp, osg::StateAttribute::ON);
 

        
        // Creating the viewer
	osgViewer::Viewer viewer ;

	// Setup camera
	osg::Matrix matrix;
	matrix.makeLookAt( osg::Vec3(0.,-8.,5.), osg::Vec3(0.,0.,1.), osg::Vec3(0.,0.,1.) );
	viewer.getCamera()->setViewMatrix(matrix);

	// add the Event handler
	viewer.addEventHandler(new EventHandler());

        // Light
        osg::ref_ptr<osg::LightSource> ls = new osg::LightSource;
        ls->getLight()->setPosition(osg::Vec4(2.5,-10, 20, 1)); // make 4th coord 1 for point
        ls->getLight()->setAmbient(osg::Vec4(0.1, 0.1, 0.1, 1.0));
        ls->getLight()->setDiffuse(osg::Vec4(1.0, 1.0, 1.0, 1.0));
        ls->getLight()->setSpecular(osg::Vec4(0.2, 0.2, 0.2, 1.0));
        myWorld.scene->addChild(ls.get());

        viewer.setSceneData( myWorld.scene );
        
        osgGA::TrackballManipulator *manipulator = new osgGA::TrackballManipulator;
        viewer.setCameraManipulator( manipulator );

        // Set the desired home coordinates for the manipulator
        osg::Vec3d eye(0.0, -5.0, 10.0);
        osg::Vec3d center(1.0, 1.0, 0.0);
        osg::Vec3d up(0.0, 1.0, 0.0);

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
	double frame_time = 0.;
    
    
    suspBL->enableFeedback(1);
    
    
        while( !viewer.done() )
	{
	 	myWorld.stepSimulation(frame_time,10);

	  	viewer.frame();
	  	timenow = myTimer.time_s();
	  	frame_time = timenow - last_time;
	  	last_time = timenow;
        
        float impulse = suspBL->getAppliedImpulse();
        printf("  imp %f   force=%f\n",impulse,impulse/frame_time);
        
        
		if (ResetFlag>0) {
		    //Reset(); 
                    myWorld.reset();
		    ResetFlag = 0;
		}
	}
}
