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

#include <osg/Material>
#include <osg/Texture2D>

#define _DEBUG_ (1)



int ResetFlag=0;
double frame_time = 0.;

// Create World
btosgWorld myWorld;

btosgBox *myBox;



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
        btRaycastVehicle *vehicle;
        osg::PositionAttitudeTransform *wheel[4];
        double wheelRotation[4];
        btosgVehicle(osg::Vec3 dim = osg::Vec3(2.,0.4,4.), double m=800. ) {
            dx = dim[0];
            dy = dim[1];
            dz = dim[2];
            osg::Geode *geo = new osg::Geode();
            if ( geo ) {
                // Box to visualize the chassis
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
            
            // Center-of-gravity is shifted by shift ????
            btTransform shift(btQuaternion::getIdentity(), btVector3(0.f, 0.f, 0.f));
            // Box for collisions and center-of-gravity definition
            btCollisionShape* boxShape = new btBoxShape(osg2bt_Vec3(dim/2.));
            btCompoundShape* chassisShape = new btCompoundShape();
            chassisShape->addChildShape(shift, boxShape);
            shape = chassisShape;
            if ( !shape ) fprintf(stderr,"Error creating btShape\n");
            
            createRigidBody();
            
            btDefaultVehicleRaycaster *rayCaster = new btDefaultVehicleRaycaster(myWorld.dynamic);
            btRaycastVehicle::btVehicleTuning tuning;
            vehicle = new btRaycastVehicle(tuning, body, rayCaster);
    
            body->setActivationState(DISABLE_DEACTIVATION);
            myWorld.dynamic->addVehicle(this->vehicle);
            
            //Adds the wheels to the vehicle
            btVector3 halfExtents = btVector3(dim[0]/2.,dim[1]/2.,dim[2]/2.);
            addWheels(&halfExtents, this->vehicle, tuning);
            
            printf("vehicle body created\n");
            
            // vehicle->setCoordinateSystem(rightIndex, upIndex, forwardIndex); // 0, 1, 2
            vehicle->setCoordinateSystem( 0, 1, 2 );
            return;

            printf("vehicle created\n");
            
            vehicle->setSteeringValue(0.,0);
            vehicle->setSteeringValue(0.,1);
	}
    
    /*
    wheelShape = new btCylinderShapeX(btVector3(wheelWidth, wheelRadius, wheelRadius));
        */
    
    void addWheels(
        btVector3* halfExtents,
        btRaycastVehicle* vehicle,
        btRaycastVehicle::btVehicleTuning tuning)
    {
        // The direction of the raycast, the btRaycastVehicle uses raycasts 
        // to sense the ground under the wheels
        btVector3 wheelDirectionCS0(0, -1, 0);

        //The axis which the wheel rotates arround
        btVector3 wheelAxleCS(-1, 0, 0);
        // center-of mass height if mass=0
        // height = suspensionRestLength-mass.g/m_suspensionStiffness
        btScalar suspensionRestLength(0.80);
        btScalar wheelWidth(0.2);
        btScalar wheelRadius(0.3);
        //The height where the wheels are connected to the chassis
        btScalar connectionHeight(0.4);

        //All the wheel configuration assumes the vehicle is centered at the origin and a right handed coordinate system is used
        btVector3 wheelConnectionPoint(halfExtents->x() + wheelWidth/2., connectionHeight, halfExtents->z() - wheelRadius);
        printf( "halfExtents %f %f %f\n",halfExtents->x(),halfExtents->y(),halfExtents->z()); 

        //Adds the front wheels
        vehicle->addWheel(wheelConnectionPoint, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, true);

        vehicle->addWheel(wheelConnectionPoint*btVector3(-1, 1, 1), wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, true);

        //Adds the rear wheels
        vehicle->addWheel(wheelConnectionPoint*btVector3(1, 1, -1), wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, false);

        vehicle->addWheel(wheelConnectionPoint*btVector3(-1, 1, -1), wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, false);

        // Create one graphics wheel
        osg::Geode *geo = new osg::Geode;
        osg::Shape *sp = new osg::Cylinder( osg::Vec3(0.,0.,0.), wheelRadius, wheelWidth);
        if ( sp) {
            osg::ShapeDrawable *sd = new osg::ShapeDrawable(sp);
            if ( sd ) {
                geo->addDrawable(sd);
                // ---------------------------------------
                // Set up a StateSet to texture the body
                // ---------------------------------------
                osg::StateSet* stateset = new osg::StateSet();
	
                osg::ref_ptr<osg::Image> image = osgDB::readRefImageFile( "beachball.png" );
                if (image)
                {
                    osg::Texture2D* texture = new osg::Texture2D;
                    texture->setImage(image);
                    texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
                    stateset->setTextureAttributeAndModes(0,texture, osg::StateAttribute::ON);
                }
                stateset->setMode(GL_LIGHTING, osg::StateAttribute::ON);

                model->setStateSet( stateset ); 
            }
            else fprintf(stderr,"Error creating osg::Shape\n");
        } else fprintf(stderr,"Error creating osg::Shape\n");
        
        osg::PositionAttitudeTransform *gen_wheel = new  osg::PositionAttitudeTransform;
        gen_wheel->setPosition(osg::Vec3(0.,0., 0.));
        gen_wheel->setAttitude(osg::Quat(-osg::PI/2.,osg::Vec3(0.,1.,0.)));
        gen_wheel->addChild(geo);
        
   
                
                //wheel[0]->setTexture("beachball.png");
      //  } else fprintf(stderr,"Error creating Geode\n");
            

            printf( "num wheels %d\n",vehicle->getNumWheels()); 
        // Configures each wheel of our vehicle, setting its friction, damping compression, etc.
        // For more details on what each parameter does, refer to the docs
        for (int i = 0; i < vehicle->getNumWheels(); i++)
        {
            
            btWheelInfo& iWheel = vehicle->getWheelInfo(i);
            /*
             * iWheel.m_suspensionStiffness = 14.0;
             * iWheel.m_wheelsDampingRelaxation = 0.2;
             * iWheel.m_wheelsDampingCompression = 0.2;
             * iWheel.m_frictionSlip = 1000.;
             * iWheel.m_rollInfluence = 0.01;
             */
            // Affects rest height
            iWheel.m_suspensionStiffness = 15.;
            iWheel.m_wheelsDampingCompression = 0.3 * 2 * btSqrt(iWheel.m_suspensionStiffness); // 0.8;
            iWheel.m_wheelsDampingRelaxation =  0.5 * 2 * btSqrt(iWheel.m_suspensionStiffness); // 1;
            //Larger friction slips will result in better handling
            iWheel.m_frictionSlip = 1.5;
            iWheel.m_rollInfluence = 1;
            
            wheel[i] = new  osg::PositionAttitudeTransform;
            if ( wheel[i] ) {
                wheel[i]->addChild(gen_wheel);
                //wheel[i]->setPosition(osg::Vec3(bt2osg_Vec3(*halfExtents)));
                osg::Vec3 iPos = bt2osg_Vec3(iWheel.m_chassisConnectionPointCS);
                wheel[i]->setPosition(iPos);
                //printf("  roda %d, %f %f %f\n",i,iPos[0],iPos[1],iPos[2]);
                model->addChild( wheel[i] );
            }
        }
	
}
    void printInfo() {
        btosgPrint("Center of Mass", vehicle->getRigidBody()->getCenterOfMassPosition());
        btosgPrint("Center of Mass", body->getCenterOfMassPosition());
        printf("Mass: %f\n", mass);
        for( int i=0 ; i<vehicle->getNumWheels(); i++)
        {
            btWheelInfo& iWheel = vehicle->getWheelInfo(i);
            printf("Wheel %d\n",i);
            btosgPrint("  chassisConnectionPoint", iWheel.m_chassisConnectionPointCS);
            btosgPrint("  WheelDirection", iWheel.m_wheelDirectionCS);
            btosgPrint("  WheelAxle", iWheel.m_wheelAxleCS);
            /*
            btScalar        m_suspensionRestLength1;    //const
            btScalar        m_maxSuspensionTravelCm;
            btScalar        getSuspensionRestLength()   const;
            btScalar        m_wheelsRadius;             //const
            btScalar        m_suspensionStiffness;      //const
            btScalar        m_wheelsDampingCompression; //const
            btScalar        m_wheelsDampingRelaxation;  //const
            */
        }
    }
    
    
    void logPosition() {
         if (body) {
            btTransform wTrans;
            body->getMotionState()->getWorldTransform(wTrans);
            btVector3 pos;
            pos = wTrans.getOrigin();
            if ( name )
                        std::cout << "Object   " << name << " position " << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
            else
                        std::cout << "Object _NO_NAME_ position " << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
            return;
            for( int i=0 ; i<4 ; i++ ) {
                btWheelInfo& iWheel = vehicle->getWheelInfo(i);
                std::cout << "  whell " << i << ", contact: " << iWheel.m_raycastInfo.m_isInContact ;
                printf("  rotation %f\n", iWheel.m_rotation);
                
            }
         }
    }
   virtual void update()
    {
        // Not required.
        // Vehicle dynamics is updated in stepSimulation();
        // updateVehicle requires frame_time that may not be available.
        vehicle->updateVehicle(frame_time);
               
        // Visual update
        // Standard btosgObject::update() can be used.
        btosgObject::update();
        logPosition();
        
        // Update Wheels
        for (int i = 0; i < vehicle->getNumWheels(); i++)
        {
            // btRaycastVehicle::updateVehicle() calls updateWheelTransform for every wheel.
            // Do not call vehicle->updateWheelTransform() here if btRaycastVehicle::updateVehicle() is called elsewhere.
            //vehicle->updateWheelTransform(i,true);
            btWheelInfo& iWheel = vehicle->getWheelInfo(i);
            if ( wheel[i] ) {
                osg::Vec3 iPos =    bt2osg_Vec3(iWheel.m_chassisConnectionPointCS) -
                                    osg::Vec3(0.,iWheel.m_raycastInfo.m_suspensionLength,0.);
                wheel[i]->setPosition(iPos);
                
                //wheel[i]->setAttitude(osg::Quat(iWheel.m_rotation,osg::Vec3(1.,0.,0.)));
                //wheel[i]->setAttitude(osg::Quat(iWheel.m_steering,osg::Vec3(0.,1.,0.)));
                
                btQuaternion steeringOrn(iWheel.m_wheelDirectionCS,-iWheel.m_steering);
                btMatrix3x3 steeringMat(steeringOrn);

                btQuaternion rotatingOrn(iWheel.m_wheelAxleCS,-iWheel.m_rotation);
                btMatrix3x3 rotatingMat(rotatingOrn);
	
                btMatrix3x3 fullMat = steeringMat*rotatingMat;
                btQuaternion fullQuat;
                fullMat.getRotation(fullQuat);
                wheel[i]->setAttitude(bt2osg_Quat(fullQuat));
                
               // btTransform wTrans = iWheel.m_worldTransform;
                
                //btTransform wTrans = vehicle->getWheelTransformWS(i);
                //wheel[i]->setAttitude(bt2osg_Quat(wTrans.getRotation()));
            }
        }   
        
        
        //printf("printf using btosgVehicle::update()\n");
        /*
        if (body) {
            btTransform wTrans;
            body->getMotionState()->getWorldTransform(wTrans);
            //wTrans = vehicle->getChassisWorldTransform();
            if ( model ) {
                model->setAttitude(bt2osg_Quat(wTrans.getRotation()));
                model->setPosition(bt2osg_Vec3(wTrans.getOrigin()));
            }
        }
        */
    }
    
};


class BlockGreen : public btosgBox {
    public:
        BlockGreen(float x, float y, float z) : btosgBox( osg::Vec3(1.,1.,1.), 100. ) {
            setPosition(btVector3(x,y,z));
            osg::Material* mat = new osg::Material;
            mat->setAmbient (osg::Material::FRONT_AND_BACK, osg::Vec4(0., 0., 0., 1.0));
            mat->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(0.1, 0.4, 0.1, 1.0));
            mat->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 1.0));
            mat->setShininess(osg::Material::FRONT_AND_BACK, 64);
            model->getOrCreateStateSet()->
                setAttributeAndModes(mat, osg::StateAttribute::ON);
        }
       // asBox(BlockGreen) { return this; };
};
class BlockRed : public btosgBox {
    public:
        BlockRed(float x, float y, float z) : btosgBox( osg::Vec3(1.,1.,1.), 10000. ) {
            setPosition(btVector3(x,y,z));
            osg::Material* mat = new osg::Material;
            mat->setAmbient (osg::Material::FRONT_AND_BACK, osg::Vec4(0., 0., 0., 1.0));
            mat->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(0.4, 0.1, 0.1, 1.0));
            mat->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 1.0));
            mat->setShininess(osg::Material::FRONT_AND_BACK, 64);
            model->getOrCreateStateSet()->
                setAttributeAndModes(mat, osg::StateAttribute::ON);
        }
       // asBox(BlockGreen) { return this; };
};


class BlockBlue : public btosgBox {
    public:
        BlockBlue(float x, float y, float z) : btosgBox( osg::Vec3(1.,0.25,1.), 10000. ) {
            setPosition(btVector3(x,y,z));
            osg::Material* mat = new osg::Material;
            mat->setAmbient (osg::Material::FRONT_AND_BACK, osg::Vec4(0., 0., 0., 1.0));
            mat->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(0.1, 0.1, 0.5, 1.0));
            mat->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 1.0));
            mat->setShininess(osg::Material::FRONT_AND_BACK, 64);
            model->getOrCreateStateSet()->
                setAttributeAndModes(mat, osg::StateAttribute::ON);
        }
       // asBox(BlockGreen) { return this; };
};



    btosgVehicle *myVehicle;

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
                        myVehicle->vehicle->applyEngineForce(-1500, 2);
                        myVehicle->vehicle->applyEngineForce(-1500, 3);
                        return false;
                    case osgGA::GUIEventAdapter::KEY_Up:
                        myVehicle->vehicle->applyEngineForce(1500, 2);
                        myVehicle->vehicle->applyEngineForce(1500, 3);
                        return false;
                    case osgGA::GUIEventAdapter::KEY_Left:
                        myVehicle->vehicle->setSteeringValue(btScalar(0.4), 0);
                        myVehicle->vehicle->setSteeringValue(btScalar(0.4), 1);
                        return false;
                    case osgGA::GUIEventAdapter::KEY_Right:
                        myVehicle->vehicle->setSteeringValue(btScalar(-0.4), 0);
                        myVehicle->vehicle->setSteeringValue(btScalar(-0.4), 1);
                        return false;
                    case 'b':
                    case '0':
                    case osgGA::GUIEventAdapter::KEY_Control_R:
                        myVehicle->vehicle->setBrake(10000, 2);
                        myVehicle->vehicle->setBrake(10000, 3);
                        return false;
                }
                break;
	case(osgGA::GUIEventAdapter::KEYUP):
		switch ( ea.getKey() ) {
                    case osgGA::GUIEventAdapter::KEY_Down:
                    case osgGA::GUIEventAdapter::KEY_Up:
                        myVehicle->vehicle->applyEngineForce(0, 2);
                        myVehicle->vehicle->applyEngineForce(0, 3);
                        return false;
                    case osgGA::GUIEventAdapter::KEY_Left:
                    case osgGA::GUIEventAdapter::KEY_Right:
                        myVehicle->vehicle->setSteeringValue(btScalar(0), 0);
                        myVehicle->vehicle->setSteeringValue(btScalar(0), 1);
                        return false;
                    case '0':
                    case 'b':
                    case osgGA::GUIEventAdapter::KEY_Control_R:
                        myVehicle->vehicle->setBrake(5, 2);
                        myVehicle->vehicle->setBrake(5, 3);
                        return false;
                    case 'S':
			std::cout << "tecla S" << std::endl;
			return false;
                    case 'i':
                        myVehicle->printInfo();
                        break;
                    case 'f':
                            std::cout << "adding force" << std::endl;
                            myBox->body->activate(true);
                            myBox->body->applyCentralImpulse(btVector3(100.,0.,0.));
                            return false;
                    case 'F':
                            std::cout << "adding Force" << std::endl;
                            //myBox->body->activate(true);
                            //myBox->body->applyCentralImpulse(btVector3(-200.,0.,0.));
                            
                            myVehicle->vehicle->applyEngineForce(500, 2);
                            myVehicle->vehicle->applyEngineForce(500, 3);
                            
                            int i;
                            for( i=0 ; i<myVehicle->vehicle->getNumWheels() ; i++) {
                                btWheelInfo& iWheel = myVehicle->vehicle->getWheelInfo(i);
                                printf(" wheel %d, radius %f, rotation %f, eforce %f, steer %f\n", i, iWheel.m_wheelsRadius, iWheel.m_rotation, iWheel.m_engineForce,iWheel.m_steering);
                            }
                            
                            // handled = true;
                            return false;
                    case 'R':
					case 'r':
						ResetFlag = 1;
						std::cout << "tecla R" << std::endl;
						break;
						
				}
			case(osgGA::GUIEventAdapter::MOVE):
				std::cout << "mouse move" << ea.getX() << " " << ea.getY() << std::endl;
				return false;
			default:
				return false;
		}
		return true;
	}
};


int main()
{
    osg::Matrix myMatrix;
        
    myWorld.dynamic->setGravity(btVector3(0., -9.8, 0.));

     // Car
    myVehicle = new btosgVehicle();
    myVehicle->setPosition(btVector3(0.,3.,0.));
    myVehicle->setName("Vehicle");
    myWorld.addObject( myVehicle );
    myVehicle->printInfo();
    {
    BlockGreen *myBlock;
    myBlock = new BlockGreen(4.,0.,-4.);
    myWorld.addObject(myBlock);
    myBlock = new BlockGreen(6.,0.,5.);
    myWorld.addObject(myBlock);
    myBlock = new BlockGreen(-7.,0.,5.);
    myWorld.addObject(myBlock);
    myBlock = new BlockGreen(9.,0.,5.);
    myWorld.addObject(myBlock);
    myBlock = new BlockGreen(10.,0.,6.);
    myWorld.addObject(myBlock);
    myBlock = new BlockGreen(-11.,0.,6.);
    myWorld.addObject(myBlock);
    }
    {
    BlockRed *myBlock;
    myBlock = new BlockRed(4.,0.,4.);
    myWorld.addObject(myBlock);
    myBlock = new BlockRed(7.,0.,5.);
    myWorld.addObject(myBlock);
    myBlock = new BlockRed(-8.,0.,5.);
    myWorld.addObject(myBlock);
    myBlock = new BlockRed(9.,0.,-5.);
    myWorld.addObject(myBlock);
    myBlock = new BlockRed(10.,0.,-6.);
    myWorld.addObject(myBlock);
    myBlock = new BlockRed(-12.,0.,6.);
    myWorld.addObject(myBlock);
    }

    {
    BlockBlue *myBlock;
    myBlock = new BlockBlue(4.,0.,-4.);
    myWorld.addObject(myBlock);
    myBlock = new BlockBlue(7.5,0.,6.);
    myWorld.addObject(myBlock);
    myBlock = new BlockBlue(-8.,0.,-5.);
    myWorld.addObject(myBlock);
    myBlock = new BlockBlue(9.,0.,5.);
    myWorld.addObject(myBlock);
    myBlock = new BlockBlue(11.,0.,-6.);
    myWorld.addObject(myBlock);
    myBlock = new BlockBlue(-13.,0.,6.);
    myWorld.addObject(myBlock);
    }
    
/*
    myBox = new btosgBox(osg::Vec3(4.,1.6,1.),800.);
    myBox->setPosition(posBody);
    myBox->setName("Body");
    myWorld.addObject( myBox );
        
  */      
    // Rodas
    osg::Material* matCylinder = new osg::Material;
    matCylinder->setAmbient (osg::Material::FRONT_AND_BACK, osg::Vec4(0.0, 0.,  0.,  1.0));
    matCylinder->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(0.6, 0.4, 0.1, 1.0));
    matCylinder->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0.,  0.,  0.,  1.0));
    matCylinder->setShininess(osg::Material::FRONT_AND_BACK, 64);
  
    
  /*  
    // Plane 1
    btosgPlane *myPlane = new btosgPlane();
    myPlane->setName("Plane");
    myPlane->setPosition(0.,0.,10.);
    myWorld.addObject( myPlane );

    osg::Material* mat = new osg::Material;
    mat->setAmbient (osg::Material::FRONT_AND_BACK, osg::Vec4(0.01, 0.01, 0.01, 0.));
    mat->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(0.1, 0.5, 0.1, 1.0));
    mat->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(1.0, 1.0, 1.0, 1.0));
    mat->setShininess(osg::Material::FRONT_AND_BACK, 64);
    myPlane->model->getOrCreateStateSet()->
        setAttributeAndModes(mat, osg::StateAttribute::ON);
    */

    // Plane 2
    printf("plano2\n");
    btosgPlane *myRamp = new btosgPlane();
    myRamp->setRotation(osg::Quat(-osg::PI/2.,osg::Vec3(1.,0.,0.)));
    myRamp->setPosition(0.,0.,0.);
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
	matrix.makeLookAt( osg::Vec3(0.,8.,5.), osg::Vec3(0.,0.,1.), osg::Vec3(0.,1.,0.) );
	viewer.getCamera()->setViewMatrix(matrix);

	// add the Event handler
	viewer.addEventHandler(new EventHandler());

        // Light
        osg::ref_ptr<osg::LightSource> ls = new osg::LightSource;
        ls->getLight()->setPosition(osg::Vec4(2.5,20, -10, 1)); // make 4th coord 1 for point
        ls->getLight()->setAmbient(osg::Vec4(0.1, 0.1, 0.1, 1.0));
        ls->getLight()->setDiffuse(osg::Vec4(1.0, 1.0, 1.0, 1.0));
        ls->getLight()->setSpecular(osg::Vec4(0.2, 0.2, 0.2, 1.0));
        myWorld.scene->addChild(ls.get());

        viewer.setSceneData( myWorld.scene );
        
        osgGA::TrackballManipulator *manipulator = new osgGA::TrackballManipulator;
        viewer.setCameraManipulator( manipulator );

        // Set the desired home coordinates for the manipulator
        osg::Vec3d eye(0.0, 15.0, -5.0);
        osg::Vec3d center(0.0, 0.0, 1.0);
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
	frame_time = 0.;
    
        //suspBL->enableFeedback(1);

        while( !viewer.done() )
	{
	 	myWorld.stepSimulation(frame_time,10);
                //myVehicle->vehicle->updateVehicle(frame_time);
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
