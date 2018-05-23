/*
	btosgVehicle.h
	Miguel Leitao, 2016
*/

#ifndef BTOSGVEHICLE_H
#define BTOSGVEHICLE_H 1

#include <osg/Material>

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <BulletDynamics/Vehicle/btRaycastVehicle.h>

#include "btosg.h"

#include <osg/Material>
#include <osg/Texture2D>

/*
class btosgWheel : public btosgCylinder {
    public:
        btosgWheel(btVector3 pos, double ang) : btosgCylinder(0.4, 0.2) {
            
            setPosition(pos);
            setRotation(osg::Quat(ang,osg::Vec3(1.,0.,0.)));
            setTexture("wheel.png");
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
*/

class btosgVehicle: public btosgObject {
    private:
        btDefaultVehicleRaycaster *rayCaster;
    public:
        float dx,dy,dz;
        osg::Vec3 dim;
        osg::Vec3 up;
        osg::Vec3 front;
        btRaycastVehicle *vehicle;
        osg::ref_ptr<osg::PositionAttitudeTransform> wheel[4];
        double wheelRotation[4];
        btosgVehicle(btosgWorld *world, osg::Vec3 dimLocal = osg::Vec3(2.,0.4,4.), double m=1000. ) {
            btVector3 grav = world->dynamic->getGravity();
            int grav_axis = grav.minAxis();
            switch ( grav_axis ) {
                case 0:
                    fprintf(stderr,"Gravity direction %d (%f,%f,%f) not supported\n",grav_axis,grav[0],grav[1],grav[2]);
                case 1:
                    up = osg::Vec3(0., 1., 0.);
                    front = osg::Vec3(0., 0., 1.);
                    break;
                case 2:
                    up = osg::Vec3(0., 0., 1.);
                    front = osg::Vec3(0., 1., 0.);
                    break;
            }
            // dimLocal contains (Width,Height,Lenght)
            dim[0] = dimLocal[0];
            dim[1] = dimLocal[1]*up[1] + dimLocal[2]*front[1];
            dim[2] = dimLocal[2]*front[2] + dimLocal[1]*up[2];
            // dim contains (dx,dy,dz)
            dx = dim[0];
            dy = dim[1];
            dz = dim[2];
            osg::ref_ptr<osg::Geode> geo = new osg::Geode();
            if ( geo.valid() ) {
                // Box to visualize the chassis
                osg::ref_ptr<osg::Shape> sp = new osg::Box( osg::Vec3(0.,0.,0.), dx, dy, dz);
                if ( sp) {
                    osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(sp);
                    if ( sd ) 
                        geo->addDrawable(sd);
                    else fprintf(stderr,"Error creating osg::Shape\n");
                } else fprintf(stderr,"Error creating osg::Shape\n");
            } else fprintf(stderr,"Error creating Geode\n");
            if ( !model )	model = new osg::PositionAttitudeTransform;
            model->addChild(geo);
            model->setNodeMask(CastsShadowTraversalMask);
		
	    osg::ref_ptr<osg::Material> mat = new osg::Material;
	    mat->setAmbient (osg::Material::FRONT_AND_BACK, osg::Vec4(0.4, 0.3, 0., 1.0));
	    mat->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(0.8, 0.7, 0.0, 1.0));
	    mat->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0.1, 0.1, 0, 1.0));
	    mat->setShininess(osg::Material::FRONT_AND_BACK, 12);
	    setMaterial(mat);

            setMass(m);
            // Center-of-gravity is shifted by shift ????
            btTransform shift(btQuaternion::getIdentity(), btVector3(0.f, 0.f, 0.f));
            // Box for collisions and center-of-gravity definition
            btCollisionShape* boxShape = new btBoxShape(osg2bt_Vec3(dim/2.));
            btCompoundShape* chassisShape = new btCompoundShape();
            chassisShape->addChildShape(shift, boxShape);
            shape = chassisShape;
            if ( !shape ) fprintf(stderr,"Error creating btShape\n");
            
            createRigidBody();
            
            rayCaster = new btDefaultVehicleRaycaster(world->dynamic);
            btRaycastVehicle::btVehicleTuning tuning;
            vehicle = new btRaycastVehicle(tuning, body, rayCaster);
    
            body->setActivationState(DISABLE_DEACTIVATION);
            world->dynamic->addVehicle(this->vehicle);
            
            //Adds the wheels to the vehicle
            btVector3 halfExtents = btVector3(dim[0]/2.,dim[1]/2.,dim[2]/2.);
            addWheels(&halfExtents, this->vehicle, tuning);
            
            printf("vehicle body created\n");
            
            vehicle->setCoordinateSystem( 0, osg2bt_Vec3(up).maxAxis(), osg2bt_Vec3(front).maxAxis() );
            return;
	}

    void addWheels(
        btVector3* halfExtents,
        btRaycastVehicle* vehicle,
        btRaycastVehicle::btVehicleTuning tuning)
    {
        // The direction of the raycast, the btRaycastVehicle uses raycasts 
        // to sense the ground under the wheels
        btVector3 wheelDirectionCS0(-osg2bt_Vec3(up));

        // The axis which the wheel rotates arround
        btVector3 wheelAxleCS( osg2bt_Vec3(front ^ up) );
        
        // center-of mass height if mass=0
        // height = suspensionRestLength-mass.g/m_suspensionStiffness
        btScalar suspensionRestLength(0.80);
        btScalar wheelWidth(0.2);
        btScalar wheelRadius(0.3);
        // The height where the wheels are connected to the chassis
        double connectionHeight = 0.4;
        double frontAxisPos = 1.25;
        double backAxisPos = 1.25;

        // All the wheel configuration assumes the vehicle is centered at the origin and a right handed coordinate system is used
        btVector3 wheelConnectPoint;
                
        // Adds the front wheels
        wheelConnectPoint = btVector3(  halfExtents->x() + wheelWidth/2.,
                                        connectionHeight*up[1] + frontAxisPos*front[1],
                                        connectionHeight*up[2] + frontAxisPos*front[2]);
        vehicle->addWheel(wheelConnectPoint, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, true);
        vehicle->addWheel(wheelConnectPoint*btVector3(-1, 1, 1), wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, true);

        //Adds the rear wheels
        wheelConnectPoint = btVector3(  halfExtents->x() + wheelWidth/2.,
                                        connectionHeight*up[1] - backAxisPos*front[1],
                                        connectionHeight*up[2] - backAxisPos*front[2]);
        vehicle->addWheel(wheelConnectPoint, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, false);
        vehicle->addWheel(wheelConnectPoint*btVector3(-1, 1, 1), wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, false);

        // Create one graphics wheel
        osg::ref_ptr<osg::Geode> geo = new osg::Geode;
        osg::ref_ptr<osg::Shape> sp;
	sp = new osg::Cylinder( osg::Vec3(0.,0.,0.), wheelRadius, wheelWidth);
        if ( sp) {
            osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(sp);
            if ( sd ) {
                geo->addDrawable(sd);
		// Setup tire material
		
                osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet();
		osg::ref_ptr<osg::Material> mat = new osg::Material;
		mat->setAmbient (osg::Material::FRONT_AND_BACK, osg::Vec4(0., 0., 0., 1.0));
		mat->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(0.1, 0.1, 0.1, 1.0));
		mat->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 1.0));
		mat->setShininess(osg::Material::FRONT_AND_BACK, 64);
		stateset->setAttribute(mat, osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON); 

	        stateset->setTextureAttributeAndModes(0,NULL, osg::StateAttribute::OFF);
                sd->setStateSet( stateset ); 
	    }   else fprintf(stderr,"Error creating osg::Shape\n");
        } else fprintf(stderr,"Error creating osg::Shape\n");
	
	sp = new osg::Cylinder( osg::Vec3(0.,0.,0.), wheelRadius*0.75, wheelWidth+0.005);
        if ( sp) {
            osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(sp);
            if ( sd ) {
                geo->addDrawable(sd);
                osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet();
		// Wheel material
		osg::ref_ptr<osg::Material> mat = new osg::Material;
		mat->setAmbient (osg::Material::FRONT_AND_BACK, osg::Vec4(0.8, 0.8, 0.8, 1.0));
		mat->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(0.4, 0.4, 0.4, 1.0));
		mat->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 1.0));
		mat->setShininess(osg::Material::FRONT_AND_BACK, 64);
		stateset->setAttribute(mat, osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON); 
		// Setup wheel texture
                osg::ref_ptr<osg::Image> image = osgDB::readRefImageFile( "img/wheel.jpg" );
                if (image) {
                    osg::Texture2D* texture = new osg::Texture2D;
                    texture->setImage(image);
                    texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
                    stateset->setTextureAttributeAndModes(0,texture, osg::StateAttribute::ON);
                }
                stateset->setMode(GL_LIGHTING, osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
                sd->setStateSet( stateset ); 
            }   else fprintf(stderr,"Error creating osg::Shape\n");
        } else fprintf(stderr,"Error creating osg::Shape\n");
        
        osg::PositionAttitudeTransform *gen_wheel = new  osg::PositionAttitudeTransform;
        gen_wheel->setPosition(osg::Vec3(0.,0.,0.));
        gen_wheel->setAttitude(osg::Quat(-osg::PI/2.,osg::Vec3(0.,1.,0.)));
        gen_wheel->addChild(geo);
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
       
            // iWheel.m_maxSuspensionForce = 150000.; // 6000
            printf("maxSupForce %f\n", iWheel.m_maxSuspensionForce);
            
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
        btosgPrint("Center of Mass", body->getCenterOfMassPosition());
        btosgPrint("Mass: %f\n", mass);
        for( int i=0 ; i<vehicle->getNumWheels(); i++)
        {
            btWheelInfo& iWheel = vehicle->getWheelInfo(i);
            btosgPrint("Wheel %d\n",i);
            btosgPrint("  ChassisConnectionPoint", iWheel.m_chassisConnectionPointCS);
            btosgPrint("  WheelDirection", iWheel.m_wheelDirectionCS);
            btosgPrint("  WheelAxle", iWheel.m_wheelAxleCS);
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
        // updateVehicle is Not required.
        // Vehicle dynamics is updated in stepSimulation();
        // updateVehicle requires frame_time that may not be available.
        // vehicle->updateVehicle(frame_time);
               
        // Visual update
        // Standard btosgObject::update() can be used.
        btosgObject::update();
        //logPosition();
        
        // Update Wheels
        for (int i = 0; i < vehicle->getNumWheels(); i++)
        {
            // btRaycastVehicle::updateVehicle() calls updateWheelTransform for every wheel.
            // Do not call vehicle->updateWheelTransform() here if btRaycastVehicle::updateVehicle() is called elsewhere.
            //vehicle->updateWheelTransform(i,true);
            btWheelInfo& iWheel = vehicle->getWheelInfo(i);
            if ( wheel[i] ) {
                osg::Vec3 iPos =    bt2osg_Vec3(iWheel.m_chassisConnectionPointCS +
                    iWheel.m_raycastInfo.m_suspensionLength * iWheel.m_wheelDirectionCS);
                wheel[i]->setPosition(iPos);
                
                btQuaternion steeringOrn(iWheel.m_wheelDirectionCS,-iWheel.m_steering);
                btMatrix3x3 steeringMat(steeringOrn);

                btQuaternion rotatingOrn(iWheel.m_wheelAxleCS,-iWheel.m_rotation);
                btMatrix3x3 rotatingMat(rotatingOrn);
	
                btMatrix3x3 fullMat = steeringMat*rotatingMat;
                btQuaternion fullQuat;
                fullMat.getRotation(fullQuat);
                wheel[i]->setAttitude(bt2osg_Quat(fullQuat));
            }
        }   
    }
    
};

#endif // BTOSGVEHICLE_H

