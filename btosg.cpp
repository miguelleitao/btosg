/*
	btosg.cpp 
	Miguel Leitao, 2016
*/


#include "btosg.h"
#include <osg/Material>
#include <osg/Texture2D>

#define _DEBUG_ (1)

osg::Vec3 bt2osg_Vec3(btVector3 bv) {
return osg::Vec3( bv.x(), bv.y(), bv.z() );
}
osg::Vec4 bt2osg_Vec4(btVector4 bv) {
	return osg::Vec4( bv.x(), bv.y(), bv.z(), bv.w() );
}
btVector3 osg2bt_Vec3(osg::Vec3 bv) {
	return btVector3( bv.x(), bv.y(), bv.z() );
}
osg::Quat bt2osg_Quat(btQuaternion bv) {
	return osg::Quat( bv.x(), bv.y(), bv.z(), bv.w() );
}

btQuaternion osg2bt_Quat(osg::Quat bv) {
	return btQuaternion( bv.x(), bv.y(), bv.z(), bv.w() );
}

static void quat2Euler(const btQuaternion& q, double& roll, double& pitch, double& yaw)
{
	// roll (x-axis rotation)
	double sinr = +2.0 * (q.w() * q.x() + q.y() * q.z());
	double cosr = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
	roll = atan2(sinr, cosr);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny = +2.0 * (q.w() * q.z() + q.x() * q.y());
	double cosy = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());  
	yaw = atan2(siny, cosy);
}

        btosgWorld::~btosgWorld() {
            //delete dynamic;
            //delete solver;
            //delete dispatcher;
            //delete collisionConfiguration;
            //delete broadphase;
            for ( auto it = objects.begin(); it != objects.end(); ++it ) {
                btosgObject *obj = *it;
                delete obj;
            }
            /*
            while (!objects.empty())
            {
                objects.pop_front();
            }
*/
            scene->unref();
        }

void btosgWorld::addObject(btosgObject *obj)  {
    objects.push_front(obj);
    if ( obj->body )  dynamic->addRigidBody(obj->body);
		else if ( _DEBUG_ ) fprintf(stderr,"Adding object without rigid body\n");
    //printf("adding object to World\n");
    if ( obj->model ) scene->addChild(obj->model);
		else if ( _DEBUG_ ) fprintf(stderr,"Adding object without visual model\n");
    
};

void btosgWorld::stepSimulation(btScalar timeStep, int maxSubSteps) {
    
        if ( steps==0L ) {
            for ( auto it = objects.begin(); it != objects.end(); ++it ) {
                btosgObject *obj = *it;
                obj->setInitState();
            }      
        }
        dynamic->stepSimulation(timeStep,maxSubSteps);
        
        for ( auto it = objects.begin(); it != objects.end(); ++it ) {
            btosgObject *obj = *it;
            obj->update();
        }
        steps += 1;
};

void btosgWorld::reset() {
        for ( auto it = objects.begin(); it != objects.end(); ++it ) {
            btosgObject *obj = *it;
            obj->reset();
        }
};

void btosgObject::loadObjectModel(char const *fname) {
    if ( ! fname || ! *fname ) {
        fprintf(stderr,"Invalid object model filename\n");
        return;
    }
    osg::Node* loadedModel = osgDB::readNodeFile(fname);
    if ( ! loadedModel ) {
	fprintf(stderr,"Error reading Object model from file '%s'\n", fname);
    }
    if (  !model)	model = new osg::PositionAttitudeTransform;
    osg::PositionAttitudeTransform* obj_rot = new osg::PositionAttitudeTransform;
    obj_rot->setAttitude(osg::Quat(-osg::PI/2.,osg::Vec3(1.,0.,0.)));
    obj_rot->addChild(loadedModel);
    model->addChild(obj_rot);
}


void btosgObject::setTexture(char const *fname)
{
    // ---------------------------------------
    // Set up a StateSet to texture the objects
    // ---------------------------------------
    osg::StateSet* stateset = new osg::StateSet();
	
	    osg::ref_ptr<osg::Image> image = osgDB::readRefImageFile( fname );
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
