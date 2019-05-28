/*
	btosg.cpp
	Miguel Leitao, 2016
*/

#include "btosg.h"
#include <osg/Material>
#include <osg/Texture2D>

#define _DEBUG_ (0)

btVector3 quat2Euler_orignal(const btQuaternion& q)
{
    // Implementation not tested.

    // roll (x-axis rotation)
    double sinr = +2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    double roll = atan2(sinr, cosr);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
    double pitch;
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2., sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny = +2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    double yaw = atan2(siny, cosy);
    return btVector3(yaw,pitch,roll);
}

btVector3 quat2Euler(const btQuaternion& q) {
    double cx,sx;
    double cy,sy;
    double cz,sz;
    double qW = q.getW();
    double qX = q.getX();
    double qY = q.getY();
    double qZ = q.getZ(); 
    double eX,eY,eZ;
    /*
    double matrix[3][3];
    matrix[0][0] = 1.0f - 2.0f * (qY * qY + qZ * qZ);
    matrix[0][1] = (2.0f * qX * qY) - (2.0f * qW * qZ);
    matrix[0][2] = 2.*qX*qZ + 2.*qY*qW;
    matrix[1][0] = 2.0f * (qX * qY + qW * qZ);
    matrix[1][1] = 1.0f - (2.0f * qX * qX) - (2.0f * qZ * qZ);
    matrix[1][2] = 2. * qY * qZ - 2. *qX*qW;
    matrix[2][0] = 2.0f * (qX * qZ - qW * qY);
    matrix[2][1] = 2.0f * (qY * qZ + qW * qX);
    matrix[2][2] = 1.0f - 2.0f * (qX * qX - qY * qY);
    */
    sy = -2. * (qX * qZ - qW * qY); //-matrix[2][0];
    cy = sqrt(1 - (sy * sy));
    eY = atan2(sy,cy);

    if (sy != 1. && sy != -1.) {
        cx = (1. - 2. * (qX * qX - qY * qY)) / cy;
        sx = 2.0f * (qY * qZ + qW * qX) / cy;
        eX = atan2(sx,cx) ;  

        cz = (1. - 2. * (qY * qY + qZ * qZ) ) / cy;
        sz = 2. * (qX * qY + qW * qZ)/ cy;
        eZ = atan2(sz,cz) ;  
    }
    else {   // Gimbal lock
        cx = (1. - (2. * qX * qX) - (2. * qZ * qZ)); //matrix[1][1];
        sx = -2.*qY*qZ + 2.*qX*qW; //-matrix[1][2];
        eX = atan2(sx,cx) ;   

        cz = 1.;
        sz = 0.;
        eZ = atan2(sz,cz) ;   
    }   
    return btVector3(eZ,eX,eY);
}



btosgVec3 btosgQuat::toEuler()
{
    return quat2Euler(*this);
}


btosgWorld::~btosgWorld() {
    delete dynamic; // ????
    delete solver;
    delete dispatcher;
    delete collisionConfiguration;
    delete broadphase;
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
    /// Registers the object obj into a simulation world.
    objects.push_front(obj);
    if ( obj->body )  dynamic->addRigidBody(obj->body);
    else if ( _DEBUG_ ) fprintf(stderr,"Adding object without rigid body\n");
    //printf("adding object to World\n");
    if ( obj->model ) scene->addChild(obj->model);
    else if ( _DEBUG_ ) fprintf(stderr,"Adding object without visual model\n");

};

void btosgWorld::stepSimulation(btScalar timeStep, int maxSubSteps) {
    /// Performs a simulation step.
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
    /// Reset all registered objects.
    for ( auto it = objects.begin(); it != objects.end(); ++it ) {
        btosgObject *obj = *it;
        obj->reset();
    }
};

void btosgObject::loadObjectModel(char const *fname) {
    /// Loads an object model from a Wavefront OBJ file.
    /// Loadded model is used to define both the collision and graphical shapes.
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
    /// Sets the object texture from a loaded image file.
    osg::StateSet* stateset = new osg::StateSet();

    osg::ref_ptr<osg::Image> image = osgDB::readRefImageFile( fname );
    if (image) {
        osg::Texture2D* texture = new osg::Texture2D;
        texture->setImage(image);
        texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
        stateset->setTextureAttributeAndModes(0,texture, osg::StateAttribute::ON);
    }
    stateset->setMode(GL_LIGHTING, osg::StateAttribute::ON);

    model->setStateSet( stateset );
}
