/*
	btosg.cpp
	Miguel Leitao, 2016
*/

#include "btosg.h"
#include <osg/Material>
#include <osg/Texture2D>

#define _DEBUG_ (0)


void btosgObject::print() {
    /// Print out objects position.
    std::cout << "Object: " << name << "\n";
    btosgVec3 pos = getPosition();

    std::cout << "  Position: " << pos.x() << ", " << pos.y() << ", " << pos.z() << "\n";
    if ( isnan(pos.x() )) exit(2);    
}

btVector3 quat2Euler(const btQuaternion& q)
{
    // Implementation tested.
    // Using HPR order rotation. 
    // Angles are interpreted when Y points forward, Z points up.
    // https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/Quaternions.pdf

    // roll (y-axis rotation)
    double sinr = +2.0 * (q.w() * q.y() + q.x() * q.z());
    double cosr = +1.0 - 2.0 * (q.y() * q.y() + q.x() * q.x());
    double roll = atan2(sinr, cosr);

    // pitch (x-axis rotation)
    double sinp = +2.0 * (q.w() * q.x() - q.z() * q.y());
    double pitch;
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2., sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny = +2.0 * (q.w() * q.z() + q.y() * q.x());
    double cosy = +1.0 - 2.0 * (q.x() * q.x() + q.z() * q.z());
    double yaw = atan2(siny, cosy);
    return btVector3(yaw,pitch,roll);
}

btosgVec3 btosgQuat::toEuler()
{
    return quat2Euler(*this);
}

int btosgWorld::deleteAllObjects() {
    /// Deletes all abjects from the btosgWorld
    int count = 0;
    while ( !objects.empty() )
    {
	btosgObject *obj = objects.front();
	removeObject(obj);
	delete(obj);
        //std::cout << ' ' << mylist.front();
        count++;
    }
    return count;
}

btosgWorld::~btosgWorld() {
/*
    for ( auto it = objects.begin() ; it != objects.end(); ++it ) {
        btosgObject *obj = *it;
	removeObject(obj);
	//delete obj;
    }
*/
    deleteAllObjects();
/*
    std::forward_list<btosgObject*>::iterator it = objects.begin();
    while (it != objects.end())
    {
	std::forward_list<btosgObject*>::iterator next = it;
	next++;
	btosgObject *obj = *it;
        removeObject(obj);  // alternatively, i = items.erase(i);
	delete(obj);
	it = next;
    }
 */   

    delete dynamic; // ????
    delete solver;
    delete dispatcher;
    delete collisionConfiguration;
    delete broadphase;
    /*
    while (!objects.empty())
    {
        objects.pop_front();
    }
    */
    //scene->unref();
}

void btosgWorld::listObjects() {
    /// Outputs a list of all objects in the btosgWorld
    int n = 0;
    printf("## Object List\n");
    for ( auto it = objects.begin() ; it != objects.end(); ++it ) {
        btosgObject *obj = *it;
	obj->print();
	n++;
    }
    printf("%d objects listed.\n",n);
}

void btosgWorld::addObject(btosgObject *obj)  {
    /// Registers the object obj into a simulation world.
    objects.push_front(obj);
    if ( obj->body )  dynamic->addRigidBody(obj->body);
    else if ( _DEBUG_ ) fprintf(stderr, "Adding object without rigid body\n");
    //printf("adding object to World\n");
    if ( obj->model ) scene->addChild(obj->model);
    else if ( _DEBUG_ ) fprintf(stderr, "Adding object without visual model\n");
}

void btosgWorld::removeObject(btosgObject *obj)  {
    /// Unregisters the object obj from the simulation world.
    
    if ( obj->body )  dynamic->removeRigidBody(obj->body);
    else if ( _DEBUG_ ) fprintf(stderr, "Removing object without rigid body\n");
    //printf("removing object from World\n");
    if ( obj->model ) scene->removeChild(obj->model);
    else if ( _DEBUG_ ) fprintf(stderr, "Removing object without visual model\n");  
    // Remove from the list.
    // This also calls de the destructor for obj.
    objects.remove(obj); 
}

void btosgWorld::stepSimulation(btScalar timeStep, int maxSubSteps) {
    /// Performs a simulation step.
    if ( steps==0L ) {
        for ( auto it = objects.begin(); it != objects.end(); ++it ) {
            btosgObject *obj = *it;
            obj->setInitState();
        }
    }
    // listObjects();
    dynamic->stepSimulation(timeStep,maxSubSteps);

    for ( auto it = objects.begin(); it != objects.end(); ++it ) {
        btosgObject *obj = *it;
        obj->update();
    }
    steps += 1;
}

void btosgWorld::reset() {
    /// Reset all registered objects.
    for ( auto it = objects.begin(); it != objects.end(); ++it ) {
        btosgObject *obj = *it;
        obj->reset();
    }
}

void btosgObject::loadObjectModel(char const *fname) {
    /// Loads an object model from a Wavefront OBJ file.
    /// Loadded model is used to define both the collision and graphical shapes.
    if ( ! fname || ! *fname ) {
        fprintf(stderr, "Invalid object model filename\n");
        return;
    }
    osg::Node* loadedModel = osgDB::readNodeFile(fname);
    if ( ! loadedModel ) {
        fprintf(stderr, "Error reading Object model from file '%s'\n", fname);
    }

    if ( name ) {
	fprintf(stderr, "Warning: Object '%s' overloaded with object '%s'\n", name, fname);
    }

    setName(fname);
    if ( ! model )	model = new osg::PositionAttitudeTransform;
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
        stateset->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);
    }
    stateset->setMode(GL_LIGHTING, osg::StateAttribute::ON);

    model->setStateSet( stateset );
}

void btosgObject::reset() {
    /// Reposition object to its inital state.
    if ( body ) {
        body->setWorldTransform(init_state);
        body->getMotionState()->setWorldTransform(init_state);
        body->clearForces();
        if ( mass>0. ) {
            body->setLinearVelocity(btVector3(0., 0., 0.));
            body->setAngularVelocity(btVector3(0., 0., 0.));
            body->activate();   // Required if the object was asleep
        }
    }
    else { // Not required for dynamic objects.
        if ( model ) {
            model->setAttitude(btosgQuat(init_state.getRotation()));
            model->setPosition(btosgVec3(init_state.getOrigin()));
        }
    }
}

void btosgObject::createRigidBody() {
    /// Creates a new rigid body as a btRigidBody object.
    if ( ! shape ) {
        fprintf(stderr,"Cannot create RigidBody without shape\n");
        return;
    }
    btDefaultMotionState* mState = new
        btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0., 0., 0.)));
    btVector3 inertia(0, 0, 0);
    shape->calculateLocalInertia(mass,inertia);
    btRigidBody::btRigidBodyConstructionInfo cInfo(mass, mState, shape, inertia);
    //printf("mass: %f\n",mass);
    cInfo.m_restitution = 0.9f;
    cInfo.m_friction = 10.f;
    body = new btRigidBody(cInfo);
    if ( !body ) fprintf(stderr,"Error creating btBody\n");
}



