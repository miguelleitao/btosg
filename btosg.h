/*
 *	btosg.h 
 *
 *	(c) Miguel Leitao, 2016
 *
 */

#ifndef BTOSG_H
#define BTOSG_H 1

#include <osgDB/ReadFile>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osgGA/TrackballManipulator>
#include <osg/ShapeDrawable>
#include <osg/Plane>
#include <osg/LightSource>
#include <osg/Material>

#include <btBulletDynamicsCommon.h>

#include "loadOBJ/include/LoadMeshFromObj.h"
#include "loadOBJ/include/GLInstanceGraphicsShape.h"

//include <GLInstanceGraphicsShape.h>

#ifdef BTOSG_SHADOW
#include <osgShadow/ShadowedScene>
#include <osgShadow/ShadowMap>
#endif
#include <forward_list>

/* Direct update of positions and orientations can be avoided for dynamic objects
 * or normal objects that are syncronized during btosgWorld::update.
 * Direct update can be enabled to allow setPosition and setOrientation methods to 
 * update the graphics model directly.
 * This can be used in special cases like objects not belonging to the stepped 
 * btosgWorld or Worlds that are not stepped through btosgWorld::stepSimulation().
 */
#define AVOID_DIRECT_MODEL_UPDATE

osg::Vec3 bt2osg_Vec3(btVector3 bv);
osg::Vec4 bt2osg_Vec4(btVector4 bv);
btVector3 osg2bt_Vec3(osg::Vec3 bv);
osg::Quat bt2osg_Quat(btQuaternion bv);
btQuaternion osg2bt_Quat(osg::Quat bv);
btVector3 quat2Euler(const btQuaternion& q);

#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

const int ReceivesShadowTraversalMask = 0x1;
const int CastsShadowTraversalMask = 0x2;

inline int btosgPrint(char const *name, const int v) {
    return printf("%s: %d\n", name, v);
}
inline int btosgPrint(char const *name, const float v) {
    return printf("%s: %f\n", name, v);
}

inline int btosgPrint(char const *name, const btVector3&  vec) {
    return printf("%s: %f %f %f\n", name, vec[0], vec[1], vec[2]);
}

/*
class btosgNode {
    // Not used.
    // May allow future hierarchy of DynamicBodies.
    std::forward_list<class btosgNode*> children;
    class btosgNode* parent;
    void addChild(class btosgNode* node) {
        children.push_front(node);
        node->parent = this;
    }
};
*/

class btosgWorld {
    private:
        unsigned long steps;
        btDbvtBroadphase* broadphase;
        btDefaultCollisionConfiguration* collisionConfiguration;
	btCollisionDispatcher* dispatcher;
        btSequentialImpulseConstraintSolver* solver;
    public:
	btDynamicsWorld *dynamic;
#ifdef BTOSG_SHADOW
        osg::ref_ptr<osgShadow::ShadowedScene> scene;
#else
	osg::ref_ptr<osg::Group> 	scene;
#endif
        std::forward_list<class btosgObject*> objects;
	btosgWorld() {

		// Create dynamic world
		broadphase = new btDbvtBroadphase();
	 	collisionConfiguration = new btDefaultCollisionConfiguration();
	 	dispatcher = new btCollisionDispatcher(collisionConfiguration);
	 	solver = new btSequentialImpulseConstraintSolver;
	 	dynamic = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);

		// Set Gravity
		dynamic->setGravity(btVector3(0., 0., -9.8));

		// Creating the root node
                #ifdef BTOSG_SHADOW
                    scene = new osgShadow::ShadowedScene;
                    scene->setReceivesShadowTraversalMask(ReceivesShadowTraversalMask);
                    scene->setCastsShadowTraversalMask(CastsShadowTraversalMask);
                    osg::ref_ptr<osgShadow::ShadowMap> sm = new osgShadow::ShadowMap;
                    scene->setShadowTechnique(sm.get());
                    sm->setTextureSize(osg::Vec2s(8096,8096));
                #else
                    scene = new osg::Group;// Creating the root node
                #endif

                steps = 0L;
	};
    ~btosgWorld();
    void stepSimulation(btScalar timeStep, int maxSubSteps);
    void addObject(class btosgObject *obj);
    void reset();
};
/*
class btosgRigidBody : public btRigidBody {
    public:
	btosgRigidBody(btRigidBody::btRigidBodyConstructionInfo& ci) : btRigidBody(ci) {}; 
	void setRestitution(float r) {
	    m_restitution = r;
	}
};
*/
class btosgObject : public osg::PositionAttitudeTransform {
   public:
	// Main components
	//osg::Geode *geo;
	//osg::ref_ptr<osg::PositionAttitudeTransform> model;
	char *name;
	btTransform init_state;
	btRigidBody *body;
	// other
	btCollisionShape* shape;
	float mass;
	btosgObject() {
		//model = NULL;
		body = NULL;
		shape = NULL; 
		mass = 0.;
                name = NULL;
                init_state = btTransform();
	};
        virtual ~btosgObject() {
            if (body) {
		delete body->getMotionState();
		delete body;
		body = NULL;
	    }
	    /*
	    if( model ) {
		model->unref();
		model = NULL;
	    }
	    */
            if (shape) {
		delete shape;
		shape = NULL;
	    }
            if (name) {
		delete name;
		name = NULL;
	    }
        }
        void setName(char const *n) {
            name = strdup(n);
        }
	void setMass(double m) {
		mass = m;
		bool isDynamic = (mass != 0.f);

    		btVector3 localInertia(0, 0, 0);
    		if (isDynamic && shape)
        		shape->calculateLocalInertia(mass, localInertia);
		if ( body ) body->setMassProps(m, localInertia); 
	}
	btVector3 getPosition() {
         	if (body) {
            		btTransform wTrans;
			body->getMotionState()->getWorldTransform(wTrans);
            		return wTrans.getOrigin();
         	}
		return osg2bt_Vec3(osg::PositionAttitudeTransform::getPosition());
		return btVector3(0.,0.,0.);
    	}
	btQuaternion getRotation() {
         	if (body) {
            		btTransform wTrans;
			body->getMotionState()->getWorldTransform(wTrans);
            		return wTrans.getRotation();
         	}
		osg2bt_Quat(osg::PositionAttitudeTransform::getAttitude());
		return btQuaternion(0.,0.,0.,0.);
    	}
	btVector3 getEuler() {
		btQuaternion qt = getRotation();
		return quat2Euler(qt);
	}
	void setPosition(const btVector3 &p) {
	    if (body) {
                btTransform wTrans;
                body->getMotionState()->getWorldTransform(wTrans);
                wTrans.setOrigin(p);
                body->setWorldTransform(wTrans);
                body->getMotionState()->setWorldTransform(wTrans);
                body->clearForces();
                body->setLinearVelocity(btVector3(0,0,0));
                body->setAngularVelocity(btVector3(0,0,0));
            }
            #ifdef AVOID_DIRECT_MODEL_UPDATE
            else
            #endif
            {   // Not required for dynamic objects.
                #ifdef _DEBUG_
		   printf("set Position in non dynamic object\n");
		#endif
		   osg::PositionAttitudeTransform::setPosition(bt2osg_Vec3(p));
            }
        }
        void setPosition(float x, float y, float z) {
		setPosition(btVector3(x,y,z));
        }
	void setRotation(btQuaternion q) {
            if (body) {
                btTransform wTrans;
                body->getMotionState()->getWorldTransform(wTrans);
                wTrans.setRotation(q);
                //wTrans.setOrigin(btVector3(x,y,z));
                body->setWorldTransform(wTrans);
                body->getMotionState()->setWorldTransform(wTrans);
                body->clearForces();
                body->setLinearVelocity(btVector3(0,0,0));
                body->setAngularVelocity(btVector3(0,0,0));
            }
            #ifdef AVOID_DIRECT_MODEL_UPDATE
            else 
            #endif
            {   // Not required for dynamic objects.
                #ifdef  _DEBUG_
		  printf("setRotation in non dynamic object\n");
		#endif
		  osg::PositionAttitudeTransform::setAttitude(bt2osg_Quat(q));
            }
	}
	void setRotation(float x, float y, float z, float w) {
	    setRotation(btQuaternion(x,y,z,w));
	}
	
	void setRotation(osg::Quat q) {
            setRotation(q[0],q[1],q[2],q[3]);
	}
	
	void setTexture(char const *fname);
	void setMaterial(osg::ref_ptr<osg::Material> mat) {
		getOrCreateStateSet()->setAttributeAndModes(mat, osg::StateAttribute::ON);
	}
	void logPosition() {
         	btVector3 pos = getPosition();
       		if ( name )
       		    std::cout << "Object " << name << " position " << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
       		else
       		    std::cout << "Object _NO_NAME_ position " << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
    	}
	virtual void update() {
            if (body) {
            	btTransform wTrans;
            	body->getMotionState()->getWorldTransform(wTrans);
            	osg::PositionAttitudeTransform::setAttitude(bt2osg_Quat(wTrans.getRotation()));
		osg::PositionAttitudeTransform::setPosition(bt2osg_Vec3(wTrans.getOrigin()));
    	    }
    	}
    	void reset() {
            if ( body ) {
            	body->setWorldTransform(init_state);
            	body->getMotionState()->setWorldTransform(init_state);
            	body->clearForces();
            	if ( mass>0. ) {
            	    body->setLinearVelocity(btVector3(0.,0.,0.));
            	    body->setAngularVelocity(btVector3(0.,0.,0.));
            	    body->activate();   // Required if the object was asleep
            	}
            }
            else { // Not required for dynamic objects.
            	osg::PositionAttitudeTransform::setAttitude(bt2osg_Quat(init_state.getRotation()));
		osg::PositionAttitudeTransform::setPosition(bt2osg_Vec3(init_state.getOrigin()));
            }
    	}
	void setInitState() {
       	    // Store current state as init state.
            // Init state is aplied by reset()
            if (body) body->getMotionState()->getWorldTransform(init_state);
    	}
    	void setInitState(btTransform iState) {
            // Store iState as init state.
            // Init state is aplied by reset()
            init_state = iState;
    	}
    	void createRigidBody() {
            if ( ! shape ) {
                fprintf(stderr,"Cannot create RigidBody without shape\n");
                return;
            }
            btDefaultMotionState* mState = new 
			btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0.,0.,0.)));
	    btVector3 inertia(0,0,0);
	    shape->calculateLocalInertia(mass,inertia);
	    btRigidBody::btRigidBodyConstructionInfo cInfo(mass,mState,shape,inertia);
            //printf("mass: %f\n",mass);
	    cInfo.m_restitution = 0.9f;
	    cInfo.m_friction = 10.f;
	    body = new btRigidBody(cInfo);
            if ( !body ) fprintf(stderr,"Error creating btBody\n");
    	}
    	void loadObjectModel(char const *fname);
};

class btosgExternalObject : public btosgObject {
    public:
	//char *fname;
	btosgExternalObject(const char *file_name) {
		loadObjectModel(file_name);

   		GLInstanceGraphicsShape* glmesh = LoadMeshFromObj(file_name, "");
   		//printf("[INFO] Obj loaded: Extracted %d verticed from obj file [%s]\n", glmesh->m_numvertices, file_name);

		const GLInstanceVertex& v = glmesh->m_vertices->at(0);
   		btConvexHullShape* shapeH = new btConvexHullShape((const btScalar*)(&(v.xyzw[0])), glmesh->m_numvertices, sizeof(GLInstanceVertex));
   		btVector3 color(1,1,1);
   		btVector3 scaling(.999,.999,.999);
   		shapeH->setLocalScaling(scaling);
		shape = shapeH;

		mass = 1.;
                createRigidBody();
                body->setDamping(0.01,0.1);
	}
};

class btosgSphere : public btosgObject {
    public:
	float radius;
	btosgSphere(float r) {
		radius = r;
		osg::ref_ptr<osg::Geode> geo = new osg::Geode();
		geo->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.,0.,0.),r)));
		//if (  !model)	model = new osg::PositionAttitudeTransform;
		addChild(geo);
                setNodeMask(CastsShadowTraversalMask);
		shape = new btSphereShape(r);
		mass = 1.;
                createRigidBody();
                body->setDamping(0.01,0.1);
	}
};


class btosgBox : public btosgObject {
    public:
	float dx,dy,dz;
	btosgBox(osg::Vec3 dim = osg::Vec3(1.,1.,1.), double m=1. ) {
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
            //if (  !model)	model = new osg::PositionAttitudeTransform;
            addChild(geo);
            setNodeMask(CastsShadowTraversalMask);
            mass = m;
            shape = new btBoxShape( osg2bt_Vec3(dim/2.) );
            if ( !shape ) fprintf(stderr,"Error creating btShape\n");
            
            createRigidBody();
            //printf("box created\n");
	}
	btosgBox(float x, float y, float z) : btosgBox( osg::Vec3(x,y,z) ) {};
        btosgBox(float r)                   : btosgBox( osg::Vec3(r,r,r) ) {};
};



class btosgPlane : public btosgObject {
    // Physical infinit, axis oriented plane.
    // Viewable as a finit, axis oriented, small depth box.
    public:
	float dx,dy,dz;
    	btosgPlane()  :  btosgPlane(10., 10., 0.) { };
        btosgPlane( osg::Vec3 v ) : btosgPlane( v[0], v[1], v[2] ) { };
    	btosgPlane(float dx, float dy, float dz)  {
		dx = max(dx, 0.001);
        	dy = max(dy, 0.001);
        	dz = max(dz, 0.001);
		osg::Geode *geo = new osg::Geode();
		if ( geo ) {
		    osg::Shape *sp = new osg::Box( osg::Vec3(0.,0.,0.), dx, dy, dz );
		    if ( sp) {
		        osg::ShapeDrawable *sd = new osg::ShapeDrawable(sp);
		        if ( sd ) 
		            geo->addDrawable(sd);
		        else fprintf(stderr,"Error creating osg::Shape\n");
		    } else fprintf(stderr,"Error creating osg::Shape\n");
		} else fprintf(stderr,"Error creating Geode\n");
		//if (  !model)	model = new osg::PositionAttitudeTransform;
		addChild(geo);
		setNodeMask(ReceivesShadowTraversalMask);
		mass = 0;
		btVector3 norm(0.,0.,1);
		if ( dx<dy && dx<dz ) 	   norm = btVector3(1.,0.,0.);
		else if ( dy<dx && dy<dz ) norm = btVector3(0.,1.,0.);
		shape = new btStaticPlaneShape(norm, 0);
		if ( !shape ) fprintf(stderr,"Error creating btShape\n");
		createRigidBody();
		// printf("box created\n");
	}
	void createRigidBody() {
        	btDefaultMotionState* mState = new 
			btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0.,0.,0.)));
		btVector3 inertia(0,0,0);
		//shape->calculateLocalInertia(mass,inertia);
		btRigidBody::btRigidBodyConstructionInfo cInfo(mass,mState,shape,inertia);
		cInfo.m_restitution = 0.9f;
		cInfo.m_friction = 0.9f;
		body = new btRigidBody(cInfo);
        	if ( !body ) fprintf(stderr,"Error creating btBody\n");
    	}
};



class btosgCone : public btosgObject {
    public:
	float radius;
        float height;
        btosgCone(float r=0.5, float h=1)  {
            radius = r;
            height = h;
            osg::Geode *geo = new osg::Geode();
            if ( geo ) {
                osg::Shape *sp = new osg::Cone( osg::Vec3(0.,0.,0.), r, h);
                if ( sp) {
                    osg::ShapeDrawable *sd = new osg::ShapeDrawable(sp);
                    if ( sd ) 
                        geo->addDrawable(sd);
                    else fprintf(stderr,"Error creating osg::Shape\n");
                } else fprintf(stderr,"Error creating osg::Shape\n");
            } else fprintf(stderr,"Error creating Geode\n");
            //if (  !model)	model = new osg::PositionAttitudeTransform;
            osg::PositionAttitudeTransform *center_pos = new osg::PositionAttitudeTransform;
            center_pos->setPosition(osg::Vec3(0.,0.,-height/4.));
            center_pos->addChild(geo);
            addChild(center_pos);
            setNodeMask(ReceivesShadowTraversalMask);
            mass = 1.;
            shape = new btConeShapeZ(r, h);
            if ( !shape ) fprintf(stderr,"Error creating btShape\n");
            createRigidBody();
            body->setDamping(0.01,0.2);
            //printf("cone created\n");
        }
};

class btosgCylinder : public btosgObject {
    public:
	float radius;
        float height;
        btosgCylinder(float r=0.5, float h=1)  {
            radius = r;
            height = h;
            osg::Geode *geo = new osg::Geode();
            if ( geo ) {
                osg::Shape *sp = new osg::Cylinder( osg::Vec3(0.,0.,0.), r, h);
                if ( sp) {
                    osg::ShapeDrawable *sd = new osg::ShapeDrawable(sp);
                    if ( sd ) 
                        geo->addDrawable(sd);
                    else fprintf(stderr,"Error creating osg::Shape\n");
                } else fprintf(stderr,"Error creating osg::Shape\n");
            } else fprintf(stderr,"Error creating Geode\n");
            //if ( !model )	model = new osg::PositionAttitudeTransform;
            osg::PositionAttitudeTransform *center_pos = new osg::PositionAttitudeTransform;
            center_pos->setPosition(osg::Vec3(0.,0., 0.));
            center_pos->addChild(geo);
            addChild(center_pos);
            setNodeMask(ReceivesShadowTraversalMask);
            mass = 1.;
            // btCylinderShapeZ is centered at origin
            shape = new btCylinderShapeZ(btVector3(r, r, h/2.));
            if ( !shape ) fprintf(stderr,"Error creating btShape\n");
            createRigidBody();
            body->setDamping(0.01,0.2);
            //printf("cylinder created\n");
        }
};

#endif // BTOSG_H

