/*
 *	btosg.h 
 *
 *	(c) Miguel Leitao, 2016
 *
 */

#ifndef BTOSG_H
#define BTOSG_H 1

#define YES 1
#define NO  0

#include <osgDB/ReadFile>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osgGA/TrackballManipulator>
#include <osg/ShapeDrawable>
#include <osg/Plane>
#include <osg/LightSource>
#include <osg/Material>

#include <btBulletDynamicsCommon.h>

#if BTOSG_LOAD_OBJ==YES
#include "loadOBJ/include/LoadMeshFromObj.h"
#include "loadOBJ/include/GLInstanceGraphicsShape.h"
//include <GLInstanceGraphicsShape.h>
#endif

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

//osg::Vec3 bt2osg_Vec3(btVector3 bv);
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

//! btosgVec3 can be used to represent 3D points and vectors.
/*! Can be used as a btVector3 or an osg::Vec3.
 *  Implemented as derived from osg::Vec3 with aditional constructor and convertor from and to btVector3.
 */
class btosgVec3 : public osg::Vec3 {
    public:
	/**@brief Constructor from 3 scalars
	 * @param x X value
	 * @param y Y value
	 * @param z Z value
	 */
	btosgVec3(double x, double y, double z) : osg::Vec3(x,y,z) {};

	//! Constructor from base class osg::Vec3
	/*! @param v osg::Vec3 object
	 */
	btosgVec3(osg::Vec3f v) : osg::Vec3(v) {};
	btosgVec3(osg::Vec3d v) : osg::Vec3(v) {};
	btosgVec3(btVector3  v) : osg::Vec3(v[0],v[1],v[2]) {};
	operator btVector3() const {
		return btVector3(x(), y(), z());
	} 
};

//! btosgQuat represents a Quaternion.
/*! Can be used as a btQuaternion or an osg::Quat.
 *  Implemented as derived from osg::Quar with aditional constructor and convertor from and to btQuaternion.
 */
class btosgQuat : public osg::Quat {
    public:
	btosgQuat() : osg::Quat() {};
	/**@brief Constructor from 4 scalars
	 * @param x X value
	 * @param y Y value
	 * @param z Z value
	 * @param w W value
	 */
	btosgQuat(double x, double y, double z, double w) : osg::Quat(x,y,z,w) {};

	//! Constructor from base class osg::Quat
	/*! @param q osg::Quat object
	 */
	btosgQuat(osg::Quat  q) : osg::Quat(q) {};
	btosgQuat(btQuaternion  q) : osg::Quat(q[0],q[1],q[2],q[3]) {};
	operator btQuaternion() const {
		return btQuaternion(x(), y(), z(), w());
	}
	btosgVec3 toEuler(); 
};

//! Physical and visual world.
/*! Integrates a btDynamicsWorld and an osg::Group.
 *  Provides automated updating of graphical objects from related 
 *  physical simulated bodies.
 */
class btosgWorld {
    private:
        unsigned long steps;
        btDbvtBroadphase* broadphase;
        btDefaultCollisionConfiguration* collisionConfiguration;
	btCollisionDispatcher* dispatcher;
        btSequentialImpulseConstraintSolver* solver;
    public:
	btDynamicsWorld *dynamic;	///< Pointer to btDynamicsWorld object.
	/// Pointer to osg::Group object storing the root node of the object tree.
#ifdef BTOSG_SHADOW
        osg::ref_ptr<osgShadow::ShadowedScene> scene;
#else
	osg::ref_ptr<osg::Group> 	scene;
#endif
        std::forward_list<class btosgObject*> objects; ///< List of Objects.
	btosgWorld() {

		// Create dynamic world
		broadphase = new btDbvtBroadphase();
	 	collisionConfiguration = new btDefaultCollisionConfiguration();
	 	dispatcher = new btCollisionDispatcher(collisionConfiguration);
	 	solver = new btSequentialImpulseConstraintSolver;
	 	dynamic = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);

		// Set Gravity
		dynamic->setGravity(btosgVec3(0., 0., -9.8));

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

//!  Main btosg object base class.
/*!  Integrates a physical object (btRigidBody), a collision shape (btCollisionShape)
 *   and a graphical object (osg::PositionAttitudeTransform).
 *   Most object classes derive directly from btosgObject class.
 */
class btosgObject {
   public:
	// Main components
	//osg::Geode *geo;
	osg::ref_ptr<osg::PositionAttitudeTransform> model;	///< Object's graphical model
	char *name;	///< Object name
	btTransform init_state;   ///< Inital state. Applied on reset events.
	btRigidBody *body;	  ///< object's rigid body
	// other
	btCollisionShape* shape;  ///< Object's collision shape.
	float mass;		  ///< Mass of object
	btosgObject() {
		model = NULL;
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
	    if( model ) {
		model->unref();
		model = NULL;
	    }
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
	    /// Sets the object's name.
            name = strdup(n);
        }
	void setMass(double m) {
		/// Sets the object's mass.
		mass = m;
		bool isDynamic = (mass != 0.f);

    		btVector3 localInertia(0, 0, 0);
    		if (isDynamic && shape)
        		shape->calculateLocalInertia(mass, localInertia);
		if ( body ) body->setMassProps(m, localInertia); 
	}
	btosgVec3 getPosition() {
		/// Returns object's position.
         	if (body) {
            		btTransform wTrans;
			body->getMotionState()->getWorldTransform(wTrans);
            		return wTrans.getOrigin();
         	}
		if (model) return model->getPosition();
		return btosgVec3(0.,0.,0.);
    	}
	btosgQuat getRotation() {
                /// Returns object's attitude as a Quaternion.
         	if (body) {
            		btTransform wTrans;
			body->getMotionState()->getWorldTransform(wTrans);
            		return wTrans.getRotation();
         	}
		if (model) return model->getAttitude();
		return btosgQuat(0.,0.,0.,1.);
    	}
	btosgVec3 getEuler() {
		/// Returns object's attitude as HPR Euler angles.
		btosgQuat qt = getRotation();
		return qt.toEuler();
	}
	void setPosition(const btosgVec3 &p) {
		/// Sets objects position.
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
                if (model) {
                    model->setPosition(p);
                }
            }
        }
        void setPosition(float x, float y, float z) {
		/// Sets objects position.
		setPosition(btosgVec3(x,y,z));
        }
	void setRotation(btosgQuat q) {
	    /// Sets objects attitude from a quaternion.
            if (body) {
                btTransform wTrans;
                body->getMotionState()->getWorldTransform(wTrans);
                wTrans.setRotation(q);
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
                if (model) {
                        model->setAttitude(q);
                }
            }
	}
	void setRotation(float x, float y, float z, float w) {
	    /// Sets objects attitude from the quaternion coords.
	    setRotation(btosgQuat(x,y,z,w));
	}
	
	void setTexture(char const *fname); //< Sets a texture from an image file.
	void setMaterial(osg::ref_ptr<osg::Material> mat) {
		/// Sets the material properties for the object.
		model->getOrCreateStateSet()->setAttributeAndModes(mat, osg::StateAttribute::ON);
	}
	void logPosition() {
		/// Outputs object's position.
         	btosgVec3 pos = getPosition();
       		if ( name )
       		    std::cout << "Object " << name << " position " << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
       		else
       		    std::cout << "Object _NO_NAME_ position " << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
    	}
	virtual void update() {
	    /// Objects's update callback.
	    /// This function is called automatically from World::setpSimulation() for each registered object.
	    /// Positions graphical object from its physhical state.
            if (body) {
            	btTransform wTrans;
            	body->getMotionState()->getWorldTransform(wTrans);
            	if ( model ) {
            	    model->setAttitude(btosgQuat(wTrans.getRotation()));
            	    model->setPosition(btosgVec3(wTrans.getOrigin()));
            	}
    	    }
    	}
    	void reset() {
	    /// Reposition object to its inital state.
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
            	if ( model ) {
            	    model->setAttitude(btosgQuat(init_state.getRotation()));
            	    model->setPosition(btosgVec3(init_state.getOrigin()));
            	}
            }
    	}
	void setInitState() {
       	    /// Stores current state as init state.
            /// Init state is aplied by reset()
            if (body) body->getMotionState()->getWorldTransform(init_state);
    	}
    	void setInitState(btTransform iState) {
            /// Stores iState as init state.
            /// Init state is aplied by reset()
            init_state = iState;
    	}
    	void createRigidBody() {
	    /// Creates a new rigid body as a btRigidBody object.
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

#if BTOSG_LOAD_OBJ
/// Object from loaded model
class btosgExternalObject : public btosgObject {
    public:
	//char *fname;
	btosgExternalObject(const char *file_name) {
		/// Constructs an object from an external model.
		loadObjectModel(file_name);

   		GLInstanceGraphicsShape* glmesh = LoadMeshFromObj(file_name, "");
   		//printf("[INFO] Obj loaded: Extracted %d verticed from obj file [%s]\n", glmesh->m_numvertices, file_name);

		const GLInstanceVertex& v = glmesh->m_vertices->at(0);
   		btConvexHullShape* shapeH = new btConvexHullShape((const btScalar*)(&(v.xyzw[0])), glmesh->m_numvertices, sizeof(GLInstanceVertex));
   		//btVector3 color(1,1,1);
   		btVector3 scaling(.999,.999,.999);
   		shapeH->setLocalScaling(scaling);
		shape = shapeH;

		mass = 1.;
                createRigidBody();
                body->setDamping(0.01,0.1);
	}
};
#endif

/// Sphere
class btosgSphere : public btosgObject {
    public:
	float radius;		///< Radius
	btosgSphere(float r) {
		/// Constructs a Sphere object.
		radius = r;
		osg::ref_ptr<osg::Geode> geo = new osg::Geode();
		geo->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.,0.,0.),r)));
		if (  !model)	model = new osg::PositionAttitudeTransform;
		model->addChild(geo);
                model->setNodeMask(CastsShadowTraversalMask);
		shape = new btSphereShape(r);
		mass = 1.;
                createRigidBody();
                body->setDamping(0.01,0.1);
	}
};

 
//!  Axis oriented box..
/*!  Represents a physical and visual axis oriented box.
 */
class btosgBox : public btosgObject {
    public:
	float dx;	///<  x dimension
	float dy;	///<  y dimension
	float dz;	///<  z dimension
	btosgBox(btosgVec3 dim = btosgVec3(1.,1.,1.), double m=1. ) {
	    /// Constructs an axis oriented box.
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
            shape = new btBoxShape( btosgVec3(dim/2.) );
            if ( !shape ) fprintf(stderr,"Error creating btShape\n");
            
            createRigidBody();
            //printf("box created\n");
	}
	btosgBox(float x, float y, float z) : btosgBox( btosgVec3(x,y,z) ) {
	    /// Constructs an axis oriented box.
	};
        btosgBox(float r)                   : btosgBox( btosgVec3(r,r,r) ) {
	    /// Constructs an axis oriented box.
	};
};


/// Infinite plane
class btosgPlane : public btosgObject {
    // Physical infinit, axis oriented plane.
    // Viewable as a finit, axis oriented, small depth box.
    private: 
	float dx,dy,dz;
    public:
    	btosgPlane()  :  btosgPlane(10., 10., 0.) { 
		/// Constructs a physical infinite plane, viewable as low thickness finite box.
                /// Viewable box has dimensions 10,10,0.
                /// Plane is created facing Z axis.
	};
		
        btosgPlane( btosgVec3 v ) : btosgPlane( v[0], v[1], v[2] ) {
		/// Constructs a physical infinite plane, viewable as low thickness finite box.
                /// Viewable box has dimensions v.x,v.y,v.z.
                /// Minimum dimension selects physical plane orientatiton.
                /// Plane is created as axis oriented.
	};
    	btosgPlane(float dx, float dy, float dz)  {
		/// Constructs a physical infinite plane, viewable as low thickness finite box.
		/// Viewable box has dimensions dx,dy,dz. 
		/// Minimum dimension selects physical plane orientatiton.
		/// Plane is created as axis oriented.
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
		if (  !model)	model = new osg::PositionAttitudeTransform;
		model->addChild(geo);
		model->setNodeMask(ReceivesShadowTraversalMask);
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
		/// Creates a Rigid Body
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


/// Cone
class btosgCone : public btosgObject {
    private:
        float radius;
        float height;
    public:
        btosgCone(float r=0.5, float h=1)  {
	    /// Constructs a Z axis cone.
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
            if (  !model)	model = new osg::PositionAttitudeTransform;
            osg::PositionAttitudeTransform *center_pos = new osg::PositionAttitudeTransform;
            center_pos->setPosition(osg::Vec3(0.,0.,-height/4.));
            center_pos->addChild(geo);
            model->addChild(center_pos);
            model->setNodeMask(ReceivesShadowTraversalMask);
            mass = 1.;
            shape = new btConeShapeZ(r, h);
            if ( !shape ) fprintf(stderr,"Error creating btShape\n");
            createRigidBody();
            body->setDamping(0.01,0.2);
            //printf("cone created\n");
        }
};

/// Cylinder
class btosgCylinder : public btosgObject {
    private:
	float radius;
        float height;
    public:
        btosgCylinder(float r=0.5, float h=1)  {
	    /// Constructs a Z axis cylinder.
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
            if ( !model )	model = new osg::PositionAttitudeTransform;
            osg::PositionAttitudeTransform *center_pos = new osg::PositionAttitudeTransform;
            center_pos->setPosition(osg::Vec3(0.,0., 0.));
            center_pos->addChild(geo);
            model->addChild(center_pos);
            model->setNodeMask(ReceivesShadowTraversalMask);
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

