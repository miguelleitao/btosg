/*
	btosg.h 
	Miguel Leitao, 2016
*/



#include <osgDB/ReadFile>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osgGA/TrackballManipulator>
#include <osg/ShapeDrawable>
#include <osg/Plane>
#include <osg/LightSource>

#include <btBulletDynamicsCommon.h>

#ifdef BTOSG_SHADOW
#include <osgShadow/ShadowedScene>
#include <osgShadow/ShadowMap>
#endif
#include <forward_list>


#define _DEBUG_ (1)


osg::Vec3 bt2osg_Vec3(btVector3 bv);
osg::Vec4 bt2osg_Vec4(btVector4 bv);
btVector3 osg2bt_Vec3(osg::Vec3 bv);
osg::Quat bt2osg_Quat(btQuaternion bv);


const int ReceivesShadowTraversalMask = 0x1;
const int CastsShadowTraversalMask = 0x2;

class btosgNode {
    // Not used
    // previsto para possibilitar futura herarqui de DynamicBodies
    std::forward_list<class btosgNode*> children;
    class btosgNode* parent;
    void addChild(class btosgNode* node) {
        children.push_front(node);
        node->parent = this;
    }
};

class btosgWorld {
    private:
        unsigned long steps;
    public:
	btDynamicsWorld *dynamic;
#ifdef BTOSG_SHADOW
        osgShadow::ShadowedScene *scene;
#else
	osg::Group 	*scene;
#endif
        std::forward_list<class btosgObject*> objects;
	btosgWorld() {

		// Create dynamic world
		btBroadphaseInterface* broadphase = new btDbvtBroadphase();
	 	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	 	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
	 	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
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
    void stepSimulation(btScalar timeStep, int maxSubSteps);
    void addObject(class btosgObject *obj);
    void reset();
};


class btosgObject  {
   public:
	// Main components
	//osg::Geode *geo;
	osg::PositionAttitudeTransform *model;
        char *name;
    btTransform init_state;
	btRigidBody *body;
	// other
	btCollisionShape* shape;
	float mass;
	btosgObject() {
		model = NULL;
		body = NULL;
		shape = NULL; 
		mass = 0.;
                name = NULL;
                init_state = btTransform();
	};
        void setName(char const *n) {
            name = strdup(n);
        }
	void setMass(double m) {
		mass = m;
		bool isDynamic = (mass != 0.f);

    		btVector3 localInertia(0, 0, 0);
    		if (isDynamic)
        		shape->calculateLocalInertia(mass, localInertia);
		body->setMassProps(m, localInertia); 
	};
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
         }
    }
	void setPosition(const btVector3 &p) {
		if (body) {
                btTransform wTrans;
                            body->getMotionState()->getWorldTransform(wTrans);
                //mState.setRotation(btQuaternion(0,0,0,1));
                wTrans.setOrigin(p);
                body->setWorldTransform(wTrans);
                body->getMotionState()->setWorldTransform(wTrans);
                body->clearForces();
                body->setLinearVelocity(btVector3(0,0,0));
                body->setAngularVelocity(btVector3(0,0,0));
            }
            else {  // Not required for dynamic objects.
                if ( _DEBUG_ ) printf("set Position in non dynamic object\n");
                if (model) {
                    model->setPosition(bt2osg_Vec3(p));
                }
            }
        }
        void setPosition(float x, float y, float z) {
		setPosition(btVector3(x,y,z));
        }
	void setRotation(float x, float y, float z, float w) {
            if (body) {
                btTransform wTrans;
                body->getMotionState()->getWorldTransform(wTrans);
                wTrans.setRotation(btQuaternion(x,y,z,w));
                //wTrans.setOrigin(btVector3(x,y,z));
                body->setWorldTransform(wTrans);
                body->getMotionState()->setWorldTransform(wTrans);
                body->clearForces();
                body->setLinearVelocity(btVector3(0,0,0));
                body->setAngularVelocity(btVector3(0,0,0));
            }
            else {  // Not required for dynamic objects.
                if ( _DEBUG_ ) printf("setRotation in non dynamic object\n");
                if (model) {
                        model->setAttitude(osg::Quat(x,y,z,w));
                }
            }
	}
	
	void setRotation(osg::Quat q) {
            setRotation(q[0],q[1],q[2],q[3]);
        }
	
	void setTexture(char const *fname);
	virtual void update() {
        if (body) {
            btTransform wTrans;
            body->getMotionState()->getWorldTransform(wTrans);
            
            if ( model ) {
                model->setAttitude(bt2osg_Quat(wTrans.getRotation()));
                model->setPosition(bt2osg_Vec3(wTrans.getOrigin()));
            }

        }
    }
    void reset() {
        if ( body ) {
            body->setWorldTransform(init_state);
            body->getMotionState()->setWorldTransform(init_state);
            body->clearForces();
            if ( mass>0. ) {
                body->setLinearVelocity(btVector3(0,0,0));
                body->setAngularVelocity(btVector3(0,0,0));
                body->activate();   // Required if the object was asleep
                
            }
        }
        else { // Not required for dynamic objects.
            if ( model ) {
                model->setAttitude(bt2osg_Quat(init_state.getRotation()));
                model->setPosition(bt2osg_Vec3(init_state.getOrigin()));
            }
        }
    }
    void setInitState() {
        // Store init state.
        // Init state is aplied by reset()
        if (body) body->getMotionState()->getWorldTransform(init_state);
    }
    void setInitState(btTransform iState) {
        // Store init state.
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
		cInfo.m_restitution = 0.9f;
		cInfo.m_friction = 10.f;
		body = new btRigidBody(cInfo);
        if ( !body ) fprintf(stderr,"Error creating btBody\n");
    }
    void loadObjectModel(char const *fname);
        
};


class btosgSphere : public btosgObject {
    public:
	float radius;
	btosgSphere(float r) {
		radius = r;
		osg::Geode *geo = new osg::Geode();
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
            if (  !model)	model = new osg::PositionAttitudeTransform;
            model->addChild(geo);
            model->setNodeMask(CastsShadowTraversalMask);
            mass = m;
            shape = new btBoxShape( osg2bt_Vec3(dim/2.) );
            if ( !shape ) fprintf(stderr,"Error creating btShape\n");
            
            createRigidBody();
            printf("box created\n");
	}
	btosgBox(float x, float y, float z) : btosgBox( osg::Vec3(x,y,z) ) {};
        btosgBox(float r)                   : btosgBox( osg::Vec3(r,r,r) ) {};
};




class btosgPlane : public btosgObject {
    public:
	float dx,dy,dz;
    btosgPlane()  {
		dx = 10;
        dy = 10;
        dz = 0.001;
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
		shape = new btStaticPlaneShape(btVector3(0,0,1), 0);
        if ( !shape ) fprintf(stderr,"Error creating btShape\n");
        
        createRigidBody();
        printf("box created\n");
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
    };
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
            printf("cone created\n");
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
            if (  !model)	model = new osg::PositionAttitudeTransform;
            osg::PositionAttitudeTransform *center_pos = new osg::PositionAttitudeTransform;
            center_pos->setPosition(osg::Vec3(0.,0., 0.));
            center_pos->addChild(geo);
            model->addChild(center_pos);
            model->setNodeMask(ReceivesShadowTraversalMask);
            mass = 1.;
            shape = new btCylinderShapeZ(btVector3(r, r, h/2.));
            if ( !shape ) fprintf(stderr,"Error creating btShape\n");
            createRigidBody();
            body->setDamping(0.01,0.2);
            printf("cylinder created\n");
        }
};


class btosgBowlingPin : public btosgObject {
private:
        float height;
        float rhead;
        float rbody;
        float zbody;
    public:
    btosgBowlingPin()  {
        loadObjectModel("pino0.obj");
        
        model->setNodeMask(CastsShadowTraversalMask);
		mass = 6.5;
        height = 0.4;
        rhead = 0.04;
        rbody = 0.07;
        zbody = -0.02;
        
		//shape = new  btCylinderShapeZ(btVector3(raio,raio,altura/2.));
        
        float rhead = 0.04;
        btCompoundShape* shape = new btCompoundShape();
        if ( !shape ) fprintf(stderr,"Error creating btCompoundShape\n");
        // Neck
        shape->addChildShape(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), new btCylinderShapeZ(btVector3(rbody/2, rbody/2, height/2)) );
        // Body
        shape->addChildShape(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,zbody)), new btSphereShape(rbody) );
        // Head
        shape->addChildShape(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,height/2-rhead)), new btSphereShape(rhead) );
        
        shape->setMargin( 0.0002 ) ;
        
		btDefaultMotionState* mState = new
		  btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0.,0.,0.)));
          
		btVector3 inertia(0,0,0);
		shape->calculateLocalInertia(mass,inertia);
		btRigidBody::btRigidBodyConstructionInfo cInfo(mass,mState,shape,inertia);
		cInfo.m_restitution = 0.75f;
		cInfo.m_friction = 0.3f;
        //cInfo.m_rollingFriction = 0.99f;
		body = new btRigidBody(cInfo);
        if ( !body ) fprintf(stderr,"Error creating btBody for BowlingPin\n");
        body->setDamping(0.,0.2);
    };
    
};
