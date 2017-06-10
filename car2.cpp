void initPhysics()
{
    broadphase = new btDbvtBroadphase();
    collisionConfiguration = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfiguration);
    solver = new btSequentialImpulseConstraintSolver();

    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
    dynamicsWorld->setGravity(btVector3(0, -9.81, 0));

    // Debug Drawer
    bulletDebugugger.setDebugMode(btIDebugDraw::DBG_DrawWireframe);
    dynamicsWorld->setDebugDrawer(&bulletDebugugger);

    //groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);
    groundShape = new btBoxShape(btVector3(50, 3, 50));
    fallShape = new btBoxShape(btVector3(1, 1, 1));

    // Orientation and Position of Ground
    groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -3, 0)));
    btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
    groundRigidBody = new btRigidBody(groundRigidBodyCI);
    dynamicsWorld->addRigidBody(groundRigidBody);

    ///////////////////////////////////////////////////////////////////////
    //              Vehicle Setup
    ///////////////////////////////////////////////////////////////////////
    vehicleChassisShape = new btBoxShape(btVector3(1.f, 0.5f, 2.f));
    vehicleBody = new btCompoundShape();

    localTrans.setIdentity();
    localTrans.setOrigin(btVector3(0, 1, 0));
    vehicleBody->addChildShape(localTrans, vehicleChassisShape);

    localTrans.setOrigin(btVector3(3, 0.f, 0));
    vehicleMotionState = new btDefaultMotionState(localTrans);
    //vehicleMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(3, 0, 0)));
    btVector3 vehicleInertia(0, 0, 0);
    vehicleBody->calculateLocalInertia(vehicleMass, vehicleInertia);
    btRigidBody::btRigidBodyConstructionInfo vehicleRigidBodyCI(vehicleMass, vehicleMotionState, vehicleBody, vehicleInertia);

    vehicleRigidBody = new btRigidBody(vehicleRigidBodyCI);
    dynamicsWorld->addRigidBody(vehicleRigidBody);

    wheelShape = new btCylinderShapeX(btVector3(wheelWidth, wheelRadius, wheelRadius));
    {
        vehicleRayCaster = new btDefaultVehicleRaycaster(dynamicsWorld);
        vehicle = new btRaycastVehicle(vehicleTuning, vehicleRigidBody, vehicleRayCaster);

        // never deactivate vehicle
        vehicleRigidBody->setActivationState(DISABLE_DEACTIVATION);
        dynamicsWorld->addVehicle(vehicle);

        float connectionHeight = 1.2f;
        bool isFrontWheel = true;

        vehicle->setCoordinateSystem(rightIndex, upIndex, forwardIndex); // 0, 1, 2

        // add wheels
        // front left
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




