#include <Urho3D/Urho3D.h>

#include <Urho3D/Physics/CollisionShape.h>
#include <Urho3D/Physics/Constraint.h>
#include <Urho3D/Core/Context.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Physics/PhysicsEvents.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/PhysicsUtils.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/Graphics/DebugRenderer.h>

#include "Vehicle.h"

#include <SDL/SDL_log.h>
#include <Bullet/BulletDynamics/Vehicle/btRaycastVehicle.h>
#include <Bullet/BulletDynamics/Dynamics/btDynamicsWorld.h>

//=============================================================================
//=============================================================================
#define CUBE_HALF_EXTENTS   1
#define DELETE_NULL(x)      { if (x) delete x; x = NULL; }

//=============================================================================
//=============================================================================
Vehicle::Vehicle(Context* context) 
    : LogicComponent( context )
    , steering_( 0.0f )
{
    // fixed update() for inputs and post update() to sync wheels for rendering
    SetUpdateEventMask( USE_FIXEDUPDATE | USE_POSTUPDATE );

    m_fEngineForce = 0.0f;                                                 
    m_fBreakingForce = 0.0f;                                               
                                                                        
    m_fmaxEngineForce = 2500.f;//this should be engine/velocity dependent  
    m_fmaxBreakingForce = 100.f;                                           
                                                                        
    m_fVehicleSteering = 0.0f;                                             
    m_fsteeringIncrement = 0.04f;                                          
    m_fsteeringClamp = 0.3f;                                               
    m_fwheelRadius = 0.5f;                                                 
    m_fwheelWidth = 0.4f;                                                  
    m_fwheelFriction = 1000;//BT_LARGE_FLOAT;                              
    m_fsuspensionStiffness = 14.0f;//20.f;                                 
    m_fsuspensionDamping = 2.0f;//2.3f;                                    
    m_fsuspensionCompression = 4.0f;//4.4f;                                
    m_frollInfluence = 0.01f;//1.0f;                                       
    m_fsuspensionRestLength = 0.6f;//0.6

    m_vehicleRayCaster = NULL;
    m_vehicle = NULL;

    m_vpNodeWheel.Clear();
}

//=============================================================================
//=============================================================================
Vehicle::~Vehicle()
{
    DELETE_NULL( m_vehicleRayCaster );
    DELETE_NULL( m_vehicle );
    
    m_vpNodeWheel.Clear();
}

//=============================================================================
//=============================================================================
void Vehicle::RegisterObject(Context* context)
{
    context->RegisterFactory<Vehicle>();
    
    ATTRIBUTE("Controls Yaw", float, controls_.yaw_, 0.0f, AM_DEFAULT);
    ATTRIBUTE("Controls Pitch", float, controls_.pitch_, 0.0f, AM_DEFAULT);
    ATTRIBUTE("Steering", float, steering_, 0.0f, AM_DEFAULT);
}

//=============================================================================
//=============================================================================
void Vehicle::ApplyAttributes()
{
}

//=============================================================================
//=============================================================================
void Vehicle::Init()
{
    // This function is called only from the main program when initially creating the vehicle, not on scene load
    ResourceCache* cache = GetSubsystem<ResourceCache>();
    
    StaticModel* hullObject = node_->CreateComponent<StaticModel>();
    hullBody_ = node_->CreateComponent<RigidBody>();
    CollisionShape* hullColShape = node_->CreateComponent<CollisionShape>();

    hullBody_->SetMass(800.0f);
    hullBody_->SetLinearDamping(0.2f); // Some air resistance
    hullBody_->SetAngularDamping(0.5f);
    hullBody_->SetCollisionLayer(1);
    
    int rightIndex = 0;
    int upIndex = 1;
    int forwardIndex = 2;
    Scene* scene = GetScene();
    PhysicsWorld *pPhysWorld = scene->GetComponent<PhysicsWorld>();
    btDynamicsWorld *pbtDynWorld = (btDynamicsWorld*)pPhysWorld->GetWorld();

    m_vehicleRayCaster = new btDefaultVehicleRaycaster( pbtDynWorld );
    m_vehicle = new btRaycastVehicle( m_tuning, hullBody_->GetBody(), m_vehicleRayCaster );
    pbtDynWorld->addVehicle( m_vehicle );

    m_vehicle->setCoordinateSystem( rightIndex, upIndex, forwardIndex );

    node_->SetScale( Vector3(1.5f, 1.0f, 3.5f) );
    Vector3 v3BoxExtents = Vector3::ONE;//Vector3(1.5f, 1.0f, 3.0f);
    hullColShape->SetBox( v3BoxExtents );

    hullObject->SetModel(cache->GetResource<Model>("Models/Box.mdl"));
    hullObject->SetMaterial(cache->GetResource<Material>("Materials/Stone.xml"));
    hullObject->SetCastShadows(true);

    float connectionHeight = -0.4f;//1.2f;
    bool isFrontWheel=true;
    btVector3 wheelDirectionCS0(0,-1,0);
    btVector3 wheelAxleCS(-1,0,0);

    btVector3 connectionPointCS0(CUBE_HALF_EXTENTS-(0.3f*m_fwheelWidth),connectionHeight,2*CUBE_HALF_EXTENTS-m_fwheelRadius);
    m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,m_fsuspensionRestLength,m_fwheelRadius,m_tuning,isFrontWheel);

    connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3f*m_fwheelWidth),connectionHeight,2*CUBE_HALF_EXTENTS-m_fwheelRadius);
    m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,m_fsuspensionRestLength,m_fwheelRadius,m_tuning,isFrontWheel);

    isFrontWheel = false;
    connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3f*m_fwheelWidth),connectionHeight,-2*CUBE_HALF_EXTENTS+m_fwheelRadius);
    m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,m_fsuspensionRestLength,m_fwheelRadius,m_tuning,isFrontWheel);

    connectionPointCS0 = btVector3(CUBE_HALF_EXTENTS-(0.3f*m_fwheelWidth),connectionHeight,-2*CUBE_HALF_EXTENTS+m_fwheelRadius);
    m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,m_fsuspensionRestLength,m_fwheelRadius,m_tuning,isFrontWheel);

    for ( int i = 0; i < m_vehicle->getNumWheels(); i++ )
    {
        btWheelInfo& wheel = m_vehicle->getWheelInfo( i );
        wheel.m_suspensionStiffness = m_fsuspensionStiffness;
        wheel.m_wheelsDampingRelaxation = m_fsuspensionDamping;
        wheel.m_wheelsDampingCompression = m_fsuspensionCompression;
        wheel.m_frictionSlip = m_fwheelFriction;
        wheel.m_rollInfluence = m_frollInfluence;
    }

   if ( m_vehicle )
   {
      m_vehicle->resetSuspension();

      for ( int i = 0; i < m_vehicle->getNumWheels(); i++ )
      {
         //synchronize the wheels with the (interpolated) chassis worldtransform
         m_vehicle->updateWheelTransform(i,true);

            btTransform transform = m_vehicle->getWheelTransformWS( i );
            Vector3 v3Origin = ToVector3( transform.getOrigin() );
            Quaternion qRot = ToQuaternion( transform.getRotation() );

            // create wheel node
            Node *wheelNode = GetScene()->CreateChild();
            m_vpNodeWheel.Push( wheelNode );

            wheelNode->SetPosition( v3Origin );
            btWheelInfo whInfo = m_vehicle->getWheelInfo( i );
            Vector3 v3PosLS = ToVector3( whInfo.m_chassisConnectionPointCS );

            wheelNode->SetRotation( v3PosLS.x_ >= 0.0 ? Quaternion(0.0f, 0.0f, -90.0f) : Quaternion(0.0f, 0.0f, 90.0f) );
            wheelNode->SetScale(Vector3(1.0f, 0.65f, 1.0f));

            StaticModel *pWheel = wheelNode->CreateComponent<StaticModel>();
            pWheel->SetModel(cache->GetResource<Model>("Models/Cylinder.mdl"));
            pWheel->SetMaterial(cache->GetResource<Material>("Materials/Stone.xml"));
            pWheel->SetCastShadows(true);
      }
   }
}

//=============================================================================
//=============================================================================
void Vehicle::FixedUpdate(float timeStep)
{
    float newSteering = 0.0f;
    float accelerator = 0.0f;

    // Read controls
    if (controls_.buttons_ & CTRL_LEFT)
        newSteering = -1.0f;
    if (controls_.buttons_ & CTRL_RIGHT)
        newSteering = 1.0f;
    if (controls_.buttons_ & CTRL_FORWARD)
        accelerator = 1.0f;
    if (controls_.buttons_ & CTRL_BACK)
        accelerator = -0.5f;

    // When steering, wake up the wheel rigidbodies so that their orientation is updated
    if ( newSteering != 0.0f )
    {
        steering_ = steering_ * 0.95f + newSteering * 0.05f;
    }
    else
    {
        steering_ = steering_ * 0.8f + newSteering * 0.2f;
    }

    // Set front wheel angles
    m_fVehicleSteering = steering_;
    int wheelIndex = 0;
    m_vehicle->setSteeringValue(m_fVehicleSteering,wheelIndex);
    wheelIndex = 1;
    m_vehicle->setSteeringValue(m_fVehicleSteering,wheelIndex);

    if ( newSteering != 0.0f || accelerator != 0.0f )
    {
        hullBody_->Activate();
    }

    // apply forces
    m_fEngineForce = m_fmaxEngineForce * accelerator;
    m_fBreakingForce = 0.f;

    // 2x wheel drive
    for ( int i = 2; i < 4; ++i )
    {
        m_vehicle->applyEngineForce( m_fEngineForce, i );
        //m_vehicle->setBrake( m_fBreakingForce, i );
    }
}

//=============================================================================
// sync wheels for rendering
//=============================================================================
void Vehicle::PostUpdate(float )
{
    for ( int i = 0; i < m_vehicle->getNumWheels(); i++ )
    {
      m_vehicle->updateWheelTransform( i, true );

        btTransform transform = m_vehicle->getWheelTransformWS( i );
        Vector3 v3Origin = ToVector3( transform.getOrigin() );
        Quaternion qRot = ToQuaternion( transform.getRotation() );

        Node *pWheel = m_vpNodeWheel[ i ];
        pWheel->SetPosition( v3Origin );
        
        btWheelInfo whInfo = m_vehicle->getWheelInfo( i );
        Vector3 v3PosLS = ToVector3( whInfo.m_chassisConnectionPointCS );
        Quaternion qRotator = ( v3PosLS.x_ >= 0.0 ? Quaternion(0.0f, 0.0f, -90.0f) : Quaternion(0.0f, 0.0f, 90.0f) );
        pWheel->SetRotation( qRot * qRotator );
    }
}

