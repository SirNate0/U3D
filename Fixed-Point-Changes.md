# Add fixed point btScalar

Source/ThirdParty/Bullet/src/LinearMath/btScalar.h
Source/Samples/11_Physics/Physics.cpp
	^ MM_FREE so the mouse shows during debugging.
	^ Reduced scale before I changed `btVector3::length()` to use `fpm::hypot()`

## Check for things that would have given NaN before. Not sure if it is correct, but returning NaN actually seems like it should be an error to me.
Source/ThirdParty/Bullet/src/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.cpp
    ^^ Also cast to double
# Setup

- Copy the [fpm](https://github.com/MikeLankamp/fpm/tree/master/include/fpm) includes into `Source/ThirdParty/Bullet/src/LinearMath/`
- Rename math.hpp -> fixedmath.hpp and ios.hpp -> fixedios.hpp
- Apply the Changes.patch

Changes are summarized below:

## Add fixed point btScalar

    Source/ThirdParty/Bullet/src/LinearMath/btScalar.h
    Source/Samples/11_Physics/Physics.cpp
      ^ MM_FREE so the mouse shows during debugging.
      ^ Reduced scale before I changed `btVector3::length()` to use `fpm::hypot()`

## Check for things that would have given NaN before. Not sure if it is correct, but returning NaN actually seems like it should be an error to me.

    Source/ThirdParty/Bullet/src/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.cpp
        ^^ Also cast to double


## Use btFabs, btSqrt, btFloor (new function)

    Source/ThirdParty/Bullet/src/BulletDynamics/Character/btKinematicCharacterController.cpp
    Source/ThirdParty/Bullet/src/BulletDynamics/ConstraintSolver/btConeTwistConstraint.cpp
    Source/ThirdParty/Bullet/src/BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.cpp
    Source/ThirdParty/Bullet/src/BulletDynamics/MLCPSolvers/btLemkeAlgorithm.cpp
        ^ Also, cast to btScalar explicitly
    Source/ThirdParty/Bullet/src/BulletSoftBody/btDeformableBackwardEulerObjective.cpp
    Source/ThirdParty/Bullet/src/BulletSoftBody/btDeformableBodySolver.cpp
    Source/ThirdParty/Bullet/src/BulletSoftBody/btSoftBodyInternals.h
    Source/ThirdParty/Bullet/src/BulletSoftBody/poly34.c
        ^ FIXME: I have a sqrt in there rather than btSqrt. Though it seemed to work, so...?
    Source/ThirdParty/Bullet/src/LinearMath/btConvexHull.cpp
    Source/ThirdParty/Bullet/src/LinearMath/btReducedVector.cpp
    Source/ThirdParty/Bullet/src/BulletDynamics/Featherstone/btMultiBody.cpp
        ^ Make cast to int explicit. Not sure why it isn't rounding, or using an int at the start...
        ^ Also, serialization.

## iostream include for fpm::fixed

    Source/ThirdParty/Bullet/src/BulletSoftBody/btSoftBodyHelpers.cpp
    Source/ThirdParty/Bullet/src/LinearMath/btModifiedGramSchmidt.h
        ^ Also btFabs

## btScalar computer for ConvexHull

    Source/ThirdParty/Bullet/src/LinearMath/btConvexHullComputer.cpp
    Source/ThirdParty/Bullet/src/LinearMath/btConvexHullComputer.h

## fixed point btMatrixXq

    Source/ThirdParty/Bullet/src/LinearMath/btMatrixX.h

## Serialization

    Source/ThirdParty/Bullet/src/BulletCollision/BroadphaseCollision/btQuantizedBvh.h
    Source/ThirdParty/Bullet/src/BulletCollision/CollisionDispatch/btCollisionObject.h
    Source/ThirdParty/Bullet/src/BulletCollision/CollisionDispatch/btCollisionWorldImporter.cpp
    Source/ThirdParty/Bullet/src/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.cpp
    Source/ThirdParty/Bullet/src/BulletCollision/CollisionShapes/btConvexHullShape.cpp
    Source/ThirdParty/Bullet/src/BulletCollision/NarrowPhaseCollision/btPersistentManifold.cpp
    Source/ThirdParty/Bullet/src/BulletDynamics/ConstraintSolver/btConeTwistConstraint.h
    Source/ThirdParty/Bullet/src/BulletDynamics/ConstraintSolver/btGearConstraint.h
    Source/ThirdParty/Bullet/src/BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h
    Source/ThirdParty/Bullet/src/BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h
    Source/ThirdParty/Bullet/src/BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.h
    Source/ThirdParty/Bullet/src/BulletDynamics/ConstraintSolver/btHingeConstraint.h
    Source/ThirdParty/Bullet/src/BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h
    Source/ThirdParty/Bullet/src/BulletDynamics/ConstraintSolver/btSliderConstraint.h
    Source/ThirdParty/Bullet/src/BulletDynamics/ConstraintSolver/btTypedConstraint.cpp
    Source/ThirdParty/Bullet/src/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.cpp
    Source/ThirdParty/Bullet/src/BulletDynamics/Dynamics/btRigidBody.cpp
    Source/ThirdParty/Bullet/src/BulletSoftBody/btSoftBody.cpp
    Source/ThirdParty/Bullet/src/LinearMath/btMatrix3x3.h
    Source/ThirdParty/Bullet/src/LinearMath/btQuaternion.h
    Source/ThirdParty/Bullet/src/LinearMath/btTransform.h
    Source/ThirdParty/Bullet/src/LinearMath/btVector3.h
        ^ ALSO hypot function

## Make cast to double explicit.

    Source/ThirdParty/Bullet/src/BulletCollision/CollisionShapes/btMiniSDF.cpp
    Source/ThirdParty/Bullet/src/BulletSoftBody/btDeformableGravityForce.h
    Source/ThirdParty/Bullet/src/BulletSoftBody/btDeformableLagrangianForce.h
        ^ This one, where I needed to use std::sqrt instead of btSqrt, makes me think it needs a btHighPrecision type as well, which would default to double.
    Source/ThirdParty/Bullet/src/BulletSoftBody/btDeformableLinearElasticityForce.h
    Source/ThirdParty/Bullet/src/BulletSoftBody/btDeformableMassSpringForce.h
    Source/ThirdParty/Bullet/src/BulletSoftBody/btDeformableMousePickingForce.h
    Source/ThirdParty/Bullet/src/BulletSoftBody/btDeformableMultiBodyConstraintSolver.cpp
    Source/ThirdParty/Bullet/src/BulletSoftBody/btDeformableNeoHookeanForce.h

## Make cast from btScalar to float explicit

    Source/Urho3D/Physics/PhysicsUtils.h
    Source/Urho3D/Physics/PhysicsWorld.cpp
    Source/Urho3D/Physics/RaycastVehicle.cpp
    Source/Urho3D/Physics/RigidBody.cpp

## Convert doubles to btScalar

    Source/ThirdParty/Bullet/src/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.cpp

## Ternary

    Source/ThirdParty/Bullet/src/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.cpp
    Source/ThirdParty/Bullet/src/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.cpp
    Source/ThirdParty/Bullet/src/BulletCollision/Gimpact/gim_basic_geometry_operations.h // GIM_CLAMP

### Make BT_CLAMP a function so the cast to btScalar happens

    Source/ThirdParty/Bullet/src/BulletCollision/Gimpact/btGeometryOperations.h

## Undo Me

    Source/ThirdParty/Bullet/src/BulletCollision/CollisionDispatch/btCollisionWorld.cpp
        - Debugging the large lengths before `fpm::hypot()`
    
## FIX ME

    Source/ThirdParty/Bullet/src/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.cpp
        - infinite is TOO LARGE FOR Q16.16

    
## UNRELATED

	bin/Data/Scripts/Editor/EditorView.as
