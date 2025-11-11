#include "RestOrientationKeeper.h"
#include "GameFramework/Actor.h"
#include "Components/PrimitiveComponent.h"
#include "Math/UnrealMathUtility.h"
#include "Math/Quat.h"

// Sets default values
ARestOrientationKeeper::ARestOrientationKeeper()
{
    PrimaryActorTick.bCanEverTick = true;

    // Get the current orientation of the object
    UPrimitiveComponent* RootComp = Cast<UPrimitiveComponent>(GetRootComponent());
    if (RootComp)
    {
        // Access the current rotation as a quaternion (more stable for orientation)
        CurrentOrientation = RootComp->GetComponentQuat(); // This gives you a quaternion
    }
}

// Called every frame
void ARestOrientationKeeper::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    ApplyTorque(DeltaTime);
}

#pragma optimize("", off)
void ARestOrientationKeeper::ApplyTorque(float DeltaTime)
{
    // Get the current orientation of the object
    UPrimitiveComponent* RootComp = Cast<UPrimitiveComponent>(GetRootComponent());
    if (!RootComp)
    {
        return;
    }

    // Access the current rotation as a quaternion (more stable for orientation)
    CurrentOrientation = RootComp->GetComponentQuat(); // This gives you a quaternion

    // Calculate the angular difference between the current and rest orientations (quaternion difference)
    FQuat RestOrientation = TargetOrientation;
    FQuat DeltaRotation = CurrentOrientation.Inverse() * RestOrientation;

    // Convert the quaternion difference to Euler angles to represent the angular displacement
    FVector RotationAxis;
    float RotationAngle;
    DeltaRotation.ToAxisAndAngle(RotationAxis, RotationAngle);

    // Proportional (spring) torque based on the angular displacement
    FVector SpringTorque = RotationAxis * RotationAngle * SpringConstant; // Spring constant determines how strong the restoring torque is

    // Get the current angular velocity in degrees per second (for damping)
    FVector CurrentAngularVelocity = RootComp->GetPhysicsAngularVelocityInDegrees();

    // Damping torque: it should oppose the angular velocity
    FVector DampingTorque = -CurrentAngularVelocity * DumpConstant;

    // Total torque is the sum of spring force and damping force
    FVector TotalTorque = SpringTorque + DampingTorque;

    // Apply the total torque to the rigid body (convert to degrees)
    RootComp->AddTorqueInDegrees(TotalTorque, "", true);  // Apply torque in degrees

    // Optional: You can add a threshold to stop applying torque when the object is close enough to the rest orientation
    if (RotationAngle < 0.01f)  // Threshold for small rotations
    {
        // Stop applying torque when the object is very close to the rest orientation
        RootComp->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector, true);
    }
}
#pragma optimize("", on)