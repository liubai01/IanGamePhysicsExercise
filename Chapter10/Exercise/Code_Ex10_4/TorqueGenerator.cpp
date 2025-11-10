#include "TorqueGenerator.h"
#include "GameFramework/Actor.h"
#include "Components/PrimitiveComponent.h"

#pragma optimize("", off)
// Sets default values
ATorqueGenerator::ATorqueGenerator()
{
    // Set this actor to call Tick() every frame.
    PrimaryActorTick.bCanEverTick = true;

    // Initialize the current angular velocity to zero
    CurrentAngularVelocity = FVector::ZeroVector;
}

// Called when the game starts or when spawned
void ATorqueGenerator::BeginPlay()
{
    Super::BeginPlay();
}

// Called every frame
void ATorqueGenerator::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    // Get the current angular velocity of the object (could be a physics body)
    UPrimitiveComponent* RootComp = Cast<UPrimitiveComponent>(GetRootComponent());
    if (RootComp)
    {
        // Access the angular velocity
        CurrentAngularVelocity = RootComp->GetPhysicsAngularVelocityInDegrees();
        
        // Apply the torque based on the difference between target and current angular velocity
        ApplyTorque(DeltaTime);
    }
}

// Apply torque to the object to correct angular velocity
void ATorqueGenerator::ApplyTorque(float DeltaTime)
{
	float EuclideanDist = FVector::Dist(CurrentAngularVelocity, TargetAngularVelocity);
    
    // Apply a falloff factor to decrease torque as the difference decreases
    float TorqueStrength = FMath::Clamp(EuclideanDist * MaxTorque * TorqueFalloffFactor, -MaxTorque, MaxTorque);

    // If there is a significant difference, apply torque
    UPrimitiveComponent* RootComp = Cast<UPrimitiveComponent>(GetRootComponent());
    if (RootComp)
    {
        // Calculate the torque vector to apply
        FVector TorqueDirection = (TargetAngularVelocity - CurrentAngularVelocity).GetSafeNormal();
        FVector Torque = TorqueDirection * FMath::Abs(TorqueStrength);

        // Apply the torque to the object
        RootComp->AddTorqueInDegrees(Torque, "", true);
    }
}

#pragma optimize("", on)