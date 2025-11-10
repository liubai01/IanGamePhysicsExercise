#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "TorqueGenerator.generated.h"

UCLASS()
class ATorqueGenerator : public AActor
{
    GENERATED_BODY()

public:
    ATorqueGenerator();

protected:
    virtual void BeginPlay() override;

public:
    virtual void Tick(float DeltaTime) override;

    // The target angular velocity the object should maintain
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Torque Settings")
    FVector TargetAngularVelocity;

    // The maximum torque that can be applied
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Torque Settings")
    float MaxTorque = 1000.0f;

    // The factor to reduce torque as it gets closer to target angular velocity
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Torque Settings")
    float TorqueFalloffFactor = 0.1f;

private:
    // Current angular velocity of the object
    FVector CurrentAngularVelocity;

    // Apply torque to the object
    void ApplyTorque(float DeltaTime);
};
