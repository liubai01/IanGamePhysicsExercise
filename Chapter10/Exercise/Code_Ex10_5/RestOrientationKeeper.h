// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RestOrientationKeeper.generated.h"

UCLASS()
class MSP_API ARestOrientationKeeper : public AActor
{
	GENERATED_BODY()

public:
	// Sets default values for this actor's properties
	ARestOrientationKeeper();

protected:
	virtual void Tick(float DeltaTime) override;

	// The target angular velocity the object should maintain
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Torque Settings")
	FQuat TargetOrientation;

	// The maximum torque that can be applied
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Torque Settings")
	float MaxTorque = 1000.0f;

	// The factor to reduce torque as it gets closer to target angular velocity
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Torque Settings")
	float TorqueFalloffFactor = 0.1f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Torque Settings")
	float SpringConstant = 10.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Torque Settings")
	float DumpConstant = 10.0f;
private:
	// Current angular velocity of the object
	FQuat CurrentOrientation;

	// Apply torque to the object
	void ApplyTorque(float DeltaTime);
};
