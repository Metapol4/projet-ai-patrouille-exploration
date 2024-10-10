// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "SmartObjectComponent.h"
#include "Components/SplineComponent.h"
#include "SO_Flag.generated.h"

/**
 * 
 */
UCLASS()
class EXPLORATION_PATROL_API USO_Flag : public USmartObjectComponent
{
	GENERATED_BODY()
	USO_Flag();
public:
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	USplineComponent* Spline;
	UFUNCTION(BlueprintCallable)
	void Test();
};
