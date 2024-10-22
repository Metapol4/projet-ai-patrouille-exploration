// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "EFlagImportance.h"
#include "SmartObjectComponent.h"
#include "SO_Flag.generated.h"

/**
 * 
 */
UCLASS()
class EXPLORATION_PATROL_API USO_Flag : public USmartObjectComponent
{
	GENERATED_BODY()
	USO_Flag();
public: // i think we'll be stuck with BP spline, this one seems severely bugged
	//UPROPERTY(BlueprintReadWrite, EditAnywhere)
	//USplineComponent* Spline;
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	EFlagImportance ImportanceLevel = EFlagImportance::NORMAL;
	UFUNCTION(BlueprintCallable)
	void Test();
};
