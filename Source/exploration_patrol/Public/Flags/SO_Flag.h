// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "FlagUtils.h"
#include "SmartObjectComponent.h"
#include "Components/SplineComponent.h"
#include "SO_Flag.generated.h"

USTRUCT(BlueprintType)
struct FFlagSegment
{
	GENERATED_BODY()
	UPROPERTY()
	uint8 id; // is this useful?
	UPROPERTY()
	FVector BeginPosition;
	UPROPERTY()
	FVector EndPosition;
	UPROPERTY()
	EFlagDirection Direction; // could also be a vector ? this seems more practical tho
};

UCLASS()
class EXPLORATION_PATROL_API USO_Flag : public USmartObjectComponent
{
	GENERATED_BODY()
	USO_Flag();
public:
	/*UPROPERTY(BlueprintReadWrite, EditAnywhere)
	USplineComponent* Spline;*/
	UFUNCTION(BlueprintCallable)
	void Test();
	UFUNCTION()
	void AddToBeginConnections(USO_Flag* FlagToconnect);
	void AddToEndConnections(USO_Flag* FlagToconnect);
	UPROPERTY(BlueprintReadWrite)
	TArray<USO_Flag*> BeginPointConnections;
	UPROPERTY(BlueprintReadWrite)
	TArray<USO_Flag*> EndPointConnections;
	UPROPERTY(BlueprintReadWrite)
	EFlagType FlagType;
	UPROPERTY(BlueprintReadWrite)
	FFlagSegment Segment;
	
};
