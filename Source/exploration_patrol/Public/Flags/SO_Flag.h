// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "FlagUtils.h"
#include "SmartObjectComponent.h"
#include "Components/SplineComponent.h"
#include "SO_Flag.generated.h"

USTRUCT(BlueprintType)
struct FTimeStep
{
	GENERATED_BODY()
	UPROPERTY(EditAnywhere)
	int GuardPathId;
	UPROPERTY(EditAnywhere)
	TArray<int> SeenAtTimeSteps;
};

USTRUCT(BlueprintType)
struct FFlagSegment
{
	GENERATED_BODY()
	UPROPERTY(EditAnywhere)
	int id;
	UPROPERTY(EditAnywhere)
	FVector BeginPosition;
	UPROPERTY(EditAnywhere)
	FVector EndPosition;
	UPROPERTY(EditAnywhere)
	float Lenght;
	UPROPERTY(EditAnywhere)
	float Area;
	UPROPERTY(EditAnywhere)
	float VisionArea;
	UPROPERTY(EditAnywhere)
	EFlagDirection Direction; // could also be a vector ? this seems more practical tho
	UPROPERTY(EditAnywhere)
	TArray<int> VisibilityGroups;
	UPROPERTY(EditAnywhere)
	TArray<FTimeStep> StepGroups;
	UPROPERTY(EditAnywhere)
	EFlagType FlagType = EFlagType::NONE;
	UPROPERTY(EditAnywhere)
	EFlagPathType PathType = EFlagPathType::NONE;
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
	UFUNCTION()
	int IsTouchingFlagType(EFlagType FlagType, bool MustBeGolden = false);
	UFUNCTION()
	int IsTouchingPathType(EFlagPathType PathType);
	UFUNCTION()
	TArray<int> GetCombinedNeighbours();
	UFUNCTION()
	void AddTimeStep(int GuardPathId, int Step);
	UFUNCTION()
	void RemoveTimeStepGroup(int GuardPathId);
	UPROPERTY()
	TArray<USO_Flag*> BeginPointConnections;
	UPROPERTY()
	TArray<USO_Flag*> EndPointConnections;
	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	TArray<int> BeginPointIds;
	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	TArray<int> EndPointIds;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FFlagSegment Segment;
	
};
