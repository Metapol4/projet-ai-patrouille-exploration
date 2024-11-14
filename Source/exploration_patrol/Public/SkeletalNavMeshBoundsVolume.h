// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "NavigationData.h"
#include "NavigationSystem.h"
#include "Flags/FlagManager.h"
#include "Flags/SO_Flag.h"
#include "NavMesh/NavMeshBoundsVolume.h"
#include "NavMesh/RecastNavMesh.h"
#include "SkeletalNavMeshBoundsVolume.generated.h"

/**
 * 
 */

UCLASS()
class EXPLORATION_PATROL_API ASkeletalNavMeshBoundsVolume : public ANavMeshBoundsVolume
{
	GENERATED_BODY()

public:

	TArray<NavNodeRef> PolyArray;

	TArray<FFlagSegment> FlagSegments;
	TArray<int> GoldenPath;
	int StartingFlagId, EndingFlagId;

	// Editor Helper
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="00Debugs")
	bool DNodes;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="00Debugs")
	bool DSegments;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="00Debugs")
	bool DGoldenPath;

	UFUNCTION(CallInEditor, BlueprintCallable, Category="01ControlPanel")
	void ClearDebugLine();
	UFUNCTION(CallInEditor, BlueprintCallable, Category="01ControlPanel")
	void ComputeGeometry();
	UFUNCTION(CallInEditor, BlueprintCallable, Category="01ControlPanel")
	void SendFlagBatch();
	UFUNCTION(CallInEditor, BlueprintCallable, Category="01ControlPanel")
	void CalculateVisionGroups();
	UFUNCTION(CallInEditor, BlueprintCallable, Category="01ControlPanel")
	void FindGoldenPath();
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="01ControlPanel")
	AFlagManager* FlagManager;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="02GeometryReference")
	ARecastNavMesh* NavMesh;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="02GeometryReference")
	AActor* StartPointIndicator;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="02GeometryReference")
	AActor* EndPointIndicator;

	bool NavPoly_GetAllPolys(TArray<NavNodeRef>& Polys);
	bool TileIsValid(const ARecastNavMesh* Navmesh, int32 TileIndex) const;
	virtual void BeginPlay() override;

private:
	bool TestDirectionnality(FVector StartLocation, FVector EndLocation);
	float AStarHeuristique(int FlagId, int GoalFlagId);
	TArray<int> AStarAlgorithme();
	TArray<int> AStarPathReconstructor(TArray<int> cameFrom, int Goal);
};
