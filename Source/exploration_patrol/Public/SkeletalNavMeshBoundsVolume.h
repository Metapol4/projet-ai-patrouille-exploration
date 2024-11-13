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
	UPROPERTY(EditAnywhere, Category=RecastNavMesh)
	ARecastNavMesh* NavMesh;

	UPROPERTY(EditAnywhere, Category=RecastNavMesh)
	AActor* StartPoint;

	UPROPERTY(EditAnywhere, Category=RecastNavMesh)
	AActor* EndPoint;

	TArray<NavNodeRef> PolyArray;

	TArray<FFlagSegment> FlagSegments;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=FlagManager)
	AFlagManager* FlagManager;

	UFUNCTION(CallInEditor, BlueprintCallable, Category=FlagManager)
	void SendFlagBatch();
	UFUNCTION(CallInEditor, BlueprintCallable, Category=FlagManager)
	void CalculateVisionGroups();
	UFUNCTION(CallInEditor, BlueprintCallable, Category=DebugVisionGroup)
	void RefreshDebugVisionGroups();
	UFUNCTION(CallInEditor, BlueprintCallable, Category=DebugVisionGroup)
	void ResetDebugVisionGroups();
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=DebugVisionGroup)
	TArray<int> DebugVisionGroups;

	bool NavPoly_GetAllPolys(TArray<NavNodeRef>& Polys);
	bool TileIsValid(const ARecastNavMesh* Navmesh, int32 TileIndex) const;
	virtual void BeginPlay() override;
};
