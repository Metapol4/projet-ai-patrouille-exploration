// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "NavigationData.h"
#include "NavigationSystem.h"
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

	UPROPERTY(EditAnywhere, Category=RecastNevMesh)
	AActor* StartPoint;

	UPROPERTY(EditAnywhere, Category=RecastNevMesh)
	AActor* EndPoint;
	
	TArray<NavNodeRef> PolyArray;
	
	bool NavPoly_GetAllPolys(TArray<NavNodeRef>& Polys);
	bool TileIsValid(const ARecastNavMesh* Navmesh, int32 TileIndex) const;
	virtual void BeginPlay() override;
};
 