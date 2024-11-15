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
	int GoldenStartingFlagId, GoldenEndingFlagId;
	TArray<int> GoldenPathCopy;
	// Editor Helper
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="00Debugs")
	bool DNodes;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="00Debugs")
	bool DSegments;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="00Debugs")
	bool DGoldenPath;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="00Debugs")
	bool DDirectionality;

	UFUNCTION(CallInEditor, BlueprintCallable, Category="01ControlPanelUtils")
	void ClearDebugLine();
	UFUNCTION(CallInEditor, BlueprintCallable, Category="01ControlPanelUtils")
	void ComputeGeometry();
	UFUNCTION(CallInEditor, BlueprintCallable, Category="02ControlPanelFlags")
	void ResetAllFlagTypes();
	UFUNCTION(CallInEditor, BlueprintCallable, Category="02ControlPanelFlags")
	void SendFlagBatch();
	UFUNCTION(CallInEditor, BlueprintCallable, Category="03ControlPanelVision")
	void CalculateVisionGroups();
	UFUNCTION(CallInEditor, BlueprintCallable, Category="04ControlPanelGolden")
	void FindGoldenPath();
	UFUNCTION(CallInEditor, BlueprintCallable, Category="05ControlPanelDirectionality")
	void CalculateDirectionnality();
	UFUNCTION(CallInEditor, BlueprintCallable, Category="06ControlPanelChallenge")
	void SelectAllChallengeSegments();
	UFUNCTION(CallInEditor, BlueprintCallable, Category="06ControlPanelChallenge")
	void SelectChallengeSegments();
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="04ControlPanelGolden")
	float MinimumPathLenght = 4000;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="05ControlPanelDirectionality")
	float AngleTolerance = 45;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="02ControlPanelFlags")
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
	float AStarAlgorithme(int StartFlagID, int EndFlagID, TArray<int>& BestPath);
	float AStarPathReconstructor(TArray<int> CameFrom, int Start, int Goal, TArray<int>& ReconstructedPath);
	void DebugDirectionality(int FlagID);
};
