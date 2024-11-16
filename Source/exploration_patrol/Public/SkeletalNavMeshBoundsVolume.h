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
	/* Poly */
	TArray<NavNodeRef> PolyArray;
	bool NavPoly_GetAllPolys(TArray<NavNodeRef>& Polys);
	bool TileIsValid(const ARecastNavMesh* Navmesh, int32 TileIndex) const;

	/* Debug */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="00Debugs")
	bool DNodes;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="00Debugs")
	bool DSegments;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="00Debugs")
	bool DGoldenPath;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="00Debugs")
	bool DDirectionality;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="00Debugs")
	AFlagManager* FlagManager;

	/* Utils */
	UFUNCTION(CallInEditor, BlueprintCallable, Category="01ControlPanelUtils")
	void ClearDebugLineConsole();
	UFUNCTION(CallInEditor, BlueprintCallable, Category="01ControlPanelUtils")
	void ComputeGeometry();
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="01ControlPanelUtils")
	bool FlushDebugLinesWhenGeneratingNewOnes = true;

	/* Flags */
	UFUNCTION(CallInEditor, BlueprintCallable, Category="02ControlPanelFlags")
	void SendFlagBatch();
	UFUNCTION(CallInEditor, BlueprintCallable, Category="02ControlPanelFlags")
	void ResetAllFlagTypes();

	TArray<FFlagSegment> FlagSegments;

	/* Vision */
	UFUNCTION(CallInEditor, BlueprintCallable, Category="03ControlPanelVision")
	void CalculateVisionGroups();

	/* Golden Path */
	UFUNCTION(CallInEditor, BlueprintCallable, Category="04ControlPanelGolden")
	void FindGoldenPath();
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="04ControlPanelGolden")
	float MinimumPathLenght = 4000;

	TArray<int> GoldenPath;
	int GoldenStartingFlagId, GoldenEndingFlagId;
	TArray<int> GoldenPathCopy;

	/* Directionality */
	UFUNCTION(CallInEditor, BlueprintCallable, Category="05ControlPanelDirectionality")
	void CalculateDirectionnality();
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="05ControlPanelDirectionality")
	float AngleTolerance = 45;

	/* Challenge */
	UFUNCTION(CallInEditor, BlueprintCallable, Category="06ControlPanelChallenge")
	void CreateChallengeGroups();
	UFUNCTION(CallInEditor, BlueprintCallable, Category="06ControlPanelChallenge")
	void SelectAllChallengeSegments();
	UFUNCTION(CallInEditor, BlueprintCallable, Category="06ControlPanelChallenge")
	void PrintChallengeGroups();
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="06ControlPanelChallenge")
	int PercentChanceOfMergingChallengeGroup = 50;

	TArray<TArray<int>> ChallengeGroups;

	UFUNCTION(BlueprintCallable, Category="06ControlPanelChallenge")
	void SelectChallengeSegments();

	/* GeometryRefs */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="07GeometryReference")
	ARecastNavMesh* NavMesh;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="07GeometryReference")
	AActor* StartPointIndicator;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="07GeometryReference")
	AActor* EndPointIndicator;

	UFUNCTION(CallInEditor, BlueprintCallable, Category="08UserFriendlyButtons")
	void GenerateAll();

	virtual void BeginPlay() override;

private:
	bool TestDirectionnality(FVector StartLocation, FVector EndLocation);
	float AStarHeuristique(int FlagId, int GoalFlagId);
	float AStarAlgorithme(int StartFlagID, int EndFlagID, TArray<int>& BestPath);
	float AStarPathReconstructor(TArray<int> CameFrom, int Start, int Goal, TArray<int>& ReconstructedPath);
	void DebugDirectionality(int FlagID);
	void ClearDebugLine();
};
