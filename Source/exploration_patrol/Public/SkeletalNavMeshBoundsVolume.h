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
	void ClearDebugLine();
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
	UFUNCTION(CallInEditor, BlueprintCallable, Category="03ControlPanelVision")
	void HighlightVisionGroupsFromList();
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="03ControlPanelVision")
	TArray<FDebugVisionGroup> DSVisionPathsToHighlight;

	/* Golden Path */
	UFUNCTION(CallInEditor, BlueprintCallable, Category="04ControlPanelGolden")
	void FindGoldenPath();
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="04ControlPanelGolden")
	float MinimumPathLenght = 6000;

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
	void CreateChallenges();
	UFUNCTION(/*CallInEditor, */BlueprintCallable, Category="06ControlPanelChallenge")
	void LegacyCreateChallengeGroups();
	UFUNCTION(/*CallInEditor, */BlueprintCallable, Category="06ControlPanelChallenge")
	void LegacySelectAllChallengeSegments();
	UFUNCTION(CallInEditor, BlueprintCallable, Category="06ControlPanelChallenge")
	void PrintChallengeGroups();
	UPROPERTY(/*EditAnywhere,*/ BlueprintReadWrite, Category="06ControlPanelChallenge")
	int PercentChanceOfMergingChallengeGroup = 50;
	UFUNCTION(CallInEditor, BlueprintCallable, Category="06ControlPanelChallenge")
	void PrintPathFromSourceId();
	UFUNCTION()
	bool AreSameChallengeGroup(int FlagA, int FlagB);
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="06ControlPanelChallenge")
	int NbOfChallenges = 3;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="06ControlPanelChallenge")
	int PathFromSourceIDDebug = 0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="06ControlPanelChallenge")
	int MinimalKLenght = 0;
	UPROPERTY()
	int KLenghtIterations;
	UPROPERTY()
	int MaxKLenghtIterationsMod = 10;

	TArray<TArray<int>> ChallengeGroups;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="06ControlPanelChallenge")
	TArray<int> ChallengePath;
	
	bool PathMoreThanKUtil(int Source, int KLenght, TArray<int> &Path, int& Goal);

	UFUNCTION(BlueprintCallable, Category="06ControlPanelChallenge")
	void LegacySelectChallengeSegments();

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
};
