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

USTRUCT(BlueprintType)
struct FNeighbors
{
	GENERATED_BODY()
	UPROPERTY(EditAnywhere)
	int ID = 0;
	UPROPERTY(EditAnywhere)
	float SortValue = 0;
};

UCLASS()
class EXPLORATION_PATROL_API ASkeletalNavMeshBoundsVolume : public ANavMeshBoundsVolume
{
	GENERATED_BODY()

public:
	UFUNCTION(CallInEditor, BlueprintCallable, Category="01ControlPanel")
	void a01ComputeGeometry();
	UFUNCTION(CallInEditor, BlueprintCallable, Category="01ControlPanel")
	void a02SendFlagBatch();
	UFUNCTION(CallInEditor, BlueprintCallable, Category="01ControlPanel")
	void a03CalculateVisionGroups();
	UFUNCTION(CallInEditor, BlueprintCallable, Category="01ControlPanel")
	void a04FindSafeSegments();
	UFUNCTION(CallInEditor, BlueprintCallable, Category="01ControlPanel")
	void a05CalculateDirectionality();
	UFUNCTION(CallInEditor, BlueprintCallable, Category="01ControlPanel")
	void a06GenerateGuardPaths();
	UFUNCTION(CallInEditor, BlueprintCallable, Category="01ControlPanel")
	void GenerateAll();

	/* Poly */
	UFUNCTION()
	void ComputeGeometry();
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
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="00Debugs")
	bool FlushDebugLinesWhenGeneratingNewOnes = true;

	/* Utils */
	UFUNCTION(CallInEditor, BlueprintCallable, Category="00Debugs")
	void ClearDebugLine();
	

	/* Flags */
	UFUNCTION()
	void SendFlagBatch();
	UFUNCTION(Category="02ControlPanelFlags")
	void ResetAllFlagTypes();

	TArray<FFlagSegment> FlagSegments;

	/* Vision */
	UFUNCTION()
	void CalculateVisionGroups();
	UFUNCTION(CallInEditor, BlueprintCallable, Category="03Vision")
	void HighlightVisionGroupsFromList();
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="03Vision")
	TArray<FDebugVisionGroup> DSVisionPathsToHighlight;

	/* Legacy Golden Path */
	void FindGoldenPath();
	float MinimumPathLenght = 6000;
	TArray<int> GoldenPath;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	int GoldenStartingFlagId;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	int GoldenEndingFlagId;
	TArray<int> GoldenPathCopy;


	/* Safe */
	UFUNCTION()
	void FindSafeSegments();
	void FindBeginAndEndFlags();
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="04Safe")
	float PercentageSafeSegment = 5.0f;


	/* Directionality */
	void CalculateDirectionnalityButton();

	void CalculateDirectionnality(EFlagType FlagType);

	EFlagDirection GetAdditiveFlagDirection(EFlagDirection WantedDirection, EFlagDirection CurrentDirection);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="05Directionality")
	float AngleTolerance = 45;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="05Directionality")
	EFlagType DebugFlagTypeDirection = EFlagType::SAFE;

	/* Guards */
	UFUNCTION(CallInEditor, BlueprintCallable, Category="06Guards")
	void GenerateOneGuardPath();
	UFUNCTION()
	void GenerateGuardPathsUntilFail();
	UFUNCTION(CallInEditor, BlueprintCallable, Category="06Guards")
	void SimulateCurrentConfiguration();
	UFUNCTION(CallInEditor, BlueprintCallable, Category="06Guards")
	void FindPlayerPathEditor();
	UFUNCTION(Category="06Guards")
	bool FindPlayerPath();
	UFUNCTION(CallInEditor, BlueprintCallable, Category="06Guards")
	void DrawChallengePaths();
	UFUNCTION(CallInEditor, BlueprintCallable, Category="06Guards")
	void DrawLatestPlayerPath();
	UFUNCTION()
	void CalculateGuardPathVisionTimeSteps();
	bool GuardPathMoreThanKGenerator(int Source, int KLenght, FVector2d VERT, TArray<int>& Path, int& End);
	UFUNCTION()
	TArray<int> PopChallengePath();
	UFUNCTION()
	void EmptyChallengePath();
	UFUNCTION()
	void DrawNextStep(int MaxStep);
	UFUNCTION()
	void CalculateNeighboursForTimeStep(AFlagActor* SelfFlag, FVector Direction, int Step, int GuardPathId);
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="06Guards")
	float LinearWeight = 1.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="06Guards")
	float ExplorationWeight = 1.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="06Guards")
	float PercentageRandomStartingPointSelection = 50;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="06Guards")
	FColor DGuardPathColor = FColor::Red;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="06Guards")
	bool UseDGuardPathColor = false;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="06Guards")
	int KLengthTarget = 2500;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="06Guards")
	int MaxGuardNb = 10;
	UPROPERTY()
	int KLenghtIterations;
	UPROPERTY()
	int GuardKLenghtIterations;
	UPROPERTY()
	int PlayerKLenghtIterations;
	UPROPERTY()
	int SimulationIterations;
	UPROPERTY()
	int MaxKLenghtIterationsMod = 10;

	/* Legacy Challenge */
	void CreateChallenges();
	void LegacyCreateChallengeGroups();
	void LegacySelectAllChallengeSegments();
	void PrintChallengeGroups();
	void ConstructChallengePaths();
	bool AreSameChallengeGroup(int FlagA, int FlagB);
	void FilterAndSortOutAltAndBidirectional(TArray<AFlagActor*>& TemporaryFlagList);
	bool CreateSourceNeighbourFromFilters(AFlagActor* SourceFlag, const TArray<int> Path,
	                                      TArray<FNeighbors>& OutSourceNeighbour);
	void LegacySelectChallengeSegments();
	void SortByMostDesirableRatio(TArray<FNeighbors>& OutSourceNeighbors, AFlagActor* Source);
	int PercentChanceOfMergingChallengeGroup = 50;
	int NbOfChallenges = 3;
	TArray<TArray<int>> ChallengeGroups;
	TArray<TArray<int>> ChallengePath;

	/* Player Path Generation */
	bool PlayerPathMoreThanKUntilGoal(int Source, int Step, TArray<int>& Path, int& Goal);
	TArray<int> PlayerPath;


	bool PathMoreThanKUtil(int Source, int KLenght, TArray<int>& Path, int& Goal);


	/* GeometryRefs */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="07GeometryReference")
	ARecastNavMesh* NavMesh;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="07GeometryReference")
	AActor* StartPointIndicator;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="07GeometryReference")
	AActor* EndPointIndicator;

	

	virtual void BeginPlay() override;

private:
	bool TestDirectionnality(FVector StartLocation, FVector EndLocation);
	float AStarHeuristique(int FlagId, int GoalFlagId);
	float AStarAlgorithme(int StartFlagID, int EndFlagID, TArray<int>& BestPath);
	float AStarPathReconstructor(TArray<int> CameFrom, int Start, int Goal, TArray<int>& ReconstructedPath);

	TArray<int> FlagCurrentlySeen;
	void AddAngleToSortValue(TArray<FNeighbors>& OutSourceNeighbors, AFlagActor* Source);
	void AddExplorationBonusToSortValue(TArray<FNeighbors>& OutSourceNeighbors, AFlagActor* Source,
	                                    TArray<int>& CameFrom);
	void AddExitBonusToSortValue(TArray<FNeighbors>& OutSourceNeighbors);
	void DebugDirectionality(int FlagID);

	void FilterAllAlreadyInUse(TArray<AFlagActor*>& TemporaryFlagList);

	FTimerHandle SimulationTimer;
	FTimerDelegate SimulationDelegate;
	float SimulationTimeStep = 0.75f;
};
