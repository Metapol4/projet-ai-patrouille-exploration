// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "FlagActor.h"
#include "SO_Flag.h"
#include "GameFramework/Actor.h"
#include "FlagManager.generated.h"

class ASkeletalNavMeshBoundsVolume;

USTRUCT(BlueprintType)
struct FDebugVisionGroup
{
	GENERATED_BODY()
	UPROPERTY(EditAnywhere)
	int id;
	UPROPERTY(EditAnywhere)
	FColor Color = FColor::Yellow;

	bool operator==(const FDebugVisionGroup& b) const
	{
		return id == b.id; // && Color == b.Color; // dont need the colour... if the id matches its the same
	}
};

UCLASS()
class EXPLORATION_PATROL_API AFlagManager : public AActor
{
	GENERATED_BODY()

public:
	// Sets default values for this actor's properties
	AFlagManager();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	//Liste de tous les segments (le id correspond à l'index)
	TArray<FFlagSegment> Segments;
	//Liste des flags actors liés aux segments (le id correspond à l'index)
	TArray<AFlagActor*> FlagActors;

	int StartingFlagId, EndingFlagId;

	UFUNCTION()
	void CreateFlagsFromSegments();
	UFUNCTION()
	void ClearAll();
	UFUNCTION()
	void LinkFlags();

public:
	TArray<TArray<AFlagActor*>> VisionGroups;
	TArray<FFlagSegment> GetSegments() const;
	TArray<AFlagActor*> GetFlagActors() const;
	void ReceiveSegmentBatch(const TArray<FFlagSegment>& SegmentBatch);
	int CurrentId;
	UFUNCTION()
	void CalculateVisionGroups();
	UFUNCTION()
	void OldCalculateVisionGroups();
	UFUNCTION()
	void NewCalculateIndividualVisionGroups();
	UFUNCTION()
	void ShowVisionGroupForActor(FDebugVisionGroup DebugInfo, bool DrawBlackLines = true);
	UFUNCTION()
	void ShowVisionGroupForActors(TArray<FDebugVisionGroup> DebugInfo);
	UFUNCTION()
	void AddToShowVisionGroupActor(FDebugVisionGroup DebugInfo);
	UFUNCTION()
	AFlagActor* GetFlagActor(int id);
	UFUNCTION()
	int GetFlagActorSize();
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Collision")
	TEnumAsByte<ECollisionChannel> TraceChannelProperty = ECC_Pawn;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Exploration")
	ASkeletalNavMeshBoundsVolume* SkeletalNavMeshBoundsVolume;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Debug")
	bool ShowVisionGroupDebugText = true;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Debug")
	float GuardVisionRange = 750;

	TArray<FDebugVisionGroup> DCurrentVisionDebug;
};
