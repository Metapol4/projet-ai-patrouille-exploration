// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "FlagActor.h"
#include "SO_Flag.h"
#include "GameFramework/Actor.h"
#include "FlagManager.generated.h"

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
	TArray<FFlagSegment> Segments;
	TArray<AFlagActor*> FlagActors;
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
	UFUNCTION(CallInEditor, BlueprintCallable)
	void TestSegmentBatch();
	int CurrentId;
	UFUNCTION()
	void CalculateVisionGroups();
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Collision")
	TEnumAsByte<ECollisionChannel> TraceChannelProperty = ECC_Pawn;
};
