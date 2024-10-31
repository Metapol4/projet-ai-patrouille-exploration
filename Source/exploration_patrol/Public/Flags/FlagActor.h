// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "SO_Flag.h"
#include "GameFramework/Actor.h"
#include "FlagActor.generated.h"

UCLASS()
class EXPLORATION_PATROL_API AFlagActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AFlagActor();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;
	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	USO_Flag* SOFlag;

	void DrawDebugSegmentFlag();

};
