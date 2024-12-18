// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "SO_Flag.h"
#include "Engine/TextRenderActor.h"
#include "GameFramework/Actor.h"
#include "Flags/DebugBillboardText.h"
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
	ADebugBillboardText* VisibilityGroupText;
	void DrawDebugSegmentFlag();
	void AddToVisibilityGroup(int Group,bool UpdateText = true);
	UFUNCTION(CallInEditor, BlueprintCallable, Category="01ControlPanel")
	void SeeVisionGroup();
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="01ControlPanel")
	bool AdditiveToDebug = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="01ControlPanel")
	FColor DVisionGroupColour = FColor::Yellow;
	UFUNCTION()
	void ResetText();
	UFUNCTION()
	void SetVisionGroupText();
};
