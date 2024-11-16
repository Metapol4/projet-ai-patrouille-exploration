// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/BillboardComponent.h"
#include "Engine/TextRenderActor.h"
#include "GameFramework/Actor.h"
#include "DebugBillboardText.generated.h"

UCLASS()
class EXPLORATION_PATROL_API ADebugBillboardText : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ADebugBillboardText();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	ATextRenderActor* TextRenderActor;
	UTextRenderComponent* TextComponent;

	void SetText(FString Text);
	void SetTextColor(FColor Color);
	void AddText(FString Text);
	void ResetText();

};
