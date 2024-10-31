// Fill out your copyright notice in the Description page of Project Settings.


#include "Flags/DebugBillboardText.h"

#include "Components/TextRenderComponent.h"

// Sets default values
ADebugBillboardText::ADebugBillboardText()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	// Initialize the text render component
	TextComponent = CreateDefaultSubobject<UTextRenderComponent>(TEXT("TextRenderComponent"));

	// Set up text properties
	TextComponent->SetHorizontalAlignment(EHTA_Center);
	TextComponent->SetVerticalAlignment(EVRTA_TextCenter);
	TextComponent->SetTextRenderColor(FColor::Red);
	TextComponent->SetWorldSize(50.0f); // Adjust to your preference
	TextComponent->SetText(FText::FromString("Debug Text Here"));
	UMaterial* mat = ConstructorHelpers::FObjectFinder<UMaterial>(TEXT("Material'/Game/Flags/BillboardText'")).
		Object;
	if (mat)
		TextComponent->SetTextMaterial(mat);
}

// Called when the game starts or when spawned
void ADebugBillboardText::BeginPlay()
{
	Super::BeginPlay();
}

// Called every frame
void ADebugBillboardText::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

void ADebugBillboardText::SetText(FString Text)
{
	TextComponent->SetText(FText::FromString(Text));
}
