// Fill out your copyright notice in the Description page of Project Settings.


#include "Flags/FlagActor.h"

#include "Components/TextRenderComponent.h"

// Sets default values
AFlagActor::AFlagActor()
{
	PrimaryActorTick.bCanEverTick = true;
	SOFlag = CreateDefaultSubobject<USO_Flag>("Flag");
	UStaticMeshComponent* cubeMeshComponent = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Cube"));

	UStaticMesh* cubeMesh = ConstructorHelpers::FObjectFinder<UStaticMesh>(
		TEXT("StaticMesh'/Engine/BasicShapes/Cube.Cube'")).Object;

	cubeMeshComponent->SetStaticMesh(cubeMesh);

	cubeMeshComponent->SetWorldScale3D(FVector(0.25f, 0.25f, 0.25f));
	cubeMeshComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);

	RootComponent = cubeMeshComponent;

}

// Called when the game starts or when spawned
void AFlagActor::BeginPlay()
{
	Super::BeginPlay();
}

// Called every frame
void AFlagActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

void AFlagActor::DrawDebugSegmentFlag()
{
	//DrawDebugBox(GetWorld(), GetActorLocation(), FVector(25, 25, 25), FColor::Green, true);
	if (VisibilityGroupText)
		VisibilityGroupText->Destroy();
	VisibilityGroupText = GetWorld()->SpawnActor<ADebugBillboardText>(ADebugBillboardText::StaticClass(),
	                                                                  GetActorLocation() + FVector(0, 0, 50),
	                                                                  FRotator::ZeroRotator);
	VisibilityGroupText->SetText("0"); // TODO: VISIBILITY NUMBER HERE
}

void AFlagActor::AddToVisibilityGroup(int Group,bool UpdateText)
{
	SOFlag->Segment.VisibilityGroups.Add(Group);
	if(UpdateText)
		VisibilityGroupText->SetText(FString::FromInt(Group));

}
