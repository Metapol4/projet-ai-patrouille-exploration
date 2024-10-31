// Fill out your copyright notice in the Description page of Project Settings.


#include "Flags/FlagActor.h"

// Sets default values
AFlagActor::AFlagActor()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	SOFlag = CreateDefaultSubobject<USO_Flag>("Flag");
	// Create a static mesh component
	UStaticMeshComponent* cubeMeshComponent = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Cube"));

	// Load the Cube mesh
	UStaticMesh* cubeMesh = ConstructorHelpers::FObjectFinder<UStaticMesh>(TEXT("StaticMesh'/Engine/BasicShapes/Cube.Cube'")).Object;

	// Set the component's mesh
	cubeMeshComponent->SetStaticMesh(cubeMesh);

	cubeMeshComponent->SetWorldScale3D(FVector(0.25f,0.25f,0.25f));

	// Set as root component
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
}
