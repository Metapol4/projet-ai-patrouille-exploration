// Fill out your copyright notice in the Description page of Project Settings.


#include "Flags/FlagActor.h"

// Sets default values
AFlagActor::AFlagActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	SOFlag = CreateDefaultSubobject<USO_Flag>("Flag");
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

