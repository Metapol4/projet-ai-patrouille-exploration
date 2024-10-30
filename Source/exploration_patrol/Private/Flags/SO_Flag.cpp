// Fill out your copyright notice in the Description page of Project Settings.


#include "Flags/SO_Flag.h"

USO_Flag::USO_Flag()
{
	// FIXME: spline spawns at world 0,0,0. idk why but its not important for now
	//Spline = CreateDefaultSubobject<USplineComponent>("PatrolSpline");
}

void USO_Flag::Test()
{
	if (GEngine)
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, FString::Printf(TEXT("I AM %s"), *GetName()));
}

void USO_Flag::AddToBeginning(USO_Flag* FlagToconnect)
{
	ConnectedBeginning.Add(FlagToconnect);
}

void USO_Flag::AddToEnd(USO_Flag* FlagToconnect)
{
	ConnectedEnd.Add(FlagToconnect);
}
