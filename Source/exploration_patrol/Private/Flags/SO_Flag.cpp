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

void USO_Flag::AddToBeginConnections(USO_Flag* FlagToconnect)
{
	BeginPointConnections.Add(FlagToconnect);
	BeginPointIds.Add(FlagToconnect->Segment.id);
}

void USO_Flag::AddToEndConnections(USO_Flag* FlagToconnect)
{
	EndPointConnections.Add(FlagToconnect);
	EndPointIds.Add(FlagToconnect->Segment.id);
}

int USO_Flag::IsTouchingFlagType(EFlagType FlagType, bool MustBeGolden)
{
	int NbOfSucceeds = 0;
	for (USO_Flag* PointConnection : BeginPointConnections)
	{
		if (PointConnection->Segment.FlagType == FlagType)
		{
			if (!MustBeGolden)
				NbOfSucceeds++;

			else if (PointConnection->Segment.PathType == EFlagPathType::GOLDEN)
				NbOfSucceeds++;
		}
	}

	for (USO_Flag* PointConnection : EndPointConnections)
	{
		if (PointConnection->Segment.FlagType == FlagType)
		{
			if (!MustBeGolden)
				NbOfSucceeds++;

			if (PointConnection->Segment.PathType == EFlagPathType::GOLDEN)
				NbOfSucceeds++;
		}
	}
	return NbOfSucceeds;
}

int USO_Flag::IsTouchingPathType(EFlagPathType PathType)
{
	int NbOfSucceeds = 0;
	for (USO_Flag* PointConnection : BeginPointConnections)
	{
		if (PointConnection->Segment.PathType == PathType)
			NbOfSucceeds++;
	}
	for (USO_Flag* PointConnection : EndPointConnections)
	{
		if (PointConnection->Segment.PathType == PathType)
			NbOfSucceeds++;
	}

	return NbOfSucceeds;
}

TArray<int> USO_Flag::GetCombinedNeighbours()
{
	TArray<int> Neighbours = TArray<int>(BeginPointIds);
	Neighbours.Append(EndPointIds);
	return Neighbours;
}

void USO_Flag::AddTimeStep(int GuardPathId, int Step)
{
	for (int i = 0; i < Segment.StepGroups.Num(); i++)
	{
		if (Segment.StepGroups[i].GuardPathId == GuardPathId)
		{
			Segment.StepGroups[i].SeenAtTimeSteps.AddUnique(Step);
			return;
		}
	}
	FTimeStep NewStepGroup;
	NewStepGroup.GuardPathId = GuardPathId;
	NewStepGroup.SeenAtTimeSteps.Add(Step);
	Segment.StepGroups.Add(NewStepGroup);
}

void USO_Flag::RemoveTimeStepGroup(int GuardPathId)
{
	int IndexToRemove = -1;
	for (int i = 0; i < Segment.StepGroups.Num(); i++)
	{
		if (Segment.StepGroups[i].GuardPathId == GuardPathId)
		{
			IndexToRemove = i;
			break;
		}
	}
	UE_LOG(LogTemp, Warning, TEXT("TIMSTP: removing at %d, FLAG: %d"), IndexToRemove, Segment.id);
	if (IndexToRemove >= 0)
	{
		Segment.StepGroups.RemoveAt(IndexToRemove);
	}
}
