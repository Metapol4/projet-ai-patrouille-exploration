// Fill out your copyright notice in the Description page of Project Settings.


#include "Flags/FlagManager.h"

#include "VectorTypes.h"
#include "exploration_patrol/exploration_patrolCharacter.h"
#include "Kismet/GameplayStatics.h"


// Sets default values
AFlagManager::AFlagManager()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void AFlagManager::BeginPlay()
{
	Super::BeginPlay();
}


void AFlagManager::CreateFlagsFromSegments()
{
	for (FFlagSegment Flag : Segments)
	{
		AFlagActor* FlagActor = Cast<AFlagActor>(GetWorld()->SpawnActor(AFlagActor::StaticClass()));
		FlagActor->SOFlag->Segment = Flag;
		FlagActor->SOFlag->Segment.id = CurrentId;
		CurrentId++;
		FVector MiddlePos = (Flag.BeginPosition + Flag.EndPosition) / 2;
		FlagActor->SetActorLocation(MiddlePos);
		FlagActor->DrawDebugSegmentFlag();
		FlagActors.Add(FlagActor);
	}
}

void AFlagManager::ClearAll()
{
	Segments.Empty();
	for (AFlagActor* Flag : FlagActors)
	{
		Flag->Destroy();
	}
}

void AFlagManager::LinkFlags()
{
	for (AFlagActor* InFlag : FlagActors) // TODO: FIXME: is this too ugly? this is version 1
	{
		for (AFlagActor* OutFlag : FlagActors)
		{
			if(InFlag == OutFlag)
				continue;
			//begin
			if (UE::Geometry::Distance(InFlag->SOFlag->Segment.BeginPosition, OutFlag->SOFlag->Segment.BeginPosition) <
				1.0f)
			{
				InFlag->SOFlag->AddToBeginConnections(OutFlag->SOFlag);
			}
			if (UE::Geometry::Distance(InFlag->SOFlag->Segment.BeginPosition, OutFlag->SOFlag->Segment.EndPosition) <
				1.0f)
			{
				InFlag->SOFlag->AddToBeginConnections(OutFlag->SOFlag);
			}
			//end
			if (UE::Geometry::Distance(InFlag->SOFlag->Segment.EndPosition, OutFlag->SOFlag->Segment.BeginPosition) <
				1.0f)
			{
				InFlag->SOFlag->AddToEndConnections(OutFlag->SOFlag);
			}
			if (UE::Geometry::Distance(InFlag->SOFlag->Segment.EndPosition, OutFlag->SOFlag->Segment.EndPosition) <
				1.0f)
			{
				InFlag->SOFlag->AddToEndConnections(OutFlag->SOFlag);
			}
		}
	}
}

TArray<FFlagSegment> AFlagManager::GetSegments() const
{
	return Segments;
}

TArray<AFlagActor*> AFlagManager::GetFlagActors() const
{
	return FlagActors;
}


void AFlagManager::ReceiveSegmentBatch(const TArray<FFlagSegment>& SegmentBatch)
{
	ClearAll();
	Segments = SegmentBatch;
	CreateFlagsFromSegments();
	LinkFlags();
}

void AFlagManager::TestSegmentBatch()
{
	/*TArray<AActor*> TestSubjects;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), Aexploration_patrolCharacter::StaticClass(), TestSubjects);
	for (AActor* TestSubject : TestSubjects)
	{
		
	}*/
}
