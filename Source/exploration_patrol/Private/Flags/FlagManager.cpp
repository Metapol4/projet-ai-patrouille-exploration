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
			if (InFlag == OutFlag)
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

void AFlagManager::CalculateVisionGroups()
{
	if (FlagActors.Num() <= 0)
		return;
	int MaxVisionTarget = 0;
	int CurrentVisionTarget = 0;
	AFlagActor* Pivot = FlagActors[0];
	CurrentVisionTarget++;
	Pivot->AddToVisibilityGroup(CurrentVisionTarget);
	// FIXME: please this sucks
	TArray<AFlagActor*> VisGroupZero;
	TArray<AFlagActor*> VisGroupOne;
	VisGroupOne.Add(Pivot);
	VisionGroups.Add(VisGroupZero);
	VisionGroups.Add(VisGroupOne);
	AFlagActor* Suspect = nullptr;

	while (CurrentVisionTarget != 0)
	{
		for (AFlagActor* Flag : FlagActors)
		{
			bool validTarget = true;
			for (int Group : Pivot->SOFlag->Segment.VisibilityGroups)
			{
				if (VisionGroups[Group].Contains(Flag))
				{
					validTarget = false;
					break;
				}
			}
			if (validTarget)
			{
				FHitResult Hit;
				FVector TraceStart = Pivot->GetActorLocation();
				FVector TraceEnd = Flag->GetActorLocation();

				FCollisionQueryParams QueryParams;
				QueryParams.AddIgnoredActor(Pivot);

				bool HasHitParent = GetWorld()->LineTraceSingleByChannel(
					Hit, TraceStart, TraceEnd, TraceChannelProperty,
					QueryParams);

				if (!HasHitParent)
				{
					bool HasHitOneChild = false;
					for (AFlagActor* FlagInGroup : VisionGroups[CurrentVisionTarget])
					{
						TraceStart = FlagInGroup->GetActorLocation();
						// trace end is the same
						FCollisionQueryParams ChildQueryParams;
						ChildQueryParams.AddIgnoredActor(FlagInGroup);
						HasHitOneChild = GetWorld()->LineTraceSingleByChannel(
							Hit, TraceStart, TraceEnd, TraceChannelProperty,
							ChildQueryParams);
					}
					if (!HasHitOneChild)
					{
						Flag->AddToVisibilityGroup(CurrentVisionTarget);
						VisionGroups[CurrentVisionTarget].Add(Flag);
					}
					else
					{
						Suspect = Flag;
					}
				}
			}
		}
		if (Suspect != nullptr)
		{
			CurrentVisionTarget++;
			Suspect->AddToVisibilityGroup(CurrentVisionTarget);
			TArray<AFlagActor*> NextVisGroup;
			NextVisGroup.Add(Suspect);
			VisionGroups.Add(NextVisGroup);
			Pivot = Suspect;
			Suspect = nullptr;
		}
		else
		{
			CurrentVisionTarget--;
			if (CurrentVisionTarget > 0)
				Pivot = VisionGroups[CurrentVisionTarget][0]; //TODO: CHECK IF CRASH
		}
	}
	if (GEngine)
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, FString::Printf(TEXT("for")));
}
