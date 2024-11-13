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

void AFlagManager::RefreshDebugVisionGroups(TArray<int> VisGroups)
{
	if (VisGroups.Num() <= 0)
		return;

	for (TArray<AFlagActor*> VisGroup : VisionGroups)
	{
		for (AFlagActor* ActorInGroup : VisGroup)
		{
			if (ActorInGroup)
				ActorInGroup->HideSelfAndText();
		}
	}
	for (int Group : VisGroups)
	{
		if(VisionGroups.Num() <= Group || Group < 0)
			continue;

		for (AFlagActor* ActorInGroup : VisionGroups[Group])
		{
			ActorInGroup->ShowSelfAndText();
		}
	}
}

void AFlagManager::ResetDebugVisionGroups()
{
	for (TArray<AFlagActor*> VisGroup : VisionGroups)
	{
		for (AFlagActor* ActorInGroup : VisGroup)
		{
			if (ActorInGroup)
				ActorInGroup->ShowSelfAndText();
		}
	}
}

void AFlagManager::CalculateVisionGroups()
{
	// Security For EmptyList
	if (FlagActors.Num() <= 0)
		return;

	// Declare internal variable
	int CurrentVisionTarget = 0;
	int LastSuspectVisionGroup = 0;
	AFlagActor* Pivot = FlagActors[0];

	//Start Logic at 1
	CurrentVisionTarget++;
	Pivot->AddToVisibilityGroup(CurrentVisionTarget);

	// FIXME: Void VisGroup to skip index 0
	TArray<AFlagActor*> VisGroupZero;
	VisionGroups.Add(VisGroupZero);

	// Create first vision group at index 1
	TArray<AFlagActor*> VisGroupOne;
	VisGroupOne.Add(Pivot);
	VisionGroups.Add(VisGroupOne);

	// Init suspect flag for next vision group
	AFlagActor* Suspect = nullptr;

	while (CurrentVisionTarget != 0 && CurrentVisionTarget <= 20)
	{
		//Constitute a COMPLETE vision group (no more node can be added to it)
		for (AFlagActor* Flag : FlagActors)
		{
			// Security to ensure not already in group
			bool validTarget = true;
			for (int Group : Pivot->SOFlag->Segment.VisibilityGroups)
			{
				if (VisionGroups[Group].Contains(Flag))
				{
					validTarget = false;
					break;
				}
				if (Flag->SOFlag->Segment.VisibilityGroups.Num() > 3)
				{
					validTarget = false;
					break;
				}
			}

			// Trace toward evaluated flag
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

				// Case : DIRECT LINE OF SIGHT WITH PIVOT
				if (!HasHitParent)
				{
					// Trace from all child
					bool NoClearVisionPath;
					int NbOfChildHidden = 0;
					for (AFlagActor* FlagInGroup : VisionGroups[CurrentVisionTarget])
					{
						TraceStart = FlagInGroup->GetActorLocation();
						// trace end is the same
						FCollisionQueryParams ChildQueryParams;
						ChildQueryParams.AddIgnoredActor(FlagInGroup);
						NoClearVisionPath = GetWorld()->LineTraceSingleByChannel(
							Hit, TraceStart, TraceEnd, TraceChannelProperty,
							ChildQueryParams);
						if (NoClearVisionPath)
							NbOfChildHidden++;
					}
					// Case : Vision with all member of group -> Condition satisfied
					if (NbOfChildHidden == 0)
					{
						Flag->AddToVisibilityGroup(CurrentVisionTarget);
						VisionGroups[CurrentVisionTarget].Add(Flag);
					}
					// Case : At least one child hit -> At least one more vision group exist
					else
					{
						Suspect = Flag;
						LastSuspectVisionGroup = CurrentVisionTarget;
					}
				}
			}
		}
		UE_LOG(LogTemp, Warning, TEXT("FLAG : Node in VG = %d"), VisionGroups[CurrentVisionTarget].Num());
		// Case : another vision group need to be formed
		if (Suspect != nullptr)
		{
			//CurrentVisionTarget++;
			UE_LOG(LogTemp, Warning, TEXT("FLAG : VG from %d to %d"), CurrentVisionTarget, VisionGroups.Num());
			CurrentVisionTarget = VisionGroups.Num();
			Suspect->AddToVisibilityGroup(CurrentVisionTarget);
			TArray<AFlagActor*> NextVisGroup;
			NextVisGroup.Add(Suspect);
			VisionGroups.Add(NextVisGroup);
			Pivot = Suspect;
			Suspect = nullptr;
		}
		else
		{
			if (CurrentVisionTarget == LastSuspectVisionGroup)
			{
				LastSuspectVisionGroup--;
				UE_LOG(LogTemp, Warning, TEXT("FLAG : VG from %d to %d"), CurrentVisionTarget, LastSuspectVisionGroup);
				CurrentVisionTarget--;
			}
			else
			{
				UE_LOG(LogTemp, Warning, TEXT("FLAG : VG from %d to %d"), CurrentVisionTarget, LastSuspectVisionGroup);
				CurrentVisionTarget = LastSuspectVisionGroup;
			}

			if (CurrentVisionTarget > 0)
				Pivot = VisionGroups[CurrentVisionTarget][0]; //TODO: CHECK IF CRASH
		}
	}
}
