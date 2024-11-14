// Fill out your copyright notice in the Description page of Project Settings.


#include "SkeletalNavMeshBoundsVolume.h"

#include "DataTypeUtils.h"
#include "VectorTypes.h"
#include "VectorUtil.h"

void ASkeletalNavMeshBoundsVolume::ClearDebugLine()
{
	FlushPersistentDebugLines(GetWorld());
}

void ASkeletalNavMeshBoundsVolume::ComputeGeometry()
{
	int CurrentID = 0;
	FlushPersistentDebugLines(GetWorld());
	if (!NavMesh) return;
	if (!StartPointIndicator) return;
	if (!EndPointIndicator) return;
	FlagSegments.Empty();

	NavPoly_GetAllPolys(PolyArray);
	for (NavNodeRef Polys : PolyArray)
	{
		FVector SegmentBeginPoint;
		NavMesh->GetPolyCenter(Polys, SegmentBeginPoint);

		if (DNodes)
		{
			DrawDebugSphere(
			GetWorld(),
			SegmentBeginPoint,
			5,
			12,
			FColor::Cyan,
			true,
			300);
		}

		TArray<NavNodeRef> NeighborsNodes;
		if (NavMesh->GetPolyNeighbors(Polys, NeighborsNodes))
		{
			FVector SegmentEndPoint;
			for (NavNodeRef Node : NeighborsNodes)
			{
				NavMesh->GetPolyCenter(Node, SegmentEndPoint);
				if(TestDirectionnality(SegmentBeginPoint, SegmentEndPoint))
				{
					FFlagSegment CurrentSegment;
					CurrentSegment.id = CurrentID;
					CurrentID++;

					CurrentSegment.BeginPosition = SegmentBeginPoint;
					CurrentSegment.EndPosition = SegmentEndPoint;
					CurrentSegment.Direction = EFlagDirection::NONE;
					CurrentSegment.Lenght = UE::Geometry::Distance(SegmentBeginPoint, SegmentEndPoint);
					FlagSegments.Add(CurrentSegment);

					if (DSegments)
					{
						FVector AdjustedLocation = SegmentBeginPoint + (SegmentEndPoint - SegmentBeginPoint) * 0.9f;
						DrawDebugDirectionalArrow(
								GetWorld(),
								SegmentBeginPoint,
								AdjustedLocation,
								500,
								FColor::Red,
								true,
								300
							);
					}
				}
			}
		}
	}
}

void ASkeletalNavMeshBoundsVolume::SendFlagBatch()
{
	if (FlagManager)
		FlagManager->ReceiveSegmentBatch(FlagSegments);
}

void ASkeletalNavMeshBoundsVolume::CalculateVisionGroups()
{
	FlagManager->CalculateVisionGroups();
}

void ASkeletalNavMeshBoundsVolume::FindGoldenPath()
{
	int NbOfFlag = FlagManager->GetFlagActorSize();
	if (NbOfFlag == -1)
		return;
	
	//Find Starting and Ending Node
	float minDistanceToStart = INFINITY;
	float minDistanceToEnd = INFINITY;
	for (int i = 0; i < NbOfFlag; i++)
	{
		AFlagActor* EvaluatedFlag = FlagManager->GetFlagActor(i);
		float distanceToStart = UE::Geometry::Distance(EvaluatedFlag->GetActorLocation(), StartPointIndicator->GetActorLocation());
		if (distanceToStart < minDistanceToStart )
		{
			minDistanceToStart = distanceToStart;
			StartingFlagId = i;
		}
		float distanceToEnd = UE::Geometry::Distance(EvaluatedFlag->GetActorLocation(), EndPointIndicator->GetActorLocation());
		if (distanceToEnd < minDistanceToEnd )
		{
			minDistanceToEnd = distanceToEnd;
			EndingFlagId = i;
		}
	}

	if (DGoldenPath)
	{
		DrawDebugSphere(
			GetWorld(),
			FlagManager->GetFlagActor(StartingFlagId)->GetActorLocation(),
			100,
			12,
			FColor::Green,
			true,
			300);

		DrawDebugSphere(
				GetWorld(),
				FlagManager->GetFlagActor(EndingFlagId)->GetActorLocation(),
				100,
				12,
				FColor::Red,
				true,
				300);
	}

	
	auto BestPathFound = AStarAlgorithme();
	for (auto FlagActor : FlagManager->GetFlagActors())
	{
		auto FlagSegment = FlagActor->SOFlag->Segment;
		FVector BeginPoint = FlagSegment.BeginPosition;
		FVector EndPoint = FlagSegment.EndPosition;
		FVector AdjustedLocation = BeginPoint + (EndPoint - BeginPoint) * 0.9f;

		FColor MainColor = FColor::Black;
		if (BestPathFound.Contains(FlagSegment.id))
		{
			MainColor = FColor::Green;
		}
		DrawDebugDirectionalArrow(
				GetWorld(),
				BeginPoint,
				AdjustedLocation,
				500,
				MainColor,
				true,
				300
			);
	}
	/*
	TArray<int> Came_from;
	Came_from.Init(0, NbOfFlag);
	TArray<int> cost_so_far;
	cost_so_far.Init(0, NbOfFlag);

	TArray<int> InspectionList;
	InspectionList.Push(StartingFlagId);
	TArray<int> PriorityQueue;
	
	PriorityQueue.Push(0);
	Came_from[StartingFlagId] = StartingFlagId;
	cost_so_far[StartingFlagId] = 0;

	while (!InspectionList.IsEmpty())
	{
		int Current = InspectionList.Pop();
		if (Current == EndingFlagId)
			break;
		
		TArray<int> Neighbors;
		AFlagActor* CurrentFlag = FlagManager->GetFlagActor(Current);
		Neighbors.Append(CurrentFlag->SOFlag->BeginPointIds);
		Neighbors.Append(CurrentFlag->SOFlag->EndPointIds);
		for (int next = 0; next < Neighbors.Num(); next++)
		{
			AFlagActor* NextFlag = FlagManager->GetFlagActor(next);
			float NewCost = cost_so_far[Current]
							+ (CurrentFlag->SOFlag->Segment.Lenght / 2)
							+ (NextFlag->SOFlag->Segment.Lenght / 2);
			
		}
		
		
	}

	
	

	if (StartingFlagId == EndingFlagId)
		UE_LOG(LogTemp, Warning, TEXT("GP : STARTING POINT AND END POINT SAME SEGMENT"))

	AFlagActor* StartingFlag = FlagManager->GetFlagActor(StartingFlagId);
	AFlagActor* EndingFlag = FlagManager->GetFlagActor(EndingFlagId);
	*/
}

bool ASkeletalNavMeshBoundsVolume::NavPoly_GetAllPolys(TArray<NavNodeRef>& Polys)
{
	if (!NavMesh) return false;

	TArray<FNavPoly> EachPolys;
	for (int32 i = 0; i < NavMesh->GetNavMeshTilesCount(); i++)
	{
		if (!TileIsValid(NavMesh, i))
		{
			continue;
		}

		NavMesh->GetPolysInTile(i, EachPolys);
	}

	for (int32 i = 0; i < EachPolys.Num(); i++)
	{
		Polys.Add(EachPolys[i].Ref);
	}

	return true;
}

bool ASkeletalNavMeshBoundsVolume::TileIsValid(const ARecastNavMesh* Navmesh, int32 TileIndex) const
{
	if (!NavMesh) return false;
	const FBox TileBounds = NavMesh->GetNavMeshTileBounds(TileIndex);
	return TileBounds.IsValid != 0;
}

void ASkeletalNavMeshBoundsVolume::BeginPlay()
{
	Super::BeginPlay();
	ComputeGeometry();
}

bool ASkeletalNavMeshBoundsVolume::TestDirectionnality(FVector StartLocation, FVector EndLocation)
{
	float ParentNodeDistanceToStart = UE::Geometry::Distance(StartPointIndicator->GetActorLocation(), StartLocation);
	float ParentNodeDistanceToEnd = UE::Geometry::Distance(EndPointIndicator->GetActorLocation(), StartLocation);

	if (ParentNodeDistanceToStart <= ParentNodeDistanceToEnd)
	{
		float ChildrenNodeDistance = UE::Geometry::Distance(StartPointIndicator->GetActorLocation(), EndLocation);
		return ChildrenNodeDistance > ParentNodeDistanceToStart;
	}
	else
	{
		float ChildrenNodeDistance = UE::Geometry::Distance(EndPointIndicator->GetActorLocation(), EndLocation);
		return ChildrenNodeDistance < ParentNodeDistanceToEnd;
	}
}

float ASkeletalNavMeshBoundsVolume::AStarHeuristique(int FlagId, int GoalFlagId)
{
	auto StartFlag = FlagManager->GetFlagActor(FlagId);
	auto GoalFlag = FlagManager->GetFlagActor(GoalFlagId);
	return UE::Geometry::Distance(StartFlag->GetActorLocation(), GoalFlag->GetActorLocation());
}

TArray<int> ASkeletalNavMeshBoundsVolume::AStarAlgorithme()
{
	int NbOfFlag = FlagManager->GetFlagActorSize();
	
	// Flags currently investigated
	TPriorityQueue<int> Frontier;
	Frontier.Push(StartingFlagId, 0);

	//For each Flag N, neighbors on the cheapest path
	TArray<int> CameFromFlagsN;
	CameFromFlagsN.Init(0,NbOfFlag);

	//Cheapest cost from Start Flag to N Flag
	TArray<int> GScore;
	GScore.Init(999999, NbOfFlag);
	GScore[StartingFlagId] = 0;

	//Current best guest as to howe cheap a path from Start Flag to ENd Flag who go through N Flag.
	TArray<int> FScore;
	FScore.Init(9999999, NbOfFlag);
	FScore[StartingFlagId] = AStarHeuristique(StartingFlagId, EndingFlagId);
	
	while (!Frontier.IsEmpty())
	{
		auto CurrentFlagId = Frontier.Pop();
		if (CurrentFlagId == EndingFlagId)
		{
			return AStarPathReconstructor(CameFromFlagsN, EndingFlagId);
		}
		
		TArray<int> Neighbors;
		AFlagActor* CurrentFlag = FlagManager->GetFlagActor(CurrentFlagId);
		Neighbors.Append(CurrentFlag->SOFlag->BeginPointIds);
		Neighbors.Append(CurrentFlag->SOFlag->EndPointIds);

		for (auto Neighbor : Neighbors)
		{
			AFlagActor* NextFlag = FlagManager->GetFlagActor(Neighbor);
			float Tentative_GScore = GScore[CurrentFlagId]
			+ (CurrentFlag->SOFlag->Segment.Lenght / 2)
			+ (NextFlag->SOFlag->Segment.Lenght / 2);
	
			if (Tentative_GScore < GScore[Neighbor])
			{
				CameFromFlagsN[Neighbor] = CurrentFlagId;
				GScore[Neighbor] = Tentative_GScore;
				FScore[Neighbor] = Tentative_GScore + AStarHeuristique(Neighbor, EndingFlagId);
				Frontier.Push(Neighbor, FScore[Neighbor]);
			}
		}
	}

	TArray<int> NullArray;
	return NullArray;
}

TArray<int> ASkeletalNavMeshBoundsVolume::AStarPathReconstructor(TArray<int> cameFrom, int Goal)
{
	TArray<int> BestPath;
	int GoalID = Goal;
	while (GoalID != StartingFlagId)
	{
		UE_LOG(LogTemp, Warning, TEXT("%d"), GoalID);
		BestPath.Add(GoalID);
		GoalID = cameFrom[GoalID];
	}
	return BestPath;
}



