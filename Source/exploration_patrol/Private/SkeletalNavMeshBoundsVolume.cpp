// Fill out your copyright notice in the Description page of Project Settings.


#include "SkeletalNavMeshBoundsVolume.h"

#include "VectorTypes.h"

void ASkeletalNavMeshBoundsVolume::ClearDebugLine()
{
	FlushPersistentDebugLines(GetWorld());
}

void ASkeletalNavMeshBoundsVolume::ComputeGeometry()
{
	int CurrentID = 0;
	FlushPersistentDebugLines(GetWorld());
	if (!NavMesh) return;
	if (!StartPoint) return;
	if (!EndPoint) return;

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
	float ParentNodeDistanceToStart = UE::Geometry::Distance(StartPoint->GetActorLocation(), StartLocation);
	float ParentNodeDistanceToEnd = UE::Geometry::Distance(EndPoint->GetActorLocation(), StartLocation);

	if (ParentNodeDistanceToStart <= ParentNodeDistanceToEnd)
	{
		float ChildrenNodeDistance = UE::Geometry::Distance(StartPoint->GetActorLocation(), EndLocation);
		return ChildrenNodeDistance > ParentNodeDistanceToStart;
	}
	else
	{
		float ChildrenNodeDistance = UE::Geometry::Distance(EndPoint->GetActorLocation(), EndLocation);
		return ChildrenNodeDistance < ParentNodeDistanceToEnd;
	}
}
