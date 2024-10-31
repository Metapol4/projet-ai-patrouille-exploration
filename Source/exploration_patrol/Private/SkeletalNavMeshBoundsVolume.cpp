// Fill out your copyright notice in the Description page of Project Settings.


#include "SkeletalNavMeshBoundsVolume.h"

#include "VectorTypes.h"

bool ASkeletalNavMeshBoundsVolume::NavPoly_GetAllPolys(TArray<NavNodeRef>& Polys)
{
	if (!NavMesh) return false;

	TArray<FNavPoly> EachPolys;
	for (int32 i=0; i<NavMesh->GetNavMeshTilesCount(); i++)
	{
		if (!TileIsValid(NavMesh, i))
		{
			continue;
		}

		NavMesh->GetPolysInTile(i, EachPolys);
	}

	for (int32 i=0; i< EachPolys.Num(); i++)
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

	if (!NavMesh) return;
	if(!StartPoint) return;
	if(!EndPoint) return;
	
	NavPoly_GetAllPolys(PolyArray);
	for (NavNodeRef Polys : PolyArray)
	{
		FVector DebugLocation;
		NavMesh->GetPolyCenter(Polys, DebugLocation);
		TArray<NavNodeRef> Neighbors;
		float ParentNodeDistanceToStart =  UE::Geometry::Distance(StartPoint->GetActorLocation(), DebugLocation);
		float ParentNodeDistanceToEnd =  UE::Geometry::Distance(EndPoint->GetActorLocation(), DebugLocation);

		DrawDebugSphere(
			GetWorld(),
			DebugLocation,
			5,
			12,
			FColor::Cyan,
			true,
			300);
		
		if(ParentNodeDistanceToStart <= ParentNodeDistanceToEnd)
		{
			if(NavMesh->GetPolyNeighbors(Polys, Neighbors))
			{
				FVector EndLineLocation;
				for (auto Neighbor : Neighbors)
				{
					NavMesh->GetPolyCenter(Neighbor, EndLineLocation);
					float ChildrenNodeDistance =  UE::Geometry::Distance(StartPoint->GetActorLocation(), EndLineLocation);
					FVector AdjustedLocation = DebugLocation + (EndLineLocation - DebugLocation) * 0.9f; 
					if(ChildrenNodeDistance > ParentNodeDistanceToStart)
					{
						DrawDebugDirectionalArrow(
											GetWorld(),
											DebugLocation,
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
		else
		{
			if(NavMesh->GetPolyNeighbors(Polys, Neighbors))
			{
				FVector EndLineLocation;
				for (auto Neighbor : Neighbors)
				{
					NavMesh->GetPolyCenter(Neighbor, EndLineLocation);
					float ChildrenNodeDistance =  UE::Geometry::Distance(EndPoint->GetActorLocation(), EndLineLocation);
					FVector AdjustedLocation = DebugLocation + (EndLineLocation - DebugLocation) * 0.9f; 
					if(ChildrenNodeDistance < ParentNodeDistanceToEnd)
					{
						DrawDebugDirectionalArrow(
											GetWorld(),
											DebugLocation,
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
