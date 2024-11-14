#include "SkeletalNavMeshBoundsVolume.h"

#include "DataTypeUtils.h"
#include "VectorTypes.h"
#include "Kismet/KismetMathLibrary.h"

void ASkeletalNavMeshBoundsVolume::BeginPlay()
{
	Super::BeginPlay();
	ComputeGeometry();
}

//CONSOLE FUNCTION
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
				if (TestDirectionnality(SegmentBeginPoint, SegmentEndPoint))
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
						DrawDebugLine(
							GetWorld(),
							SegmentBeginPoint,
							AdjustedLocation,
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

void ASkeletalNavMeshBoundsVolume::ResetAllFlagTypes()
{
	ClearDebugLine();
	for (AFlagActor* Element : FlagManager->GetFlagActors())
	{
		Element->SOFlag->Segment.FlagType = EFlagType::NONE;
		Element->SOFlag->Segment.PathType = EFlagPathType::NONE;
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
	ClearDebugLine();
	int NbOfFlag = FlagManager->GetFlagActorSize();
	if (NbOfFlag == -1)
		return;

	//Find Starting and Ending Node
	float minDistanceToStart = INFINITY;
	float minDistanceToEnd = INFINITY;
	for (int i = 0; i < NbOfFlag; i++)
	{
		AFlagActor* EvaluatedFlag = FlagManager->GetFlagActor(i);
		float distanceToStart = UE::Geometry::Distance(EvaluatedFlag->GetActorLocation(),
		                                               StartPointIndicator->GetActorLocation());
		if (distanceToStart < minDistanceToStart)
		{
			minDistanceToStart = distanceToStart;
			GoldenStartingFlagId = i;
		}
		float distanceToEnd = UE::Geometry::Distance(EvaluatedFlag->GetActorLocation(),
		                                             EndPointIndicator->GetActorLocation());
		if (distanceToEnd < minDistanceToEnd)
		{
			minDistanceToEnd = distanceToEnd;
			GoldenEndingFlagId = i;
		}
	}

	if (DGoldenPath)
	{
		DrawDebugSphere(
			GetWorld(),
			FlagManager->GetFlagActor(GoldenStartingFlagId)->GetActorLocation(),
			100,
			12,
			FColor::Green,
			true,
			300);

		DrawDebugSphere(
			GetWorld(),
			FlagManager->GetFlagActor(GoldenEndingFlagId)->GetActorLocation(),
			100,
			12,
			FColor::Red,
			true,
			300);
	}

	auto StartFlag = FlagManager->GetFlagActor(GoldenStartingFlagId);
	auto EndFlag = FlagManager->GetFlagActor(GoldenEndingFlagId);

	if (UE::Geometry::Distance(StartFlag->GetActorLocation(), EndFlag->GetActorLocation()) > MinimumPathLenght)
	{
		UE_LOG(LogTemp, Warning, TEXT("GP : MinimumPathLenght too short"))
		return;
	}

	int Divider = 1;
	float CurrentLenght = 0;
	TArray<int> BestPathFound;
	TArray<int> ForcedFlagInPath;


	while (Divider < 5)
	{
		CurrentLenght = 0;
		BestPathFound.Empty();
		ForcedFlagInPath.Empty();


		ForcedFlagInPath.Add(GoldenStartingFlagId);
		for (int i = 1; i < Divider; i++)
		{
			UE_LOG(LogTemp, Warning, TEXT("GP : Segment Path in %d !"), Divider)
			float SearchDistanceToLast = MinimumPathLenght / Divider;
			float SearchDistanceToEnd = MinimumPathLenght * (Divider - i) / Divider;
			UE_LOG(LogTemp, Warning, TEXT("GP : fisrt length %f !"), SearchDistanceToLast)
			UE_LOG(LogTemp, Warning, TEXT("GP : second length %f !"), SearchDistanceToEnd)

			auto PrecedentFlagPosition = FlagManager->GetFlagActor(ForcedFlagInPath.Last())->GetActorLocation();
			float BestCandidateScore = 999999;
			int BestCandidate = ForcedFlagInPath.Last();

			for (int j = 0; j < NbOfFlag; j++)
			{
				auto EvaluatedFlagPosition = FlagManager->GetFlagActor(j)->GetActorLocation();
				float EvaluatedDistanceToLast = UE::Geometry::Distance(PrecedentFlagPosition, EvaluatedFlagPosition);
				float EvaluatedDistanceToEnd = UE::Geometry::Distance(EndFlag->GetActorLocation(),
				                                                      EvaluatedFlagPosition);
				float EvaluatedCandidateScore = abs(EvaluatedDistanceToLast - SearchDistanceToLast) + abs(
					EvaluatedDistanceToEnd - SearchDistanceToEnd);
				if (EvaluatedCandidateScore < BestCandidateScore)
				{
					BestCandidateScore = EvaluatedCandidateScore;
					BestCandidate = j;
				}
			}
			ForcedFlagInPath.Add(BestCandidate);
			if (DGoldenPath)
			{
				DrawDebugSphere(
					GetWorld(),
					FlagManager->GetFlagActor(BestCandidate)->GetActorLocation(),
					100,
					12,
					FColor::Blue,
					true,
					300);
			}
		}
		ForcedFlagInPath.Add(GoldenEndingFlagId);

		for (int j = 0; j < ForcedFlagInPath.Num() - 1; j++)
		{
			UE_LOG(LogTemp, Warning, TEXT("GP : Add To BestPath !"))
			TArray<int> PathSegmentFound;
			CurrentLenght += AStarAlgorithme(ForcedFlagInPath[j], ForcedFlagInPath[j + 1], PathSegmentFound);
			BestPathFound.Append(PathSegmentFound);
		}
		UE_LOG(LogTemp, Warning, TEXT("GP : Current Path Lenght : %f"), CurrentLenght)
		if (CurrentLenght > MinimumPathLenght)
		{
			UE_LOG(LogTemp, Warning, TEXT("GP : Found Valid Path !"))
			break;
		}

		Divider++;
	}

	GoldenPath = BestPathFound;
	GoldenPathCopy = TArray(GoldenPath);
	GoldenPathCopy.Remove(GoldenStartingFlagId);
	GoldenPathCopy.Remove(GoldenEndingFlagId);

	for (AFlagActor* FlagActor : FlagManager->GetFlagActors())
	{
		auto FlagSegment = FlagActor->SOFlag->Segment;
		FVector BeginPoint = FlagSegment.BeginPosition;
		FVector EndPoint = FlagSegment.EndPosition;
		FVector AdjustedLocation = BeginPoint + (EndPoint - BeginPoint) * 0.9f;

		FColor MainColor = FColor::Black;
		if (BestPathFound.Contains(FlagSegment.id))
		{
			MainColor = FColor::Green;
			FlagActor->SOFlag->Segment.PathType = EFlagPathType::GOLDEN;
		}
		else
		{
			FlagActor->SOFlag->Segment.PathType = EFlagPathType::ALTERNATIVE;
		}

		if (DGoldenPath)
		{
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
		else
		{
			DrawDebugDirectionalArrow(
				GetWorld(),
				BeginPoint,
				AdjustedLocation,
				500,
				FColor::Red,
				true,
				300
			);
		}
	}
}

void ASkeletalNavMeshBoundsVolume::CalculateDirectionnality()
{
	AFlagActor* StartFlag = FlagManager->GetFlagActor(GoldenStartingFlagId);
	StartFlag->SOFlag->Segment.FlagType = EFlagType::SAFE;
	AFlagActor* GoalFlag = FlagManager->GetFlagActor(GoldenEndingFlagId);
	GoalFlag->SOFlag->Segment.FlagType = EFlagType::SAFE;

	for (AFlagActor* FlagActor : FlagManager->GetFlagActors())
	{
		if (FlagActor->SOFlag->Segment.FlagType == EFlagType::SAFE)
			continue;

		//Test begin to end
		//Test end to beginning
	}
}

void ASkeletalNavMeshBoundsVolume::SelectAllChallengeSegments()
{
	ClearDebugLine();
	
	while(GoldenPathCopy.Num() > 0)
	{
		SelectChallengeSegments();
	}
	GoldenPathCopy = TArray(GoldenPath);
}

void ASkeletalNavMeshBoundsVolume::SelectChallengeSegments()
{
	AFlagActor* BeginFlag = FlagManager->GetFlagActor(GoldenStartingFlagId);
	BeginFlag->SOFlag->Segment.FlagType = EFlagType::SAFE;
	DrawDebugLine(
		GetWorld(),
		BeginFlag->SOFlag->Segment.BeginPosition,
		BeginFlag->SOFlag->Segment.EndPosition,
		FColor::Blue,
		true,
		300
	);
	AFlagActor* EndFlag = FlagManager->GetFlagActor(GoldenEndingFlagId);
	EndFlag->SOFlag->Segment.FlagType = EFlagType::SAFE;
	DrawDebugLine(
		GetWorld(),
		EndFlag->SOFlag->Segment.BeginPosition,
		EndFlag->SOFlag->Segment.EndPosition,
		FColor::Blue,
		true,
		300
	);


	if (GoldenPathCopy.Num() <= 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("golden path copy is empty"));
		return;
	}

	UE_LOG(LogTemp, Warning, TEXT("golden path copy length: %d"), GoldenPathCopy.Num());
	int RandomGPSegment = UKismetMathLibrary::RandomInteger64InRange(0, GoldenPathCopy.Num() - 1);
	UE_LOG(LogTemp, Warning, TEXT("random gp segment: %d"), RandomGPSegment);
	AFlagActor* GPSegmentFlag = FlagManager->GetFlagActor(GoldenPathCopy[RandomGPSegment]);
	GoldenPathCopy.RemoveAt(RandomGPSegment);

	if (GPSegmentFlag->SOFlag->IsTouchingFlagType(EFlagType::SAFE, false))
	{
		GPSegmentFlag->SOFlag->Segment.FlagType = EFlagType::RISKY;
		DrawDebugLine(
			GetWorld(),
			GPSegmentFlag->SOFlag->Segment.BeginPosition,
			GPSegmentFlag->SOFlag->Segment.EndPosition,
			FColor::Yellow,
			true,
			300
		);
		return;
	}


	int NbSurroundingChallenges = GPSegmentFlag->SOFlag->IsTouchingFlagType(EFlagType::CHALLENGE, true);

	bool IsPartOfChallengeGroup = UKismetMathLibrary::RandomBool();

	switch (NbSurroundingChallenges)
	{
	case 0:
		if (GPSegmentFlag->SOFlag->IsTouchingPathType(EFlagPathType::ALTERNATIVE))
			GPSegmentFlag->SOFlag->Segment.FlagType = EFlagType::CHALLENGE;
		else
			GPSegmentFlag->SOFlag->Segment.FlagType = EFlagType::RISKY;
		break;
	case 1:
		if (IsPartOfChallengeGroup)
		{
			/*TODO: ADD TO CHALLENGE GROUP WHEN THATS DONE*/
			GPSegmentFlag->SOFlag->Segment.FlagType = EFlagType::CHALLENGE; //FIXME: change for group :D
		}
		else
			GPSegmentFlag->SOFlag->Segment.FlagType = EFlagType::RISKY;
		break;
	default:
		GPSegmentFlag->SOFlag->Segment.FlagType = EFlagType::RISKY;
		break;
	}

	switch (GPSegmentFlag->SOFlag->Segment.FlagType)
	{
	case EFlagType::NONE:
		DrawDebugLine(
			GetWorld(),
			GPSegmentFlag->SOFlag->Segment.BeginPosition,
			GPSegmentFlag->SOFlag->Segment.EndPosition,
			FColor::Black,
			true,
			300
		);
		break;
	case EFlagType::SAFE:
		DrawDebugLine(
			GetWorld(),
			GPSegmentFlag->SOFlag->Segment.BeginPosition,
			GPSegmentFlag->SOFlag->Segment.EndPosition,
			FColor::Blue,
			true,
			300
		);
		break;
	case EFlagType::RISKY:
		DrawDebugLine(
			GetWorld(),
			GPSegmentFlag->SOFlag->Segment.BeginPosition,
			GPSegmentFlag->SOFlag->Segment.EndPosition,
			FColor::Yellow,
			true,
			300
		);
		break;
	case EFlagType::CHALLENGE:
		DrawDebugLine(
			GetWorld(),
			GPSegmentFlag->SOFlag->Segment.BeginPosition,
			GPSegmentFlag->SOFlag->Segment.EndPosition,
			FColor::Red,
			true,
			300
		);
		break;
	default:
		DrawDebugLine(
			GetWorld(),
			GPSegmentFlag->SOFlag->Segment.BeginPosition,
			GPSegmentFlag->SOFlag->Segment.EndPosition,
			FColor::Black,
			true,
			300
		);
		break;
	}
}

//GEOMETRY FUNCTION
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

//ASTAR FUNCTION
float ASkeletalNavMeshBoundsVolume::AStarHeuristique(int Start, int Goal)
{
	auto StartFlag = FlagManager->GetFlagActor(Start);
	auto GoalFlag = FlagManager->GetFlagActor(Goal);
	return UE::Geometry::Distance(StartFlag->GetActorLocation(), GoalFlag->GetActorLocation());
}

float ASkeletalNavMeshBoundsVolume::AStarAlgorithme(int StartFlagID, int EndFlagID, TArray<int>& BestPath)
{
	int NbOfFlag = FlagManager->GetFlagActorSize();

	// Flags currently investigated
	TPriorityQueue<int> Frontier;
	Frontier.Push(StartFlagID, 0);

	//For each Flag N, neighbors on the cheapest path
	TArray<int> CameFromFlagsN;
	CameFromFlagsN.Init(0, NbOfFlag);

	//Cheapest cost from Start Flag to N Flag
	TArray<int> GScore;
	GScore.Init(999999, NbOfFlag);
	GScore[StartFlagID] = 0;

	//Current best guest as to howe cheap a path from Start Flag to ENd Flag who go through N Flag.
	TArray<int> FScore;
	FScore.Init(9999999, NbOfFlag);
	FScore[StartFlagID] = AStarHeuristique(StartFlagID, EndFlagID);

	while (!Frontier.IsEmpty())
	{
		auto CurrentFlagId = Frontier.Pop();
		if (CurrentFlagId == EndFlagID)
		{
			return AStarPathReconstructor(CameFromFlagsN, StartFlagID, EndFlagID, BestPath);
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
				FScore[Neighbor] = Tentative_GScore + AStarHeuristique(Neighbor, EndFlagID);
				Frontier.Push(Neighbor, FScore[Neighbor]);
			}
		}
	}

	return -1;
}

float ASkeletalNavMeshBoundsVolume::AStarPathReconstructor(TArray<int> CameFrom, int Start, int Goal,
                                                           TArray<int>& ReconstructedPath)
{
	ReconstructedPath.Empty();
	float TotalPathLenght = FlagManager->GetFlagActor(Goal)->SOFlag->Segment.Lenght / 2;
	int GoalID = Goal;
	while (GoalID != Start)
	{
		UE_LOG(LogTemp, Warning, TEXT("%d"), GoalID);
		ReconstructedPath.Add(GoalID);
		auto Flag = FlagManager->GetFlagActor(GoalID);
		if (GoalID != Goal)
			TotalPathLenght += Flag->SOFlag->Segment.Lenght;
		GoalID = CameFrom[GoalID];
	}
	TotalPathLenght += FlagManager->GetFlagActor(Start)->SOFlag->Segment.Lenght / 2;
	return TotalPathLenght;
}
