#include "SkeletalNavMeshBoundsVolume.h"

#include "CookOnTheFly.h"
#include "DataTypeUtils.h"
#include "ToolContextInterfaces.h"
#include "VectorTypes.h"
#include "Kismet/KismetMathLibrary.h"

void ASkeletalNavMeshBoundsVolume::GenerateAll()
{
	SendFlagBatch();
	CalculateVisionGroups();
	//Find Safe Segment
	FindSafeSegments();
	CalculateDirectionnality(EFlagType::SAFE);
	//CreateChallenges();
	//FindGoldenPath();
	/* OldChallenge selection
	SelectAllChallengeSegments();
	CreateChallengeGroups();
	PrintChallengeGroups();
	*/
}

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
		TArray<FVector> SegmentVertex;
		NavMesh->GetPolyVerts(Polys, SegmentVertex);
		float SegmentArea = .0f;
		for (int i = 1; i < SegmentVertex.Num() - 1; i++)
		{
			FVector A = SegmentVertex[0];
			FVector B = SegmentVertex[i];
			FVector C = SegmentVertex[i + 1];

			FVector AB = A - B;
			FVector AC = A - C;

			float NormAB = AB.Size();
			float NormAC = AC.Size();

			AB.Normalize();
			AC.Normalize();
			float AngleD = UE::Geometry::AngleD(AB, AC);

			SegmentArea += .25f * (NormAB * NormAC * UKismetMathLibrary::DegSin(AngleD));
		}


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

					TArray<FVector> SegmentVertexEnd;
					NavMesh->GetPolyVerts(Node, SegmentVertexEnd);
					float SegmentAreaEnd = .0f;
					for (int i = 1; i < SegmentVertexEnd.Num() - 1; i++)
					{
						FVector A = SegmentVertexEnd[0];
						FVector B = SegmentVertexEnd[i];
						FVector C = SegmentVertexEnd[i + 1];

						FVector AB = A - B;
						FVector AC = A - C;

						float NormAB = AB.Size();
						float NormAC = AC.Size();

						AB.Normalize();
						AC.Normalize();
						float AngleD = UE::Geometry::AngleD(AB, AC);

						SegmentAreaEnd += .25f * (NormAB * NormAC * UKismetMathLibrary::DegSin(AngleD));
					}

					CurrentSegment.BeginPosition = SegmentBeginPoint;
					CurrentSegment.EndPosition = SegmentEndPoint;
					CurrentSegment.Direction = EFlagDirection::NONE;
					CurrentSegment.Lenght = UE::Geometry::Distance(SegmentBeginPoint, SegmentEndPoint);
					CurrentSegment.Area = SegmentArea + SegmentAreaEnd;
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

void ASkeletalNavMeshBoundsVolume::HighlightVisionGroupsFromList()
{
	if (DSVisionPathsToHighlight.Num() > 0)
	{
		ClearDebugLine();
		FlagManager->ShowVisionGroupForActors(DSVisionPathsToHighlight);
	}
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

void ASkeletalNavMeshBoundsVolume::FindSafeSegments()
{
	ClearDebugLine();

	//Find Starting and Ending Node
	int NbOfFlag = FlagManager->GetFlagActorSize();
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

	FlagManager->GetFlagActor(GoldenStartingFlagId)->SOFlag->Segment.FlagType = EFlagType::SAFE;
	FlagManager->GetFlagActor(GoldenEndingFlagId)->SOFlag->Segment.FlagType = EFlagType::SAFE;

	TArray<AFlagActor*> TemporaryFlagList = FlagManager->GetFlagActors();

	TemporaryFlagList.Sort([](const AFlagActor& ip1, const AFlagActor& ip2)
	{
		return ip1.SOFlag->Segment.VisionArea < ip2.SOFlag->Segment.VisionArea;
	});

	int ListLimit = UKismetMathLibrary::FFloor(TemporaryFlagList.Num() * (PercentageSafeSegment / 100));
	if (ListLimit > TemporaryFlagList.Num())
		ListLimit = TemporaryFlagList.Num();

	for (int i = 0; i < ListLimit; i++)
	{
		TemporaryFlagList[i]->SOFlag->Segment.FlagType = EFlagType::SAFE;
		DrawDebugLine(
			GetWorld(),
			TemporaryFlagList[i]->SOFlag->Segment.BeginPosition,
			TemporaryFlagList[i]->SOFlag->Segment.EndPosition,
			FColor::Yellow,
			true,
			300
		);
	}
}

void ASkeletalNavMeshBoundsVolume::CalculateDebugDirectionnality()
{
	CalculateDirectionnality(DebugFlagTypeDirection);
}

void ASkeletalNavMeshBoundsVolume::CalculateDirectionnality(EFlagType FlagType)
{
	ClearDebugLine();
	AFlagActor* StartFlag = FlagManager->GetFlagActor(GoldenStartingFlagId);
	StartFlag->SOFlag->Segment.FlagType = EFlagType::SAFE;
	AFlagActor* GoalFlag = FlagManager->GetFlagActor(GoldenEndingFlagId);
	GoalFlag->SOFlag->Segment.FlagType = EFlagType::SAFE;

	for (AFlagActor* FlagActor : FlagManager->GetFlagActors())
	{
		if (FlagActor->SOFlag->Segment.FlagType == FlagType)
			continue;

		FlagActor->SOFlag->Segment.Direction = GetAdditiveFlagDirection(
			EFlagDirection::BOTH, FlagActor->SOFlag->Segment.Direction);
		for (int id : FlagActor->SOFlag->Segment.VisibilityGroups)
		{
			UE_LOG(LogTemp, Warning, TEXT("DIRCT : Evaluating %d"), id)
			if (FlagManager->GetFlagActor(id)->SOFlag->Segment.FlagType != FlagType)
				continue;

			FVector ActorToSafe = FlagManager->GetFlagActor(id)->GetActorLocation() - FlagActor->GetActorLocation();
			ActorToSafe.Normalize();

			//Test begin to end
			FVector BeginToEnd = FlagActor->SOFlag->Segment.EndPosition - FlagActor->SOFlag->Segment.BeginPosition;
			BeginToEnd.Normalize();

			float EvaluatedAngle = UE::Geometry::AngleD(ActorToSafe, BeginToEnd);
			UE_LOG(LogTemp, Warning, TEXT("DIRCT : Angle Begin to End %f"), EvaluatedAngle)
			if (EvaluatedAngle < AngleTolerance)
			{
				FlagActor->SOFlag->Segment.Direction = GetAdditiveFlagDirection(
					EFlagDirection::END_BEGIN, FlagActor->SOFlag->Segment.Direction);
			}

			//Test end to begin
			FVector EndToBegin = FlagActor->SOFlag->Segment.BeginPosition - FlagActor->SOFlag->Segment.EndPosition;
			EndToBegin.Normalize();

			EvaluatedAngle = UE::Geometry::AngleD(ActorToSafe, EndToBegin);
			UE_LOG(LogTemp, Warning, TEXT("DIRCT : Angle End to Begin %f"), EvaluatedAngle)
			if (EvaluatedAngle < AngleTolerance)
			{
				switch (FlagActor->SOFlag->Segment.Direction)
				{
				case EFlagDirection::NONE:
				case EFlagDirection::BEGIN_END:
				case EFlagDirection::IMPOSSIBLE:
					break;
				case EFlagDirection::BOTH:
					FlagActor->SOFlag->Segment.Direction = GetAdditiveFlagDirection(
						EFlagDirection::BEGIN_END, FlagActor->SOFlag->Segment.Direction);;
					break;
				case EFlagDirection::END_BEGIN:
					FlagActor->SOFlag->Segment.Direction = GetAdditiveFlagDirection(
						EFlagDirection::IMPOSSIBLE, FlagActor->SOFlag->Segment.Direction);;
					break;
				default: break;
				}
			}
		}

		DebugDirectionality(FlagActor->SOFlag->Segment.id);
	}
}

EFlagDirection ASkeletalNavMeshBoundsVolume::GetAdditiveFlagDirection(EFlagDirection WantedDirection,
                                                                      EFlagDirection CurrentDirection)
{
	switch (CurrentDirection)
	{
	case EFlagDirection::NONE:
		return WantedDirection;

	case EFlagDirection::BEGIN_END:
		if (WantedDirection == EFlagDirection::END_BEGIN)
			return EFlagDirection::IMPOSSIBLE;
		if (WantedDirection == EFlagDirection::BOTH)
			return CurrentDirection;

		return WantedDirection;

	case EFlagDirection::END_BEGIN:
		if (WantedDirection == EFlagDirection::BEGIN_END)
			return EFlagDirection::IMPOSSIBLE;
		if (WantedDirection == EFlagDirection::BOTH)
			return CurrentDirection;

		return WantedDirection;

	case EFlagDirection::BOTH:
		return WantedDirection;

	case EFlagDirection::IMPOSSIBLE:
		return EFlagDirection::IMPOSSIBLE;
	}
	return CurrentDirection;
}

void ASkeletalNavMeshBoundsVolume::LegacySelectAllChallengeSegments()
{
	ClearDebugLine();

	while (GoldenPathCopy.Num() > 0)
	{
		LegacySelectChallengeSegments();
	}
	GoldenPathCopy = TArray(GoldenPath);
}

void ASkeletalNavMeshBoundsVolume::LegacySelectChallengeSegments()
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

	int PartOfChallengeGroupRoll = UKismetMathLibrary::RandomIntegerInRange(0, 100);

	switch (NbSurroundingChallenges)
	{
	case 0:
		if (GPSegmentFlag->SOFlag->IsTouchingPathType(EFlagPathType::ALTERNATIVE))
			GPSegmentFlag->SOFlag->Segment.FlagType = EFlagType::CHALLENGE;
		else
			GPSegmentFlag->SOFlag->Segment.FlagType = EFlagType::RISKY;
		break;
	case 1:
		if (PartOfChallengeGroupRoll <= PercentChanceOfMergingChallengeGroup)
			GPSegmentFlag->SOFlag->Segment.FlagType = EFlagType::CHALLENGE;
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

void ASkeletalNavMeshBoundsVolume::GenerateOneGuardPath()
{
	ClearDebugLine();
	TArray<AFlagActor*> TemporaryFlagList = FlagManager->GetFlagActors();

	FilterAndSortOutAltAndBidirectional(TemporaryFlagList);

	int ListLimit =
		UKismetMathLibrary::FFloor(TemporaryFlagList.Num() * (PercentageRandomStartingPointSelection / 100));
	if (ListLimit > TemporaryFlagList.Num())
		ListLimit = TemporaryFlagList.Num();

	int StartingFlag = UKismetMathLibrary::RandomIntegerInRange(0, ListLimit);
	int EndingFlag = -1;

	TArray<int> EvaluatedGuardPath;
	EvaluatedGuardPath.Init(-1, FlagManager->GetFlagActorSize());
	EvaluatedGuardPath[StartingFlag] = StartingFlag;

	TArray<int> ReconstructedChallengePath;
	GuardPathMoreThanKGenerator(StartingFlag, KLengthTarget, FVector2d(1, 1), EvaluatedGuardPath, EndingFlag);


	
	AStarPathReconstructor(EvaluatedGuardPath, StartingFlag, EndingFlag, ReconstructedChallengePath);
	for (int ChallengePathID : ReconstructedChallengePath)
	{
		AFlagActor* ChallengeFlag = FlagManager->GetFlagActor(ChallengePathID);
		FColor MainColor = FColor::Green;
		if (ChallengePathID == StartingFlag)
		{
			MainColor = FColor::Red;
		}
		DrawDebugDirectionalArrow(
			GetWorld(),
			ChallengeFlag->SOFlag->Segment.BeginPosition,
			ChallengeFlag->SOFlag->Segment.EndPosition,
			500,
			MainColor,
			true,
			300
		);
	}
}

void ASkeletalNavMeshBoundsVolume::CreateChallenges()
{
	// makes button spammable :D
	ClearDebugLine();
	ChallengeGroups.Empty();
	float TotalGPLength = 0;
	for (int Element : GoldenPath)
	{
		AFlagActor* Flag = FlagManager->GetFlagActor(Element);
		TotalGPLength += Flag->SOFlag->Segment.Lenght;
	}
	if (TotalGPLength <= 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("CHLNG: Golden Path of length 0"))
		return;
	}

	// first and last always safe
	AFlagActor* StartFlag = FlagManager->GetFlagActor(GoldenStartingFlagId);
	AFlagActor* EndFlag = FlagManager->GetFlagActor(GoldenEndingFlagId);

	StartFlag->SOFlag->Segment.FlagType = EFlagType::SAFE;
	DrawDebugLine(
		GetWorld(),
		StartFlag->SOFlag->Segment.BeginPosition,
		StartFlag->SOFlag->Segment.EndPosition,
		FColor::Blue,
		true,
		300
	);
	EndFlag->SOFlag->Segment.FlagType = EFlagType::SAFE;
	DrawDebugLine(
		GetWorld(),
		EndFlag->SOFlag->Segment.BeginPosition,
		EndFlag->SOFlag->Segment.EndPosition,
		FColor::Blue,
		true,
		300
	);

	float LengthPerChallenge = TotalGPLength / NbOfChallenges;
	int SavedJ = 0;
	for (int i = 0; i < NbOfChallenges; i++)
	{
		TArray<int> NewGroup = TArray<int>();
		ChallengeGroups.Add(NewGroup);

		float CurrentChallengeLength = 0;
		for (int j = SavedJ + 1; j < GoldenPath.Num() - 1; j++)
		{
			AFlagActor* Flag = FlagManager->GetFlagActor(GoldenPath[j]);
			CurrentChallengeLength += Flag->SOFlag->Segment.Lenght;

			if (CurrentChallengeLength < LengthPerChallenge)
			{
				Flag->SOFlag->Segment.FlagType = EFlagType::CHALLENGE;
				ChallengeGroups.Last().Add(Flag->SOFlag->Segment.id);

				DrawDebugLine(
					GetWorld(),
					Flag->SOFlag->Segment.BeginPosition,
					Flag->SOFlag->Segment.EndPosition,
					FColor::Red,
					true,
					300
				);
			}
			else
			{
				Flag->SOFlag->Segment.FlagType = EFlagType::SAFE;
				SavedJ = j;

				DrawDebugLine(
					GetWorld(),
					Flag->SOFlag->Segment.BeginPosition,
					Flag->SOFlag->Segment.EndPosition,
					FColor::Blue,
					true,
					300
				);
				break;
			}
		}
	}
}

void ASkeletalNavMeshBoundsVolume::LegacyCreateChallengeGroups()
{
	for (int i = 1; i < GoldenPath.Num(); i++)
	{
		AFlagActor* CurrentSegment = FlagManager->GetFlagActor(GoldenPath[i]);
		AFlagActor* PreviousSegment = FlagManager->GetFlagActor(GoldenPath[i - 1]);
		if (CurrentSegment->SOFlag->Segment.FlagType == EFlagType::CHALLENGE)
		{
			if (PreviousSegment->SOFlag->Segment.FlagType != EFlagType::CHALLENGE)
			{
				TArray<int> NewGroup = TArray<int>();
				NewGroup.Add(CurrentSegment->SOFlag->Segment.id);
				ChallengeGroups.Add(NewGroup);
			}
			else
				ChallengeGroups.Last().Add(CurrentSegment->SOFlag->Segment.id);
		}
	}
}

void ASkeletalNavMeshBoundsVolume::PrintChallengeGroups()
{
	for (int i = 0; i < ChallengeGroups.Num(); i++)
	{
		FString GroupPrint = "group " + FString::FromInt(i);
		GroupPrint.Append(" ");
		for (auto SingularPrint : ChallengeGroups[i])
		{
			GroupPrint.Append(FString::FromInt(SingularPrint));
			GroupPrint.Append(" ");
		}
		UE_LOG(LogTemp, Warning, TEXT("CHLNGGRP: is %s"), *GroupPrint);
	}
}

void ASkeletalNavMeshBoundsVolume::ConstructChallengePaths()
{
	//ClearDebugLine();

	int Goal = 0;
	TArray<int> ReconstructedChallengePath;

	for (int i = 0; i < ChallengeGroups.Num(); i++)
	{
		UE_LOG(LogTemp, Warning, TEXT("CHLG : %d"), i)
		TArray<int> ChallengePathInitialiser;
		ChallengePathInitialiser.Init(-1, FlagManager->GetFlagActorSize());
		ChallengePath.Add(ChallengePathInitialiser);

		int StartingFlag = 0;
		int MaximumVisionSize = -1;
		for (int j = 0; j < ChallengeGroups[i].Num(); j++)
		{
			int EvaluatedVisionSize = FlagManager->GetFlagActor(ChallengeGroups[i][j])
			                                     ->SOFlag->Segment.VisibilityGroups.Num();
			if (EvaluatedVisionSize > MaximumVisionSize)
			{
				MaximumVisionSize = EvaluatedVisionSize;
				StartingFlag = ChallengeGroups[i][j];
			}
		}

		ChallengePath[i][StartingFlag] = StartingFlag;
		KLenghtIterations = 0;

		PathMoreThanKUtil(StartingFlag, KLengthTarget, ChallengePath[i], Goal);
		AStarPathReconstructor(ChallengePath[i], StartingFlag, Goal, ReconstructedChallengePath);
		for (auto ChallengePathID : ReconstructedChallengePath)
		{
			AFlagActor* ChallengeFlag = FlagManager->GetFlagActor(ChallengePathID);
			FColor MainColor = FColor::Green;
			if (ChallengePathID == StartingFlag)
			{
				MainColor = FColor::Red;
			}
			DrawDebugDirectionalArrow(
				GetWorld(),
				ChallengeFlag->SOFlag->Segment.BeginPosition,
				ChallengeFlag->SOFlag->Segment.EndPosition,
				500,
				MainColor,
				true,
				300
			);
		}
	}
}

bool ASkeletalNavMeshBoundsVolume::AreSameChallengeGroup(int FlagA, int FlagB)
{
	for (auto Element : ChallengeGroups)
	{
		if (Element.Contains(FlagA) && Element.Contains(FlagB))
		{
			return true;
		}
	}
	return false;
}

void ASkeletalNavMeshBoundsVolume::FilterAndSortOutAltAndBidirectional(TArray<AFlagActor*>& TemporaryFlagList)
{
	for (AFlagActor* Flag : FlagManager->GetFlagActors())
	{
		if (Flag->SOFlag->Segment.FlagType == EFlagType::SAFE)
		{
			if (TemporaryFlagList.Contains(Flag))
				TemporaryFlagList.Remove(Flag);
		}

		if (Flag->SOFlag->Segment.Direction != EFlagDirection::BOTH)
		{
			if (TemporaryFlagList.Contains(Flag))
				TemporaryFlagList.Remove(Flag);
		}
	}

	TemporaryFlagList.Sort([](const AFlagActor& ip1, const AFlagActor& ip2)
	{
		return ip1.SOFlag->Segment.VisionArea > ip2.SOFlag->Segment.VisionArea;
	});
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
		if (GoalID < 0)
		{
			UE_LOG(LogTemp, Warning, TEXT("CHLG : FAIL AT %d"), GoalID);
			return TotalPathLenght;
		}
		UE_LOG(LogTemp, Warning, TEXT("%d"), GoalID);
		ReconstructedPath.Add(GoalID);
		auto Flag = FlagManager->GetFlagActor(GoalID);
		if (GoalID != Goal)
			TotalPathLenght += Flag->SOFlag->Segment.Lenght;
		GoalID = CameFrom[GoalID];
	}
	ReconstructedPath.Add(Start);
	TotalPathLenght += FlagManager->GetFlagActor(Start)->SOFlag->Segment.Lenght / 2;
	return TotalPathLenght;
}

void ASkeletalNavMeshBoundsVolume::DebugDirectionality(int FlagID)
{
	AFlagActor* FlagActor = FlagManager->GetFlagActor(FlagID);
	FVector Begin = FlagActor->SOFlag->Segment.BeginPosition;
	FVector End = FlagActor->SOFlag->Segment.EndPosition;
	FVector AdjustedBegin = Begin + (End - Begin) * 0.2f;
	FVector AdjustedEnd = Begin + (End - Begin) * 0.8f;

	switch (FlagActor->SOFlag->Segment.Direction)
	{
	case EFlagDirection::NONE:
		DrawDebugLine(
			GetWorld(),
			FlagActor->SOFlag->Segment.BeginPosition,
			FlagActor->SOFlag->Segment.EndPosition,
			FColor::Red,
			true,
			300
		);
		break;
	case EFlagDirection::BEGIN_END:
		DrawDebugDirectionalArrow(
			GetWorld(),
			AdjustedBegin,
			AdjustedEnd,
			500,
			FColor::Yellow,
			true,
			300
		);
		break;
	case EFlagDirection::END_BEGIN:
		DrawDebugDirectionalArrow(
			GetWorld(),
			AdjustedEnd,
			AdjustedBegin,
			500,
			FColor::Yellow,
			true,
			300
		);
		break;
	case EFlagDirection::BOTH:
		DrawDebugDirectionalArrow(
			GetWorld(),
			AdjustedBegin,
			AdjustedEnd,
			1000,
			FColor::Green,
			true,
			300
		);
		DrawDebugDirectionalArrow(
			GetWorld(),
			AdjustedEnd,
			AdjustedBegin,
			1000,
			FColor::Green,
			true,
			300
		);
		break;
	case EFlagDirection::IMPOSSIBLE:
		DrawDebugLine(
			GetWorld(),
			FlagActor->SOFlag->Segment.BeginPosition,
			FlagActor->SOFlag->Segment.EndPosition,
			FColor::Black,
			true,
			300
		);
		break;
	default: ;
	}
}

//GARD PATH FUNCTION
/*
	 * This function search recursively among all node from a starting point until 2 conditions
	 * are met, minimal length and desired difficulty. Length is prescriptive, difficulty is indicative.
*/
bool ASkeletalNavMeshBoundsVolume::PathMoreThanKUtil(int Source, int KLenght, TArray<int>& Path, int& Goal)
{
	/*security, prevent infinite stack*/
	KLenghtIterations++;
	if (KLenghtIterations > MaxKLenghtIterationsMod * KLengthTarget)
		return true;

	// Minimal Lenght reached, unfold all recursivity
	if (KLenght <= 0)
		return true;

	TArray<int> SourceNeighbors;
	AFlagActor* SourceFlag = FlagManager->GetFlagActor(Source);

	//Filter : Avoid backtracking 
	bool HasPassedThroughBegin = false;
	bool HasPassedThroughEnd = false;
	for (int Segment : SourceFlag->SOFlag->BeginPointIds)
	{
		if (Path.Contains(Segment))
		{
			HasPassedThroughBegin = true;
			break;
		}
	}
	for (int Segment : SourceFlag->SOFlag->EndPointIds)
	{
		if (Path.Contains(Segment))
		{
			HasPassedThroughEnd = true;
			break;
		}
	}
	if (HasPassedThroughBegin && HasPassedThroughEnd)
		return false;

	if (!HasPassedThroughBegin)
		SourceNeighbors.Append(SourceFlag->SOFlag->BeginPointIds);
	if (!HasPassedThroughEnd)
		SourceNeighbors.Append(SourceFlag->SOFlag->EndPointIds);

	for (int i = 0; i < SourceNeighbors.Num(); i++)
	{
		int NeighborsId = SourceNeighbors[i];
		AFlagActor* Neighbour = FlagManager->GetFlagActor(NeighborsId);
		int NeighborWeight = Neighbour->SOFlag->Segment.Lenght;

		if (Path[NeighborsId] != -1)
			continue;

		bool VisibilityFilter = false;
		for (int j = 0; j < Neighbour->SOFlag->Segment.VisibilityGroups.Num(); j++)
		{
			AFlagActor* NeighborsVisFlag = FlagManager->GetFlagActor(Neighbour->SOFlag->Segment.VisibilityGroups[j]);
			if (NeighborsVisFlag->SOFlag->Segment.FlagType == EFlagType::SAFE)
			{
				VisibilityFilter = true;
				break;
			}
		}

		if (VisibilityFilter)
			continue;

		if (Neighbour->SOFlag->Segment.PathType == EFlagPathType::GOLDEN)
		{
			if (!AreSameChallengeGroup(Source, NeighborsId))
			{
				continue;
			}
		}

		if (NeighborWeight >= KLenght)
		{
			Goal = NeighborsId;
			Path[NeighborsId] = Source;
			return true;
		}

		Path[NeighborsId] = Source;

		if (PathMoreThanKUtil(NeighborsId, KLenght - NeighborWeight, Path, Goal))
			return true;

		Path[NeighborsId] = -1;
	}
	return false;
}

bool ASkeletalNavMeshBoundsVolume::GuardPathMoreThanKGenerator(int Source, int KLenght, FVector2d VERT,
                                                               TArray<int>& Path, int& End)
{
	/*security, prevent infinite stack*/
	KLenghtIterations++;
	if (KLenghtIterations > MaxKLenghtIterationsMod * KLengthTarget)
		return true;

	// Target Lenght reached, unfold all recursivity
	if (KLenght <= 0)
		return true;

	TArray<int> SourceNeighbors;
	AFlagActor* SourceFlag = FlagManager->GetFlagActor(Source);

	//Filter : Avoid backtracking 
	if (!CreateSourceNeighbourFromFilters(SourceFlag, Path, SourceNeighbors))
		return false;

	SortByMostDesirableRatio(SourceNeighbors, SourceFlag);

	for (int i = 0; i < SourceNeighbors.Num(); i++)
	{
		int NeighborsId = SourceNeighbors[i];
		AFlagActor* Neighbour = FlagManager->GetFlagActor(NeighborsId);

		int NeighborWeight = Neighbour->SOFlag->Segment.Lenght;

		if (Path[NeighborsId] != -1)
			continue;

		bool VisibilityFilter = false;
		for (int j = 0; j < Neighbour->SOFlag->Segment.VisibilityGroups.Num(); j++)
		{
			AFlagActor* NeighborsVisFlag = FlagManager->GetFlagActor(Neighbour->SOFlag->Segment.VisibilityGroups[j]);
			if (NeighborsVisFlag->SOFlag->Segment.FlagType == EFlagType::SAFE)
			{
				VisibilityFilter = true;
				break;
			}
		}

		if (VisibilityFilter)
			continue;

		if (Neighbour->SOFlag->Segment.PathType == EFlagPathType::GOLDEN)
		{
			if (!AreSameChallengeGroup(Source, NeighborsId))
			{
				continue;
			}
		}

		if (NeighborWeight >= KLenght)
		{
			End = NeighborsId;
			Path[NeighborsId] = Source;
			return true;
		}

		Path[NeighborsId] = Source;

		if (GuardPathMoreThanKGenerator(NeighborsId, KLenght - NeighborWeight, VERT, Path, End))
			return true;

		Path[NeighborsId] = -1;
	}
	return false;
}

bool ASkeletalNavMeshBoundsVolume::CreateSourceNeighbourFromFilters(
	AFlagActor* SourceFlag, const TArray<int> Path, TArray<int>& OutSourceNeighbour)
{
	bool HasPassedThroughBegin = false;
	bool HasPassedThroughEnd = false;
	for (int Segment : SourceFlag->SOFlag->BeginPointIds)
	{
		if (Path.Contains(Segment))
		{
			HasPassedThroughBegin = true;
			break;
		}
	}
	for (int Segment : SourceFlag->SOFlag->EndPointIds)
	{
		if (Path.Contains(Segment))
		{
			HasPassedThroughEnd = true;
			break;
		}
	}
	if (HasPassedThroughBegin && HasPassedThroughEnd)
		return false;

	if (!HasPassedThroughBegin)
		OutSourceNeighbour.Append(SourceFlag->SOFlag->BeginPointIds);
	if (!HasPassedThroughEnd)
		OutSourceNeighbour.Append(SourceFlag->SOFlag->EndPointIds);

	return true;
}

void ASkeletalNavMeshBoundsVolume::SortByMostDesirableRatio(TArray<int>& OutSourceNeighbors, AFlagActor* Source)
{
	struct FSortByAngle
	{
		FSortByAngle(const FVector& InSourceLocation)
			: SourceDirection(InSourceLocation)
		{
		}

		/* The Location to use in our Sort comparision. */
		FVector SourceDirection;

		bool operator()(const AActor& A, const AActor& B) const
		{
			FVector SourceToNeighborA = A.GetActorLocation() - SourceDirection;
			SourceToNeighborA.Normalize();
			float AngleA = UE::Geometry::AngleD(SourceToNeighborA, SourceDirection);
			AngleA = UKismetMathLibrary::Abs(AngleA);
			
			FVector SourceToNeighborB = B.GetActorLocation() - SourceDirection;
			SourceToNeighborB.Normalize();
			float AngleB = UE::Geometry::AngleD(SourceToNeighborB, SourceDirection);
			AngleB = UKismetMathLibrary::Abs(AngleB);
			
			return AngleA < AngleB;
		}
	};
	if(OutSourceNeighbors.Num() <= 0)
		return;

	FVector DirectionVector;
	
	if (Source->SOFlag->BeginPointIds.Contains(OutSourceNeighbors[0]))
		DirectionVector = Source->SOFlag->Segment.EndPosition - Source->SOFlag->Segment.BeginPosition;
	else if (Source->SOFlag->EndPointIds.Contains(OutSourceNeighbors[0]))
		DirectionVector = Source->SOFlag->Segment.BeginPosition - Source->SOFlag->Segment.EndPosition;
	DirectionVector.Normalize();
	
	TArray<AFlagActor*> OutSourceActors;
	for (auto Element : OutSourceNeighbors)
	{
		OutSourceActors.Add(FlagManager->GetFlagActor(Element));
	}

	OutSourceActors.Sort(FSortByAngle(DirectionVector));

	OutSourceNeighbors.Empty();

	for (auto Element : OutSourceActors)
	{
		OutSourceNeighbors.Add(Element->SOFlag->Segment.id);
	}
}
