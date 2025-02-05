#include "SkeletalNavMeshBoundsVolume.h"

#include "DataTypeUtils.h"
#include "VectorTypes.h"
#include "Kismet/KismetMathLibrary.h"
#include <ctime>
#include <fstream>
#include <iostream>

void ASkeletalNavMeshBoundsVolume::GenerateAll()
{
	std::clock_t c_startall = std::clock();
	std::clock_t c_start = std::clock();
	/*ComputeGeometry();
	std::clock_t c_endgeo = std::clock();
	float time_elapsed_ms = 1000.0 * (c_endgeo - c_start) / CLOCKS_PER_SEC;
	TimeToGenerateComputeGeometry = time_elapsed_ms / 1000;
	c_start = std::clock();
	SendFlagBatch();
	std::clock_t c_endflag = std::clock();
	float time_elapsed_msflag = 1000.0 * (c_endflag - c_start) / CLOCKS_PER_SEC;
	TimeToGenerateSendFlagBatch = time_elapsed_msflag / 1000;*/
	FlagManager->ResetAllFlags();
	c_start = std::clock();
	CalculateVisionGroups();
	std::clock_t c_endvisi = std::clock();
	float time_elapsed_msvisi = 1000.0 * (c_endvisi - c_start) / CLOCKS_PER_SEC;
	TimeToGenerateCalculateVisionGroups = time_elapsed_msvisi / 1000;
	c_start = std::clock();
	FindSafeSegments();
	std::clock_t c_endsafe = std::clock();
	float time_elapsed_mssafe = 1000.0 * (c_endsafe - c_start) / CLOCKS_PER_SEC;
	TimeToGenerateFindSafeSegments = time_elapsed_mssafe / 1000;
	c_start = std::clock();
	CalculateDirectionnality(EFlagType::SAFE);
	std::clock_t c_enddir = std::clock();
	float time_elapsed_msdir = 1000.0 * (c_enddir - c_start) / CLOCKS_PER_SEC;
	TimeToGenerateCalculateDirectionnality = time_elapsed_msdir / 1000;
	c_start = std::clock();
	GenerateGuardPathsUntilFail();
	std::clock_t c_endguard = std::clock();
	float time_elapsed_msguard = 1000.0 * (c_endguard - c_start) / CLOCKS_PER_SEC;
	TimeToGenerateGenerateGuardPathsUntilFail = time_elapsed_msguard / 1000;
	std::clock_t c_end = std::clock();
	float time_elapsed_msall = 1000.0 * (c_end - c_startall) / CLOCKS_PER_SEC;
	TimeToGenerate = time_elapsed_msall / 1000;
}

void ASkeletalNavMeshBoundsVolume::TestScenario()
{
	float SumTimeToGenerate = 0;
	float SumPlayerPathLength = 0;
	float SumNbOfGuards = 0;
	TArray<float> AllTimeToGenerate;
	TArray<float> AllPlayerPathLength;
	TArray<float> AllNbOfGuards;

	for (int i = 0; i < NbOfGenerations; i++)
	{
		GenerateAll();
		SumTimeToGenerate += TimeToGenerate;
		SumPlayerPathLength += PlayerPathLength;
		SumNbOfGuards += NbOfGuards;
		AllTimeToGenerate.Add(TimeToGenerate);
		AllPlayerPathLength.Add(PlayerPathLength);
		AllNbOfGuards.Add(NbOfGuards);
	}
	AVGTimeToGenerate = SumTimeToGenerate / NbOfGenerations;
	AVGPlayerPathLength = SumPlayerPathLength / NbOfGenerations;
	AVGNbOfGuards = SumNbOfGuards / NbOfGenerations;
	AllTimeToGenerate.Sort();
	AllPlayerPathLength.Sort();
	AllNbOfGuards.Sort();
	MINTimeToGenerate = AllTimeToGenerate[0];
	MAXTimeToGenerate = AllTimeToGenerate[AllTimeToGenerate.Num() - 1];
	int MedianIndex = AllTimeToGenerate.Num() / 2;
	MEDTimeToGenerate = AllTimeToGenerate[MedianIndex];

	MINPlayerPathLength = AllPlayerPathLength[0];
	MAXPlayerPathLength = AllPlayerPathLength[AllPlayerPathLength.Num() - 1];
	MedianIndex = AllPlayerPathLength.Num() / 2;
	MEDPlayerPathLength = AllPlayerPathLength[MedianIndex];

	MINNbOfGuards = AllNbOfGuards[0];
	MAXNbOfGuards = AllNbOfGuards[AllNbOfGuards.Num() - 1];
	MedianIndex = AllNbOfGuards.Num() / 2;
	MEDNbOfGuards = AllNbOfGuards[MedianIndex];
}

void ASkeletalNavMeshBoundsVolume::WriteTestScenariosToCSV()
{
	if (ScenarioCSVResults.IsEmpty())
		return;
	/*FScenarioCSVLine TestStruct;
	TestStruct.ID = 69;
	// parameters
	TestStruct.MaxNbOfGuards = 5.2f;
	TestStruct.KlengthTarget = 8000.6f;
	TestStruct.PercentStartingSelection = 45.3f;
	TestStruct.PercentSafe = 2.1f;
	TestStruct.ExplorationW = 98.5f;
	TestStruct.LinearW = 89.7f;
	TestStruct.AngleTolerance = 180.8f;
	TestStruct.GuardVisionRange = 99999.9f;
	// metrics

	TestStruct.GenerationTime = 5.9f;
	TestStruct.PlayerPathLength = 78222.3f;
	TestStruct.GeneratedGuardNB = 45.6f;*/
	std::ofstream myfile;
	myfile.open("D:/scenarioresults.csv");
	myfile <<
		"ID,"
		"Average Generation Time,Average Player Distance,Average Number Of Generated Guards,"
		"Min Generation Time,Med Generation Time,Max Generation Time,"
		"Min Player Distance,Med Player Distance,Max Player Distance,"
		"Min Number Of Generated Guards,Med Number Of Generated Guards,Max Number Of Generated Guards,"
		"MaxGuards,Klength,Percent Selection,Percent Safe,Exploration Weight,LinearWeight,AngleTolerance,GuardVisionRange\n";
	for (auto CurrentResult : ScenarioCSVResults)
	{
		FString Current = FString::SanitizeFloat(CurrentResult.ID);
		Current.Append(",");
		Current.Append(FString::SanitizeFloat(CurrentResult.AVGGenerationTime));
		Current.Append(",");
		Current.Append(FString::SanitizeFloat(CurrentResult.AVGPlayerPathLength));
		Current.Append(",");
		Current.Append(FString::SanitizeFloat(CurrentResult.AVGGeneratedGuardNB));
		Current.Append(",");
		Current.Append(FString::SanitizeFloat(CurrentResult.MINGenerationTime));
		Current.Append(",");
		Current.Append(FString::SanitizeFloat(CurrentResult.MEDGenerationTime));
		Current.Append(",");
		Current.Append(FString::SanitizeFloat(CurrentResult.MAXGenerationTime));
		Current.Append(",");
		Current.Append(FString::SanitizeFloat(CurrentResult.MINPlayerPathLength));
		Current.Append(",");
		Current.Append(FString::SanitizeFloat(CurrentResult.MEDPlayerPathLength));
		Current.Append(",");
		Current.Append(FString::SanitizeFloat(CurrentResult.MAXPlayerPathLength));
		Current.Append(",");
		Current.Append(FString::SanitizeFloat(CurrentResult.MINGeneratedGuardNB));
		Current.Append(",");
		Current.Append(FString::SanitizeFloat(CurrentResult.MEDGeneratedGuardNB));
		Current.Append(",");
		Current.Append(FString::SanitizeFloat(CurrentResult.MAXGeneratedGuardNB));
		Current.Append(",");
		Current.Append(FString::SanitizeFloat(CurrentResult.MaxNbOfGuards));
		Current.Append(",");
		Current.Append(FString::SanitizeFloat(CurrentResult.KlengthTarget));
		Current.Append(",");
		Current.Append(FString::SanitizeFloat(CurrentResult.PercentStartingSelection));
		Current.Append(",");
		Current.Append(FString::SanitizeFloat(CurrentResult.PercentSafe));
		Current.Append(",");
		Current.Append(FString::SanitizeFloat(CurrentResult.ExplorationW));
		Current.Append(",");
		Current.Append(FString::SanitizeFloat(CurrentResult.LinearW));
		Current.Append(",");
		Current.Append(FString::SanitizeFloat(CurrentResult.AngleTolerance));
		Current.Append(",");
		Current.Append(FString::SanitizeFloat(CurrentResult.GuardVisionRange));
		Current.Append("\n");

		myfile << std::string(TCHAR_TO_UTF8(*Current));
	}
	myfile.close();
}

void ASkeletalNavMeshBoundsVolume::TestAllScenario()
{
	ScenarioCSVResults.Empty();
	TArray<TArray<float>> ValueMatrix;
	TArray<float> ArrayColumn;
	ArrayColumn.Init(0, 3);
	//ValueMatrix.Init(ArrayColumn, 8);
	ValueMatrix.Init(ArrayColumn, 4);
	/*low*/
	ValueMatrix[0][0] = 2;
	ValueMatrix[1][0] = 1000;
	ValueMatrix[2][0] = 10;
	ValueMatrix[3][0] = 5;
	//ValueMatrix[4][0] = 0;
	//ValueMatrix[5][0] = 0;
	//ValueMatrix[6][0] = 30;
	//ValueMatrix[7][0] = 200;
	/*base*/
	ValueMatrix[0][1] = 4;
	ValueMatrix[1][1] = 2000;
	ValueMatrix[2][1] = 50;
	ValueMatrix[3][1] = 10;
	//ValueMatrix[4][1] = 1;
	//ValueMatrix[5][1] = 1;
	//ValueMatrix[6][1] = 45;
	//ValueMatrix[7][1] = 750;
	/*high*/
	ValueMatrix[0][2] = 10;
	ValueMatrix[1][2] = 4000;
	ValueMatrix[2][2] = 90;
	ValueMatrix[3][2] = 20;
	//ValueMatrix[4][2] = 2;
	//ValueMatrix[5][2] = 2;
	//ValueMatrix[6][2] = 90;
	//ValueMatrix[7][2] = 2000;

	int a, b, c, d = 0; //, e, f, g, h = 0;

	for (int i = 0; i < 81 /*6561*/; i++)
	{
		UE_LOG(LogTemp, Warning, TEXT("SCNRIO: progress:  %d"), i);

		a = i % 3;
		b = FMathf::Floor(i / 3);
		b %= 3;
		c = FMathf::Floor(i / 9);
		c %= 3;
		d = FMathf::Floor(i / 27);
		d %= 3;
		/*e = FMathf::Floor(i / 81);
		e %= 3;
		f = FMathf::Floor(i / 243);
		f %= 3;
		g = FMathf::Floor(i / 729);
		g %= 3;
		h = FMathf::Floor(i / 2187);
		h %= 3;*/

		float ScenGuardNB = ValueMatrix[0][a];
		float ScenKlength = ValueMatrix[1][b];
		float ScenPercentSelect = ValueMatrix[2][c];
		float ScenPercentSafe = ValueMatrix[3][d];
		/*float ScenExploration = ValueMatrix[4][e];
		float ScenLinear = ValueMatrix[5][f];
		float ScenAngle = ValueMatrix[6][g];
		float ScenVisRange = ValueMatrix[7][h];*/

		MaxGuardNb = ScenGuardNB;
		KLengthTarget = ScenKlength;
		PercentageRandomStartingPointSelection = ScenPercentSelect;
		PercentageSafeSegment = ScenPercentSafe;
		/*ExplorationWeight = ScenExploration;
		LinearWeight = ScenLinear;
		AngleTolerance = ScenAngle;
		FlagManager->GuardVisionRange = ScenVisRange;*/

		TestScenario();
		FScenarioCSVLine NewLine;
		NewLine.ID = i;
		// parameters
		NewLine.MaxNbOfGuards = MaxGuardNb;
		NewLine.KlengthTarget = KLengthTarget;
		NewLine.PercentStartingSelection = PercentageRandomStartingPointSelection;
		NewLine.PercentSafe = PercentageSafeSegment;
		NewLine.ExplorationW = ExplorationWeight;
		NewLine.LinearW = LinearWeight;
		NewLine.AngleTolerance = AngleTolerance;
		NewLine.GuardVisionRange = FlagManager->GuardVisionRange;
		// metrics

		NewLine.AVGGenerationTime = AVGTimeToGenerate;
		NewLine.AVGPlayerPathLength = AVGPlayerPathLength;
		NewLine.AVGGeneratedGuardNB = AVGNbOfGuards;

		NewLine.MEDGenerationTime = MEDTimeToGenerate;
		NewLine.MEDPlayerPathLength = MEDPlayerPathLength;
		NewLine.MEDGeneratedGuardNB = MEDNbOfGuards;

		NewLine.MINGenerationTime = MINTimeToGenerate;
		NewLine.MINPlayerPathLength = MINPlayerPathLength;
		NewLine.MINGeneratedGuardNB = MINNbOfGuards;

		NewLine.MAXGenerationTime = MAXTimeToGenerate;
		NewLine.MAXPlayerPathLength = MAXPlayerPathLength;
		NewLine.MAXGeneratedGuardNB = MaxGuardNb;

		ScenarioCSVResults.Add(NewLine);
		/*FString Current = FString::SanitizeFloat(scen0);
		Current.Append(" - ");
		Current.Append(FString::Printf(TEXT("%f"), scen1));
		Current.Append(" - ");
		Current.Append(FString::Printf(TEXT("%f"), scen2));
		Current.Append(" - ");
		Current.Append(FString::Printf(TEXT("%f"), scen3));
		Current.Append(" - ");
		Current.Append(FString::Printf(TEXT("%f"), scen4));
		Current.Append(" - ");
		Current.Append(FString::Printf(TEXT("%f"), scen5));
		Current.Append(" - ");
		Current.Append(FString::Printf(TEXT("%f"), scen6));
		Current.Append(" - ");
		Current.Append(FString::Printf(TEXT("%f"), scen7));
		UE_LOG(LogTemp, Warning, TEXT("SCNRO: %s"), *Current);*/
	}
	WriteTestScenariosToCSV();
}

void ASkeletalNavMeshBoundsVolume::BeginPlay()
{
	Super::BeginPlay();
	ComputeGeometry();
	SendFlagBatch();
}

//CONSOLE FUNCTION
void ASkeletalNavMeshBoundsVolume::ClearDebugLine()
{
	FlushPersistentDebugLines(GetWorld());
	FlagManager->ResetAllDebugTexts();
}

void ASkeletalNavMeshBoundsVolume::DeterminePlayerPathLength()
{
	PlayerPathLength = 0;
	if (PlayerPath.IsEmpty())
		return;
	for (int FlagId : PlayerPath)
	{
		AFlagActor* ChallengeFlag = FlagManager->GetFlagActor(FlagId);

		PlayerPathLength += ChallengeFlag->SOFlag->Segment.Lenght;
	}
}

void ASkeletalNavMeshBoundsVolume::ComputeGeometry()
{
	int CurrentID = 0;
	ClearDebugLine();
	if (!NavMesh) return;
	if (!StartPointIndicator) return;
	if (!EndPointIndicator) return;
	FlagSegments.Empty();
	PolyArray.Empty();

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
			/*DrawDebugSphere(
				GetWorld(),
				SegmentBeginPoint,
				5,
				12,
				FColor::Cyan,
				true,
				300);*/
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
						/*DrawDebugLine(
							GetWorld(),
							SegmentBeginPoint,
							AdjustedLocation,
							FColor::Red,
							true,
							300
						);*/
					}
				}
			}
		}
	}
}


void ASkeletalNavMeshBoundsVolume::SendFlagBatch()
{
	if (FlagManager)
	{
		//UE_LOG(LogTemp, Warning, TEXT("FLGBTCH: flag segments length %d"), FlagSegments.Num());
		FlagManager->ReceiveSegmentBatch(FlagSegments);
	}
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

void ASkeletalNavMeshBoundsVolume::FindSafeSegments()
{
	ClearDebugLine();
	FindBeginAndEndFlags();

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
		/*DrawDebugLine(
			GetWorld(),
			TemporaryFlagList[i]->SOFlag->Segment.BeginPosition,
			TemporaryFlagList[i]->SOFlag->Segment.EndPosition,
			FColor::Yellow,
			true,
			300
		);*/
	}
}

void ASkeletalNavMeshBoundsVolume::FindBeginAndEndFlags()
{
	int NbOfFlag = FlagManager->GetFlagActorSize();
	if (NbOfFlag == -1)
		return;

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
			if (FlagManager->GetFlagActor(id)->SOFlag->Segment.FlagType != FlagType)
				continue;

			FVector ActorToSafe = FlagManager->GetFlagActor(id)->GetActorLocation() - FlagActor->GetActorLocation();
			ActorToSafe.Normalize();

			//Test begin to end
			FVector BeginToEnd = FlagActor->SOFlag->Segment.EndPosition - FlagActor->SOFlag->Segment.BeginPosition;
			BeginToEnd.Normalize();

			float EvaluatedAngle = UE::Geometry::AngleD(ActorToSafe, BeginToEnd);
			if (EvaluatedAngle < AngleTolerance)
			{
				FlagActor->SOFlag->Segment.Direction = GetAdditiveFlagDirection(
					EFlagDirection::END_BEGIN, FlagActor->SOFlag->Segment.Direction);
			}

			//Test end to begin
			FVector EndToBegin = FlagActor->SOFlag->Segment.BeginPosition - FlagActor->SOFlag->Segment.EndPosition;
			EndToBegin.Normalize();

			EvaluatedAngle = UE::Geometry::AngleD(ActorToSafe, EndToBegin);
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


void ASkeletalNavMeshBoundsVolume::GenerateOneGuardPath()
{
	TArray<AFlagActor*> TemporaryFlagList = FlagManager->GetFlagActors();

	FilterAndSortOutAltAndBidirectional(TemporaryFlagList);
	FilterAllAlreadyInUse(TemporaryFlagList);

	int ListLimit =
		UKismetMathLibrary::FFloor(TemporaryFlagList.Num() * (PercentageRandomStartingPointSelection / 100.0f));
	if (ListLimit > TemporaryFlagList.Num())
		ListLimit = TemporaryFlagList.Num();

	int RandomIndex = UKismetMathLibrary::RandomIntegerInRange(0, ListLimit - 1);
	int StartingFlag = TemporaryFlagList[RandomIndex]->SOFlag->Segment.id;

	int EndingFlag = -1;

	TArray<int> EvaluatedGuardPath;
	EvaluatedGuardPath.Init(-1, FlagManager->GetFlagActorSize());
	EvaluatedGuardPath[StartingFlag] = StartingFlag;

	FlagCurrentlySeen.Init(-1, FlagManager->GetFlagActorSize());

	TArray<int> ReconstructedChallengePath;
	GuardKLenghtIterations = 0;
	if (!GuardPathMoreThanKGenerator(StartingFlag, KLengthTarget, FVector2d(1, 1), EvaluatedGuardPath, EndingFlag))
		return;

	if (EndingFlag < 0)
		return;

	AStarPathReconstructor(EvaluatedGuardPath, StartingFlag, EndingFlag, ReconstructedChallengePath);
	if (!ReconstructedChallengePath.IsEmpty())
	{
		ChallengePath.Add(ReconstructedChallengePath);
	}
	DrawChallengePaths();
}

void ASkeletalNavMeshBoundsVolume::GenerateGuardPathsUntilFail()
{
	EmptyChallengePath();
	if (MaxGuardNb <= 0)
	{
		FindPlayerPath();
		ClearDebugLine();
		DrawLatestPlayerPath();
		return;
	}

	int Iterations = 0;
	for (int i = 0; i < 10; i++)
	{
		GenerateOneGuardPath();
		bool Success = FindPlayerPath();
		if (!Success)
		{
			if (!ChallengePath.IsEmpty())
			{
				//UE_LOG(LogTemp, Warning, TEXT("pop"));
				PopChallengePath();
			}
		}
		else
			i = 0;

		Iterations++;
		if (Iterations >= 1000)
			break;
		if (ChallengePath.Num() >= MaxGuardNb && MaxGuardNb != 0)
			break;
	}
	NbOfGuards = ChallengePath.Num();
	ClearDebugLine();
	DrawLatestPlayerPath();
	DrawChallengePaths();
}

void ASkeletalNavMeshBoundsVolume::SimulateCurrentConfiguration()
{
	if (PlayerPath.IsEmpty())
		return;
	if (ChallengePath.IsEmpty())
		return;

	int MaxTimeStep = PlayerPath.Num() - 1;
	SimulationIterations = 0;
	SimulationDelegate.BindUFunction(this, "DrawNextStep", MaxTimeStep);
	GetWorld()->GetTimerManager().SetTimer(SimulationTimer, SimulationDelegate, SimulationTimeStep, true);
}

void ASkeletalNavMeshBoundsVolume::DrawNextStep(int MaxStep)
{
	//UE_LOG(LogTemp, Warning, TEXT("SIMUL : DRAW STEP # %d"), SimulationIterations)

	if (!PlayerPath.IsEmpty() && !ChallengePath.IsEmpty())
	{
		//Draw Player Position
		int PlayerIndex = MaxStep - SimulationIterations;
		if (PlayerIndex < 0)
		{
			//UE_LOG(LogTemp, Error, TEXT("Player index negative"))
			return;
		}
		AFlagActor* PlayerFlag = FlagManager->GetFlagActor(PlayerPath[PlayerIndex]);
		FVector PlayerPosition = PlayerFlag->GetActorLocation();
		DrawDebugSphere(
			GetWorld(),
			PlayerPosition,
			80.0f,
			12,
			FColor::Green,
			false,
			SimulationTimeStep * 0.95
		);

		//Draw Each Guard Position
		int iterator = 0;
		FlagManager->ResetAllDebugTexts();
		for (auto GuardPath : ChallengePath)
		{
			if (GuardPath.IsEmpty())
				continue;

			// Create cursor
			int Num = GuardPath.Num();
			int Modulo = Num * 2;

			int CurrentCursor = SimulationIterations % Modulo;
			int NextCursor = (SimulationIterations + 1) % Modulo;

			// Adjust backward
			if (CurrentCursor > Num - 1)
			{
				CurrentCursor = 2 * Num - CurrentCursor - 1;
			}
			if (NextCursor > Num - 1)
			{
				NextCursor = (2 * Num) - NextCursor - 1;
			}

			//Get Guard Flag
			AFlagActor* GuardFlag = FlagManager->GetFlagActor(GuardPath[CurrentCursor]);
			FString MainText = "Guard " + FString::FromInt(iterator);
			//GuardFlag->VisibilityGroupText->SetText(MainText);
			FString AdditiveText = FString::FromInt(CurrentCursor) + " / " + FString::FromInt(GuardPath.Num());
			//GuardFlag->VisibilityGroupText->AddText(AdditiveText);
			//GuardFlag->VisibilityGroupText->SetTextColor(FColor::Yellow);

			FVector GuardPosition = GuardFlag->GetActorLocation();

			//Draw guard position
			DrawDebugSphere(
				GetWorld(),
				GuardPosition,
				80.0f,
				12,
				FColor::White,
				false,
				SimulationTimeStep * 0.95
			);

			//Guard direction
			FVector GuardDirection = FVector(0, 1, 0);
			AFlagActor* NextGuardFlag = FlagManager->GetFlagActor(GuardPath[NextCursor]);
			FVector NextGuardPosition = NextGuardFlag->GetActorLocation();
			GuardDirection = NextGuardPosition - GuardPosition;

			//Draw guard sight
			DrawDebugCone(
				GetWorld(),
				GuardPosition + FVector(0, 0, 100),
				GuardDirection,
				750,
				FMath::DegreesToRadians(AngleTolerance),
				FMath::DegreesToRadians(45.0f),
				12,
				FColor::Red,
				false,
				SimulationTimeStep * 0.95,
				0,
				1.0f
			);
			iterator++;
		}
	}
	else
	{
		//UE_LOG(LogTemp, Error, TEXT("SIMUL : DRAW STEP # %d FAILED , LIST EMPTY"), SimulationIterations)
	}
	SimulationIterations++;
	if (SimulationIterations >= MaxStep)
	{
		GetWorld()->GetTimerManager().ClearTimer(SimulationTimer);
	}
}

bool ASkeletalNavMeshBoundsVolume::FindPlayerPath()
{
	CalculateGuardPathVisionTimeSteps();

	FindBeginAndEndFlags();

	TArray<AFlagActor*> TemporaryFlagList = FlagManager->GetFlagActors();

	TArray<int> EvaluatedGuardPath;
	EvaluatedGuardPath.Init(-1, FlagManager->GetFlagActorSize());
	EvaluatedGuardPath[GoldenStartingFlagId] = GoldenStartingFlagId;

	TArray<int> ReconstructedChallengePath;
	PlayerKLenghtIterations = 0;
	PlayerPathMoreThanKUntilGoal(GoldenStartingFlagId, 1, EvaluatedGuardPath,
	                             GoldenEndingFlagId);

	float Success = AStarPathReconstructor(EvaluatedGuardPath, GoldenStartingFlagId, GoldenEndingFlagId,
	                                       ReconstructedChallengePath);

	if (Success < 0)
		return false;

	PlayerPath.Empty();
	PlayerPath = ReconstructedChallengePath;
	DrawLatestPlayerPath();
	DeterminePlayerPathLength();
	return true;
}

void ASkeletalNavMeshBoundsVolume::DrawChallengePaths()
{
	for (TArray<int> CurrentChallenge : ChallengePath)
	{
		FColor MainColor;
		MainColor.R = UKismetMathLibrary::RandomIntegerInRange(0, 255);
		MainColor.G = UKismetMathLibrary::RandomIntegerInRange(0, 255);
		MainColor.B = UKismetMathLibrary::RandomIntegerInRange(0, 255);
		for (int ChallengePathID : CurrentChallenge)
		{
			AFlagActor* ChallengeFlag = FlagManager->GetFlagActor(ChallengePathID);
			if (UseDGuardPathColor)
				MainColor = DGuardPathColor;
			/*DrawDebugLine(
				GetWorld(),
				ChallengeFlag->SOFlag->Segment.BeginPosition,
				ChallengeFlag->SOFlag->Segment.EndPosition,
				MainColor,
				true,
				300
			);*/
		}
	}
}

void ASkeletalNavMeshBoundsVolume::DrawLatestPlayerPath()
{
	if (PlayerPath.IsEmpty())
		return;
	for (int FlagId : PlayerPath)
	{
		AFlagActor* ChallengeFlag = FlagManager->GetFlagActor(FlagId);
		FColor MainColor = FColor::Yellow;
		if (FlagId == GoldenStartingFlagId)
		{
			MainColor = FColor::Purple;
		}
		/*DrawDebugLine(
			GetWorld(),
			ChallengeFlag->SOFlag->Segment.BeginPosition,
			ChallengeFlag->SOFlag->Segment.EndPosition,
			MainColor,
			true,
			300
		);*/
	}
}

void ASkeletalNavMeshBoundsVolume::CalculateGuardPathVisionTimeSteps()
{
	FVector LastDir;
	for (int PathIndex = 0; PathIndex < ChallengePath.Num(); PathIndex++)
	{
		TArray<int> CurrentChallenge = ChallengePath[PathIndex];
		int Step = 0;
		// forwards
		for (int i = 0; i < CurrentChallenge.Num() - 1; i++)
		{
			AFlagActor* SelfFlag = FlagManager->GetFlagActor(CurrentChallenge[i]);
			AFlagActor* NextFlag = FlagManager->GetFlagActor(CurrentChallenge[i + 1]);

			FVector SelfToNextDir = SelfFlag->GetActorLocation() - NextFlag->GetActorLocation();
			SelfToNextDir.Normalize();
			LastDir = SelfToNextDir;

			CalculateNeighboursForTimeStep(SelfFlag, SelfToNextDir, Step, PathIndex);
			Step++;
		}
		//last check forwards
		AFlagActor* LastFlag = FlagManager->GetFlagActor(CurrentChallenge[CurrentChallenge.Num() - 1]);
		CalculateNeighboursForTimeStep(LastFlag, LastDir, Step, PathIndex);
		Step++;
		for (int i = CurrentChallenge.Num() - 1; i > 0; i--)
		{
			AFlagActor* SelfFlag = FlagManager->GetFlagActor(CurrentChallenge[i]);
			AFlagActor* NextFlag = FlagManager->GetFlagActor(CurrentChallenge[i - 1]);

			FVector SelfToNextDir = SelfFlag->GetActorLocation() - NextFlag->GetActorLocation();
			SelfToNextDir.Normalize();
			LastDir = SelfToNextDir;

			CalculateNeighboursForTimeStep(SelfFlag, SelfToNextDir, Step, PathIndex);
			Step++;
		}
		LastFlag = FlagManager->GetFlagActor(CurrentChallenge[0]);
		CalculateNeighboursForTimeStep(LastFlag, LastDir, Step, PathIndex);
		Step++;
	}
}

TArray<int> ASkeletalNavMeshBoundsVolume::PopChallengePath()
{
	int PathId = ChallengePath.Num() - 1;
	TArray<int> PoppedPath = ChallengePath[PathId];
	for (AFlagActor* ChallengePathFlag : FlagManager->GetFlagActors())
	{
		ChallengePathFlag->SOFlag->RemoveTimeStepGroup(PathId);
	}
	ChallengePath.RemoveAt(PathId);
	return PoppedPath;
}

void ASkeletalNavMeshBoundsVolume::EmptyChallengePath()
{
	while (ChallengePath.Num() > 0)
	{
		PopChallengePath();
	}
}


void ASkeletalNavMeshBoundsVolume::CalculateNeighboursForTimeStep(AFlagActor* SelfFlag, FVector Direction, int Step,
                                                                  int GuardPathId)
{
	TArray<int> SelfNeighbours = SelfFlag->SOFlag->Segment.VisibilityGroups;
	for (int j = 0; j < SelfNeighbours.Num(); j++)
	{
		AFlagActor* Neighbour = FlagManager->GetFlagActor(SelfNeighbours[j]);
		if (SelfNeighbours[j] == SelfFlag->SOFlag->Segment.id)
		{
			Neighbour->SOFlag->AddTimeStep(GuardPathId, Step);
			continue;
		}

		FVector SelfToNeighbour = SelfFlag->GetActorLocation() - Neighbour->GetActorLocation();
		SelfToNeighbour.Normalize();
		float EvaluatedAngle = UE::Geometry::AngleD(Direction, SelfToNeighbour);
		/*UE_LOG(LogTemp, Warning, TEXT("STEP : Step: %d, From: %d, to: %d, angle: %f"), Step,
		       SelfFlag->SOFlag->Segment.id, SelfNeighbours[j], EvaluatedAngle)*/

		if (EvaluatedAngle < AngleTolerance * 2)
		{
			Neighbour->SOFlag->AddTimeStep(GuardPathId, Step);
		}
	}
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

void ASkeletalNavMeshBoundsVolume::FilterAllAlreadyInUse(TArray<AFlagActor*>& TemporaryFlagList)
{
	for (TArray<int> GuardPath : ChallengePath)
	{
		for (auto FlagID : GuardPath)
		{
			AFlagActor* Flag = FlagManager->GetFlagActor(FlagID);
			if (TemporaryFlagList.Contains(Flag))
			{
				TemporaryFlagList.Remove(Flag);
			}
		}
	}
}

void ASkeletalNavMeshBoundsVolume::a01ComputeGeometry()
{
	ComputeGeometry();
}

void ASkeletalNavMeshBoundsVolume::a02SendFlagBatch()
{
	SendFlagBatch();
}

void ASkeletalNavMeshBoundsVolume::a03CalculateVisionGroups()
{
	CalculateVisionGroups();
	int FlagSelection = UKismetMathLibrary::RandomIntegerInRange(0, FlagManager->GetFlagActorSize());
	FlagManager->GetFlagActor(FlagSelection)->SeeVisionGroup();
}

void ASkeletalNavMeshBoundsVolume::a04FindSafeSegments()
{
	FindSafeSegments();
}

void ASkeletalNavMeshBoundsVolume::a05CalculateDirectionality()
{
	CalculateDirectionnality(EFlagType::SAFE);
}

void ASkeletalNavMeshBoundsVolume::a06GenerateGuardPaths()
{
	GenerateGuardPathsUntilFail();
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
			//UE_LOG(LogTemp, Warning, TEXT("CHLG : FAIL AT %d"), GoalID);
			return -1;
		}
		//UE_LOG(LogTemp, Warning, TEXT("%d"), GoalID);
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
		/*DrawDebugLine(
			GetWorld(),
			FlagActor->SOFlag->Segment.BeginPosition,
			FlagActor->SOFlag->Segment.EndPosition,
			FColor::Red,
			true,
			300
		);*/
		break;
	case EFlagDirection::BEGIN_END:
		/*DrawDebugDirectionalArrow(
			GetWorld(),
			AdjustedBegin,
			AdjustedEnd,
			500,
			FColor::Yellow,
			true,
			300
		);*/
		break;
	case EFlagDirection::END_BEGIN:
		/*DrawDebugDirectionalArrow(
			GetWorld(),
			AdjustedEnd,
			AdjustedBegin,
			500,
			FColor::Yellow,
			true,
			300
		);*/
		break;
	case EFlagDirection::BOTH:
		/*DrawDebugDirectionalArrow(
			GetWorld(),
			AdjustedBegin,
			AdjustedEnd,
			1000,
			FColor::Green,
			true,
			300
		);*/
		/*DrawDebugDirectionalArrow(
			GetWorld(),
			AdjustedEnd,
			AdjustedBegin,
			1000,
			FColor::Green,
			true,
			300
		);*/
		break;
	case EFlagDirection::IMPOSSIBLE:
		/*DrawDebugLine(
			GetWorld(),
			FlagActor->SOFlag->Segment.BeginPosition,
			FlagActor->SOFlag->Segment.EndPosition,
			FColor::Black,
			true,
			300
		);*/
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
	// SECURITY :  limit recursivity
	GuardKLenghtIterations++;

	TArray<FNeighbors> SourceNeighbors;
	AFlagActor* SourceFlag = FlagManager->GetFlagActor(Source);

	//Filter : Avoid backtracking 
	if (!CreateSourceNeighbourFromFilters(SourceFlag, Path, SourceNeighbors))
		return false;

	//Calculate Priority
	if (LinearWeight > 0)AddAngleToSortValue(SourceNeighbors, SourceFlag);

	if (ExplorationWeight > 0)
		AddExplorationBonusToSortValue(SourceNeighbors, SourceFlag, Path);

	//Sort 
	SourceNeighbors.Sort([](const FNeighbors& ip1, const FNeighbors& ip2)
	{
		return ip1.SortValue > ip2.SortValue;
	});

	//Test each neighbors in priority order
	for (int i = 0; i < SourceNeighbors.Num(); i++)
	{
		// Get Neighbors Actor
		int NeighborsId = SourceNeighbors[i].ID;
		AFlagActor* Neighbour = FlagManager->GetFlagActor(NeighborsId);
		int NeighborWeight = Neighbour->SOFlag->Segment.Lenght;

		//GO TO NEXT NEIGHBORS ...

		//	... if Neighbors already in path
		if (Path[NeighborsId] != -1)
			continue;

		// ... if Neighbors see a flag tagged SAFE
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

		// ... if neighbors is used by another guard
		bool OtherGuardFilter = false;
		for (TArray<int> GuardPath : ChallengePath)
		{
			if (GuardPath.Contains(NeighborsId))
			{
				OtherGuardFilter = true;
				break;
			}
		}
		if (OtherGuardFilter)
		{
			//UE_LOG(LogTemp, Warning, TEXT("CONTINUE : OTHER GUARD AT : %d"), NeighborsId)
			continue;
		}

		// PATH COMPLETED ...

		//	... if K lenght reached with next neighbors
		if (NeighborWeight >= KLenght)
		{
			//UE_LOG(LogTemp, Warning, TEXT("RETURN : DESIRED LENGHT REACHED"))
			End = NeighborsId;
			Path[NeighborsId] = Source;
			return true;
		}
		// ... if max iteration has been reached
		if (GuardKLenghtIterations > MaxKLenghtIterationsMod * KLengthTarget)
		{
			//UE_LOG(LogTemp, Warning, TEXT("RETURN : MAX ITERATION"))
			End = NeighborsId;
			Path[NeighborsId] = Source;
			return true;
		}

		// RECURSION 
		Path[NeighborsId] = Source;

		if (GuardPathMoreThanKGenerator(NeighborsId, KLenght - NeighborWeight, VERT, Path, End))
			return true;

		Path[NeighborsId] = -1;
	}
	return false;
}

bool ASkeletalNavMeshBoundsVolume::PlayerPathMoreThanKUntilGoal(int Source, int Step, TArray<int>& Path, int& Goal)
{
	/*security, prevent infinite stack*/
	PlayerKLenghtIterations++;

	TArray<FNeighbors> SourceNeighbors;
	AFlagActor* SourceFlag = FlagManager->GetFlagActor(Source);

	//Filter : Avoid backtracking 
	if (!CreateSourceNeighbourFromFilters(SourceFlag, Path, SourceNeighbors))
		return false;

	//Calculate Priority
	AddExitBonusToSortValue(SourceNeighbors);

	//Sort by Priority
	SourceNeighbors.Sort([](const FNeighbors& ip1, const FNeighbors& ip2)
	{
		return ip1.SortValue > ip2.SortValue;
	});

	for (int i = 0; i < SourceNeighbors.Num(); i++)
	{
		int NeighborsId = SourceNeighbors[i].ID;
		AFlagActor* Neighbour = FlagManager->GetFlagActor(NeighborsId);

		if (Path[NeighborsId] != -1)
			continue;

		bool VisibilityFilter = false;
		if (!Neighbour->SOFlag->Segment.StepGroups.IsEmpty())
		{
			for (FTimeStep StepGroup : Neighbour->SOFlag->Segment.StepGroups)
			{
				if (!ChallengePath.IsEmpty())
				{
					if (!ChallengePath[StepGroup.GuardPathId].IsEmpty())
					{
						int StepSize = ChallengePath[StepGroup.GuardPathId].Num() * 2;
						int LocalStep = Step % StepSize;
						if (StepGroup.SeenAtTimeSteps.Contains(LocalStep))
						{
							VisibilityFilter = true;
							break;
						}
					}
				}
			}
		}
		if (VisibilityFilter)
			continue;

		// Goal reached, unfold all recursively
		if (NeighborsId == Goal)
		{
			Path[NeighborsId] = Source;
			return true;
		}
		if (PlayerKLenghtIterations > 500000)
		{
			Path[NeighborsId] = Source;
			return true;
		}
		Path[NeighborsId] = Source;
		if (PlayerPathMoreThanKUntilGoal(NeighborsId, Step + 1, Path, Goal))
			return true;
		Path[NeighborsId] = -1;
	}
	return false;
}

bool ASkeletalNavMeshBoundsVolume::CreateSourceNeighbourFromFilters(
	AFlagActor* SourceFlag, const TArray<int> Path, TArray<FNeighbors>& OutSourceNeighbour)
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
	{
		for (auto ID : SourceFlag->SOFlag->BeginPointIds)
		{
			FNeighbors NewNeighbors;
			NewNeighbors.ID = ID;
			OutSourceNeighbour.Add(NewNeighbors);
		}
	}
	if (!HasPassedThroughEnd)
	{
		for (auto ID : SourceFlag->SOFlag->EndPointIds)
		{
			FNeighbors NewNeighbors;
			NewNeighbors.ID = ID;
			OutSourceNeighbour.Add(NewNeighbors);
		}
	}
	return true;
}

void ASkeletalNavMeshBoundsVolume::AddAngleToSortValue(TArray<FNeighbors>& OutSourceNeighbors, AFlagActor* Source)
{
	//Security
	if (OutSourceNeighbors.IsEmpty())
	{
		return;
	}

	//Source Direction
	FVector SourceDirection;
	if (Source->SOFlag->EndPointIds.Contains(OutSourceNeighbors[0].ID))
		SourceDirection = Source->SOFlag->Segment.EndPosition - Source->SOFlag->Segment.BeginPosition;
	else if (Source->SOFlag->BeginPointIds.Contains(OutSourceNeighbors[0].ID))
		SourceDirection = Source->SOFlag->Segment.BeginPosition - Source->SOFlag->Segment.EndPosition;
	SourceDirection.Normalize();

	//Evaluate each neighbors
	for (int i = 0; i < OutSourceNeighbors.Num(); i++)
	{
		AFlagActor* EvaluatedNeighbors = FlagManager->GetFlagActor(OutSourceNeighbors[i].ID);
		FVector SourceToNeighbor = EvaluatedNeighbors->GetActorLocation() - Source->GetActorLocation();
		SourceToNeighbor.Normalize();
		float Angle = UE::Geometry::AngleD(SourceToNeighbor, SourceDirection);
		Angle = UKismetMathLibrary::Abs(Angle);
		OutSourceNeighbors[i].SortValue += LinearWeight * (1.0f - (Angle / 180.0f));
	}
}

void ASkeletalNavMeshBoundsVolume::AddExplorationBonusToSortValue(TArray<FNeighbors>& OutSourceNeighbors,
                                                                  AFlagActor* Source, TArray<int>& CameFrom)
{
	TArray<float> NeighborsDistance;
	NeighborsDistance.Init(0, OutSourceNeighbors.Num());
	float MaxValue = .0f;

	for (int i = 0; i < OutSourceNeighbors.Num(); i++)
	{
		//GetNeighbors
		AFlagActor* NeighborFlag = FlagManager->GetFlagActor(OutSourceNeighbors[i].ID);
		//Distance entre le neighbors et la source
		float SourceToNeighbor = UE::Geometry::Distance(Source->GetActorLocation(), NeighborFlag->GetActorLocation());
		//Add to distance value
		NeighborsDistance[i] = SourceToNeighbor;

		//Remonte dans le came from depuis la source
		int Cursor = Source->SOFlag->Segment.id;
		int IterationCounter = 0;
		constexpr int MaxIteration = 5;
		while (CameFrom[Cursor] != Cursor || IterationCounter < MaxIteration)
		{
			IterationCounter++;
			Cursor = CameFrom[Cursor];
			AFlagActor* Flag = FlagManager->GetFlagActor(Cursor);
			NeighborsDistance[i] += UE::Geometry::Distance(NeighborFlag->GetActorLocation(), Flag->GetActorLocation());
		}

		if (NeighborsDistance[i] > MaxValue)
			MaxValue = NeighborsDistance[i];
	}


	for (int i = 0; i < OutSourceNeighbors.Num(); i++)
	{
		OutSourceNeighbors[i].SortValue += ExplorationWeight * NeighborsDistance[i] / MaxValue;
	}
}

void ASkeletalNavMeshBoundsVolume::AddExitBonusToSortValue(TArray<FNeighbors>& OutSourceNeighbors)
{
	TArray<float> NeighborsDistance;
	NeighborsDistance.Init(0, OutSourceNeighbors.Num());
	float MaxValue = .0f;

	for (int i = 0; i < OutSourceNeighbors.Num(); i++)
	{
		//GetNeighbors
		AFlagActor* NeighborFlag = FlagManager->GetFlagActor(OutSourceNeighbors[i].ID);
		//Get Exit
		AFlagActor* ExitFlag = FlagManager->GetFlagActor(GoldenEndingFlagId);
		//Distance entre le neighbors et la destination
		float NeighborToExit = UE::Geometry::Distance(NeighborFlag->GetActorLocation(), ExitFlag->GetActorLocation());
		//Add to distance value
		NeighborsDistance[i] = NeighborToExit;
		//Save highest value for normalization
		if (NeighborsDistance[i] > MaxValue)
			MaxValue = NeighborsDistance[i];
	}

	for (int i = 0; i < OutSourceNeighbors.Num(); i++)
	{
		OutSourceNeighbors[i].SortValue += 1 - (NeighborsDistance[i] / MaxValue);
	}
}
