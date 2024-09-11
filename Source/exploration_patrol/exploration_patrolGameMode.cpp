// Copyright Epic Games, Inc. All Rights Reserved.

#include "exploration_patrolGameMode.h"
#include "exploration_patrolCharacter.h"
#include "UObject/ConstructorHelpers.h"

Aexploration_patrolGameMode::Aexploration_patrolGameMode()
{
	// set default pawn class to our Blueprinted character
	static ConstructorHelpers::FClassFinder<APawn> PlayerPawnBPClass(TEXT("/Game/ThirdPerson/Blueprints/BP_ThirdPersonCharacter"));
	if (PlayerPawnBPClass.Class != NULL)
	{
		DefaultPawnClass = PlayerPawnBPClass.Class;
	}
}
