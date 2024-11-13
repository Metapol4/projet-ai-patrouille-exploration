#pragma once
#include "CoreMinimal.h"


UENUM(BlueprintType)
enum class EFlagType : uint8
{
	NONE,
	SAFE,
	RISKY,
	CHALLENGE
};
UENUM(BlueprintType)
enum class EFlagPathType : uint8
{
	NONE,
	GOLDEN,
	ALTERNATIVE
};
UENUM(BlueprintType)
enum class EFlagDirection : uint8
{
	NONE,
	BEGIN_END,
	END_BEGIN,
	BOTH
};

