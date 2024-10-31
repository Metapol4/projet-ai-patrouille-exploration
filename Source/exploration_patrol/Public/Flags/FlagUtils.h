#pragma once
#include "CoreMinimal.h"


UENUM(BlueprintType)
enum class EFlagType : uint8
{
	NONE,
	SAFE,
	RISKY,
	CHALLENGE,
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

