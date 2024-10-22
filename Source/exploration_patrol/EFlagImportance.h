// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "EFlagImportance.generated.h"

UENUM()
enum class EFlagImportance : uint8
{
	VERY_LOW,
	LOW,
	NORMAL,
	HIGH,
	VERY_HIGH
};
