// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class exploration_patrol : ModuleRules
{
	public exploration_patrol(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "EnhancedInput", "AIModule" });
	}
}
