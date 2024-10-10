// Fill out your copyright notice in the Description page of Project Settings.


#include "SO_Flag.h"

USO_Flag::USO_Flag()
{ // FIXME: spline spawns at world 0,0,0. idk why but its not important for now
	Spline = CreateDefaultSubobject<USplineComponent>("PatrolSpline");
}
