// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "MapGen.h"
#include "Car.h"
#include "Item.h"
#include "MyMath.h"
#include <unordered_set>
#include <vector>
#include "RRT.h"
#include "DynamicCarPaths.h"
#include "Path.h"
#include "GuardHelper.h"
#include "FormationDynamicPoint.generated.h"

UCLASS()
class AIMASA2_API AFormationDynamicPoint : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AFormationDynamicPoint();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
	// Called every frame
	virtual void Tick( float DeltaSeconds ) override;

	
	int n_agents;
	bool has_initialized = false;
	AMapGen * map;
};
