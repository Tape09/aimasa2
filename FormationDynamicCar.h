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
#include "FormationDynamicCar.generated.h"

UCLASS()
class AIMASA2_API AFormationDynamicCar : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AFormationDynamicCar();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
	// Called every frame
	virtual void Tick( float DeltaSeconds ) override;

	void init();
	std::vector<std::shared_ptr<RRTNode>> my_path;
	std::shared_ptr<Path> calc_path(FVector pos0, FVector vel0, FVector pos1, FVector vel1);

	int buffer_ticks = 100;

	float v_max;
	float a_max;

	float t_now = 0;
	float pidx = 0;
	bool is_driving = false;
	
	int n_agents;
	bool has_initialized = false;
	AMapGen * map;
	
};
