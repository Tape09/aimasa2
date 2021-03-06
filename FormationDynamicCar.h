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
	

	std::shared_ptr<Path> calc_path(FVector pos0, FVector vel0, FVector pos1, FVector vel1);
	void random_optimization();


	float formation_error();
	float formation_error(std::vector<std::vector<std::shared_ptr<RRTNode>>> & path, float t);

	float total_cost(std::vector<std::vector<std::shared_ptr<RRTNode>>> & path);
	
	void map_goals();

	int buffer_ticks = 100;

	float v_max;
	float a_max;

	float cost;

	float abs_time = 0;
	std::vector<float> t_now;	
	std::vector<float> pidx;		
	std::vector<bool> is_driving;
	
	std::unordered_map<int,int> start_goal_mapping;
	std::vector<std::vector<std::shared_ptr<RRTNode>>> my_path;

	int n_iterations = 20000;
	int n_agents;
	bool has_initialized = false;
	bool stop = false;
	AMapGen * map;
	
};
