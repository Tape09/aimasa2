// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "MapGen.h"
#include "Car2.h"
#include "DynamicCarPaths.h"
#include "MyMath.h"
#include <memory>
#include "RRT.h"
#include "Path.h"
#include "DCController.generated.h"

UCLASS()
class AIMASA2_API ADCController : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ADCController();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
	// Called every frame
	virtual void Tick( float DeltaSeconds ) override;

	void init();

	int buffer_ticks = 100;
	bool has_initialized = false;

	float v_max;
	float a_max;

	float t_now = 0;
	float pidx = 0;
	bool is_driving = false;

	std::vector<std::shared_ptr<RRTNode>> my_path;
	std::shared_ptr<Path> calc_path(FVector pos0, FVector vel0, FVector pos1, FVector vel1);


	AMapGen * map;
};
