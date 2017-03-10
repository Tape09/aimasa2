// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "MapGen.h"
#include "Car.h"
#include "Item.h"
#include "MyMath.h"
#include <unordered_set>
#include <vector>
#include "GuardHelper.h"
#include "StaticGuard.generated.h"



UCLASS()
class AIMASA2_API AStaticGuard : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AStaticGuard();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
	// Called every frame
	virtual void Tick( float DeltaSeconds ) override;

	void init();

	std::vector<Guard> guards;
	bool has_initialized = false;
	int n_guards;
	const int n_sets = 1000;
	AMapGen * map;
	
};
