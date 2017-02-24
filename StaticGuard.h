// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "MapGen.h"
#include "Car.h"
#include "Item.h"
#include "MyMath.h"
#include <unordered_set>
#include <vector>
#include "StaticGuard.generated.h"





struct IntSet {
	std::unordered_set<int> ints;

	IntSet operator-(const IntSet & other) {
		IntSet out;
		for (const int & i : ints) {
			if (other.ints.count(i) == 0) {
				out.ints.insert(i);
			}
		}
		return out;
	}

	IntSet operator+(const IntSet & other) {
		IntSet out;
		out.ints.insert(ints.begin(), ints.end());
		out.ints.insert(other.ints.begin(),other.ints.end());
		return out;
	}

	IntSet & operator+=(const IntSet & other) {
		ints.insert(other.ints.begin(), other.ints.end());
		return *this;
	}

	IntSet & operator-=(const IntSet & other) {
		for (const int & i : other.ints) {
			ints.erase(i);
		}
		return *this;
	}


};

struct Guard {
	FVector pos;
	IntSet items;
};

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
