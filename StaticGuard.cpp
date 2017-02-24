// Fill out your copyright notice in the Description page of Project Settings.

#include "aimasa2.h"
#include "StaticGuard.h"


// Sets default values
AStaticGuard::AStaticGuard()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void AStaticGuard::BeginPlay()
{
	Super::BeginPlay();

	const UWorld * world = GetWorld();

	if (world) {
		FActorSpawnParameters spawnParams;
		spawnParams.Owner = this;
		spawnParams.Instigator = Instigator;

		map = GetWorld()->SpawnActor<AMapGen>(FVector(0, 0, 0), FRotator::ZeroRotator, spawnParams);
	}

	n_guards = map->startPoints.Num();

	print("Map initializing...", 50);
	print_log("Map initializing...");	
}

// Called every frame
void AStaticGuard::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );
	if (!has_initialized) {
		init();
		has_initialized = true;
		print("Map initialized!", 50);
		print_log("Map initialized!");
	}
}

void AStaticGuard::init() {
	// generate sets
	std::vector<Guard> generated_guards;

	for (int i = 0; i < n_sets; ++i) {
		int rand_idx = FMath::RandRange(0, map->itemPoints.Num() - 1);
		FVector base = map->itemPoints[rand_idx];

		float random_angle;
		float random_radius;

		FVector random_direction;
		FVector random_point;

		do {
			random_angle = FMath::RandRange(0.0, twopi);
			random_direction = FVector(cos(random_angle), sin(random_angle), 0.0);
			random_radius = FMath::RandRange(0.0, map->sensor_range);
			random_point = base + random_radius*random_direction;
		} while (isInAnyPolygon(random_point, map->allGroundPoints) || !isInPolygon(random_point, map->wallPoints));

		// get all items within sensor range
		IntSet items_in_sight;
		for (int j = 0; j < map->itemPoints.Num(); ++j) {
			if (FVector::DistSquared(random_point, map->itemPoints[j]) <= map->sensor_range2) {
				items_in_sight.ints.insert(j);
			}
		}

		// remove items not in line of sight
		IntSet items_not_in_sight;
		for (const int & j : items_in_sight.ints) {
			if (!map->Trace(random_point, map->itemPoints[j])) {
				items_not_in_sight.ints.insert(j);
			}
		}
		items_in_sight -= items_not_in_sight;

		Guard g;
		g.pos = random_point;
		g.items = items_in_sight;

		generated_guards.push_back(g);
	}

	// greedy set cover
	IntSet all_covered_items;

	for (int i = 0; i < n_guards; ++i) {
		int best_guard = 0;
		int best_n_new_items = -1;

		for (int j = 0; j < generated_guards.size(); ++j) {
			int n_new_items = (generated_guards[j].items - all_covered_items).ints.size();
			if (n_new_items > best_n_new_items) {
				best_guard = j;
				best_n_new_items = n_new_items;
			}
		}
		all_covered_items += generated_guards[best_guard].items;
		guards.push_back(generated_guards[best_guard]);
	}

	print_log("GUARDS:");
	for (int i = 0; i < n_guards; ++i) {
		map->cars[i]->SetActorLocation(guards[i].pos);
		map->drawCircle(guards[i].pos,map->sensor_range);
		print_log(guards[i].pos);
	}
	print_log("=====");


	print_log("UNCOVERED");
	for (int i = 0; i < map->items.Num(); ++i) {
		if (all_covered_items.ints.count(i) == 0) {
			map->items[i]->SetActorHiddenInGame(true);
			print_log(map->itemPoints[i]);
			map->drawPoint(map->itemPoints[i]);
		}
	}
	print_log("=====");

	int n_items = map->itemPoints.Num();
	int n_covered = all_covered_items.ints.size();

	print_log("COVERED: " + FString::FromInt(n_covered) + "/" + FString::FromInt(n_items));

}