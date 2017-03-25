// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "MapGen.h"
#include "Car.h"
#include "Item.h"
#include "MyMath.h"
#include "PathPlanner_KP.h"
#include <unordered_set>
#include <vector>
#include "GuardHelper.h"
#include <iterator>
#include <algorithm>
#include <limits>
#include <chrono>
#include "ShortSearch.generated.h"




UCLASS()
class AIMASA2_API AShortSearch : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AShortSearch();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
	// Called every frame
	virtual void Tick( float DeltaSeconds ) override;

	struct PathSegment {
		FVector waypoint;
		float total_dist;
	};

	void two_opt_mtsp(std::vector<Guard*> & path);
	void two_opt_mtsp_random(std::vector<Guard*> & path, int ms_limit);
	void two_opt_swap1(std::vector<Guard*> & path, int idx1, int idx2);
	void two_opt_swap2(std::vector<Guard*> & path, int idx1, int idx2);

	float path_len(const std::vector<Guard*> & path);
	float edge_dist(Guard* g1, Guard* g2);
	
	void make_play_path(const std::vector<Guard*> & path);

	void init();

	void test_random_path();
	void test_2opt();

	void draw_path(const std::vector<Guard*> & path) const;

	std::vector<Guard> all_guards;

	int buffer_ticks = 100;
	bool has_initialized = false;

	int n_guards;
	int n_items;

	int n_sets = 10000;
	int n_iterations = 5000;
	float p_mut = 0.5;

	float t_now = 0;

	PathPlanner_KP planner;
	AMapGen * map;

	std::vector<Guard> start_guards;
	std::vector<Guard> generated_guards;
	std::vector<std::vector<Guard*>> final_path;
	std::vector< std::vector<PathSegment> > play_path;
	//std::vector< int> next_segment;

	std::unordered_map<std::pair<Guard*,Guard*>, float, std::hash<std::pair<Guard*,Guard*>>, UnorderedEqual> edge_dists;
};
