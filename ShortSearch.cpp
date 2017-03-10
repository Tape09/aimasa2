// Fill out your copyright notice in the Description page of Project Settings.

#include "aimasa2.h"
#include "ShortSearch.h"


// Sets default values
AShortSearch::AShortSearch()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void AShortSearch::BeginPlay()
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
	n_items = map->itemPoints.Num();
	

	print("Map initializing...", 50);
	print_log("Map initializing...");
}

// Called every frame
void AShortSearch::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );
	if (!has_initialized) {
		init();
		has_initialized = true;
		print("Map initialized!", 50);
		print_log("Map initialized!");
	}

	if (buffer_ticks > 0) {
		--buffer_ticks;
		return;
	}


}

void AShortSearch::init() {
	planner = PathPlanner_KP(map);
	//planner.draw_vgraph();
	//test_random_path();
	//test_2opt();

	IntSet start_items;
	std::vector<Guard> start_guards;
	for (int i = 0; i < n_guards; ++i) {
		Guard g;
		g.pos = map->startPoints[i];
		g.is_start = true;

		// get all items within sensor range
		IntSet items_in_sight;
		for (int j = 0; j < map->itemPoints.Num(); ++j) {
			if (FVector::DistSquared(g.pos, map->itemPoints[j]) <= map->sensor_range2) {
				items_in_sight.ints.insert(j);
			}
		}

		// remove items not in line of sight
		IntSet items_not_in_sight;
		for (const int & j : items_in_sight.ints) {
			if (!map->Trace(g.pos, map->itemPoints[j])) {
				items_not_in_sight.ints.insert(j);
			}
		}
		items_in_sight -= items_not_in_sight;
		g.items = items_in_sight;
		start_items += items_in_sight;
		start_guards.push_back(g);
	}


	// create random guards
	std::vector<Guard> generated_guards;

	for (int i = 0; i < n_sets; ++i) {
		int rand_idx = FMath::RandRange(0, map->itemPoints.Num() - 1);
		FVector base = map->itemPoints[rand_idx];

		FVector random_point = map->randomPoint(base);

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

	all_guards = generated_guards;
	for (int i = 0; i < start_guards.size(); ++i) {
		all_guards.push_back(start_guards[i]);
	}

	// initial random cover
	IntSet all_items;
	for (int i = 0; i < n_items; ++i) {
		all_items.ints.insert(i);
	}

	IntSet lost_items = all_items - start_items;

	std::vector<Guard*> pgenerated_guards;
	pgenerated_guards.reserve(generated_guards.size());
	for (int i = 0; i < generated_guards.size(); ++i) {
		pgenerated_guards.push_back(&generated_guards[i]);
	}

	std::vector<Guard*> pstart_guards;
	pstart_guards.reserve(start_guards.size());
	for (int i = 0; i < start_guards.size(); ++i) {
		pstart_guards.push_back(&start_guards[i]);
	}

	std::vector<Guard*> new_path = randomCover(pgenerated_guards, lost_items);
	//std::vector<Guard*> nonstart_guards = rcover;


	new_path.insert(new_path.begin(),pstart_guards.begin(),pstart_guards.end());
	if(new_path.size() > 1) {
		std::random_shuffle(new_path.begin()+1,new_path.end());
	}

	std::vector<Guard*> old_path = new_path;
	std::vector<Guard*> temp_cover;

	bool mutate;

	for(int i = 0; i<n_iterations; ++i) {
		print_log(path_len(new_path));

		// mutate?
		mutate = FMath::FRandRange(0.0, 1.0) < p_mut;
		
		if(mutate) {
			int mut_idx;
			do {
				mut_idx = FMath::RandRange(0, new_path.size() - 1);
			} while(new_path[mut_idx]->is_start);

			temp_cover = randomCover(pgenerated_guards, new_path[mut_idx]->items);
			new_path.erase(std::next(new_path.begin(),mut_idx));
			new_path.insert(std::next(new_path.begin(), mut_idx),temp_cover.begin(), temp_cover.end());
		}

		// random cover
		new_path = randomCoverSpecial(new_path,lost_items);

		// two opt
		two_opt_mtsp(new_path);

		//print_log(mutate);
		//print_log(new_path.size());

		// compare
		if (path_len(new_path) < path_len(old_path)) {
			old_path = new_path;
			if (mutate) {
				p_mut = std::min(0.95, p_mut + 0.01);
			} else {
				p_mut = std::max(0.05, p_mut - 0.01);
			}
		} else {
			new_path = old_path;
		}
		
	}

	
	print_log(path_len(new_path));
	draw_path(new_path);
}

void AShortSearch::draw_path(const std::vector<Guard*> & path) const {
	for (int i = 0; i < path.size() - 1; ++i) {		
		if (path[i]->is_start) {
			map->drawPoint(path[i]->pos, 30.0, FColor(0, 255, 0));
		}

		if (!path[i + 1]->is_start) {
			map->drawPoint(path[i+1]->pos);
			Path_KP pkp = planner.find_path(path[i]->pos, path[i + 1]->pos);
			for (int j = 0; j < pkp.waypoints.size()-1; ++j) {
				//print_log(pkp.waypoints[j]);
				map->drawLine(pkp.waypoints[j], pkp.waypoints[j + 1]);
			}			
		}
	}
}



float AShortSearch::path_len(const std::vector<Guard*> & path) {
	float max_path_len = 0;
	float current_path_len = 0;
	for (int i = 0; i < path.size()-1; ++i) {
		if (path[i]->is_start) {
			max_path_len = std::max(max_path_len, current_path_len);
			current_path_len = 0;			
		}

		if(!path[i + 1]->is_start) {
			current_path_len += edge_dist(path[i],path[i+1]);
		}
	}
	max_path_len = std::max(max_path_len, current_path_len);
	return max_path_len;
}


float AShortSearch::edge_dist(Guard* g1,Guard* g2) {
	PGuardPair p(g1, g2);
	if (edge_dists.count(p) == 0) {
		edge_dists[p] = planner.find_path(g1->pos,g2->pos).dist;
	}

	return edge_dists[p];
}



void AShortSearch::test_2opt() {
	std::vector<Guard> test_guards;
	Guard g;
	g.pos = FVector(100, 100, 100);
	test_guards.push_back(g);
	g.pos = FVector(1, 1, 1);
	test_guards.push_back(g);
	g.pos = FVector(2, 2, 2);
	test_guards.push_back(g);
	g.pos = FVector(200, 200, 200);
	test_guards.push_back(g);
	g.pos = FVector(3, 3, 3);
	test_guards.push_back(g);
	g.pos = FVector(4, 4, 4);
	test_guards.push_back(g);
	g.pos = FVector(300, 300, 300);
	test_guards.push_back(g);
	g.pos = FVector(5, 5, 5);
	test_guards.push_back(g);
	g.pos = FVector(6, 6, 6);
	test_guards.push_back(g);

	test_guards[0].is_start = true;
	test_guards[3].is_start = true;
	test_guards[6].is_start = true;

	std::vector<Guard*> test_path;
	for (int i = 0; i < test_guards.size(); ++i) {
		test_path.push_back(&test_guards[i]);
	}

	print_log("ORIGINAL");
	for (int i = 0; i < test_path.size(); ++i) {
		print_log(test_path[i]->pos.ToString());		
	}
	print_log(path_len(test_path));

	//print_log("2OPT1");

	//two_opt_swap1(test_path,2,7);

	//for (int i = 0; i < test_path.size(); ++i) {
	//	print_log(test_path[i]->pos.ToString());
	//}

	//print_log("2OPT2");

	//two_opt_swap2(test_path, 2, 7);

	//for (int i = 0; i < test_path.size(); ++i) {
	//	print_log(test_path[i]->pos.ToString());
	//}

	print_log("2OPT");

	two_opt_mtsp(test_path);

	for (int i = 0; i < test_path.size(); ++i) {
		print_log(test_path[i]->pos.ToString());
	}
	print_log(path_len(test_path));


}

void AShortSearch::two_opt_mtsp(std::vector<Guard*> & path) {
	
	std::vector<Guard*> temp_path1;
	std::vector<Guard*> temp_path2;

	int best_i = 0;
	int best_j = 0;
	int best_swap;

	float old_path_len = path_len(path);
	float new_path_len1;
	float new_path_len2;

	old_path_len = path_len(path);

	do {
		best_swap = 0;
		for (int i = 0; i < path.size() - 2; ++i) {
			for (int j = i+2; j < path.size(); ++j) {
				temp_path1 = path;
				temp_path2 = path;

				two_opt_swap1(temp_path1,i,j);
				two_opt_swap2(temp_path2,i,j);

				new_path_len1 = path_len(temp_path1);
				new_path_len2 = path_len(temp_path2);

				if (new_path_len1 < old_path_len) {
					old_path_len = new_path_len1;
					best_i = i;
					best_j = j;
					best_swap = 1;
				}

				if (new_path_len2 < old_path_len) {
					old_path_len = new_path_len2;
					best_i = i;
					best_j = j;
					best_swap = 2;
				}
			}
		}
		if (best_swap == 1) {
			two_opt_swap1(path,best_i,best_j);
		}

		if (best_swap == 2) {
			two_opt_swap2(path, best_i, best_j);
		}
	} while (best_swap != 0);
}

void AShortSearch::two_opt_swap1(std::vector<Guard*> & path, int idx1, int idx2) {
	auto it1 = std::next(path.begin(), 1 + std::min(idx1, idx2));
	auto it2 = std::next(path.begin(), 1 + std::max(idx1, idx2));

	std::reverse(it1,it2);
}


void AShortSearch::two_opt_swap2(std::vector<Guard*> & path, int idx1, int idx2) {
	auto it1 = std::next(path.begin(), 1 + std::min(idx1, idx2));
	auto it2 = std::next(path.begin(), 1 + std::max(idx1, idx2));


	std::vector<Guard*> second_loop(it1,it2);
	
	auto mid = second_loop.begin();
	for (auto it = second_loop.begin(); it != second_loop.end(); ++it) {
		if ((*it)->is_start) {
			mid = it;
			break;
		}
	}

	std::rotate(second_loop.begin(),mid,second_loop.end());
	path.erase(it1,it2);
	path.insert(path.end(),second_loop.begin(),second_loop.end());
}



void AShortSearch::test_random_path() {
	FVector from;
	FVector to;

	int rand_idx;
	FVector base;

	float random_angle;
	float random_radius;

	FVector random_direction;
	FVector random_point;

	rand_idx = FMath::RandRange(0, map->itemPoints.Num() - 1);
	base = map->itemPoints[rand_idx];

	do {
		random_angle = FMath::RandRange(0.0, twopi);
		random_direction = FVector(cos(random_angle), sin(random_angle), 0.0);
		random_radius = FMath::RandRange(0.0, map->sensor_range);
		random_point = base + random_radius*random_direction;
	} while (isInAnyPolygon(random_point, map->allGroundPoints) || !isInPolygon(random_point, map->wallPoints));

	from = random_point;

	rand_idx = FMath::RandRange(0, map->itemPoints.Num() - 1);
	base = map->itemPoints[rand_idx];

	do {
		random_angle = FMath::RandRange(0.0, twopi);
		random_direction = FVector(cos(random_angle), sin(random_angle), 0.0);
		random_radius = FMath::RandRange(0.0, map->sensor_range);
		random_point = base + random_radius*random_direction;
	} while (isInAnyPolygon(random_point, map->allGroundPoints) || !isInPolygon(random_point, map->wallPoints));

	to = random_point;

	Path_KP path = planner.find_path(from, to);

	map->drawPoint(from, 30.0, FColor(0, 255, 0));
	map->drawPoint(to, 30.0, FColor(0, 255, 0));

	if (path.waypoints.size() == 0) {
		print_log("FAILED TO FIND PATH");
		return;
	}

	FVector draw_from;
	FVector draw_to;

	print_log(from);
	print_log(to);

	draw_from = from;

	for (int i = 0; i < path.waypoints.size(); ++i) {
		draw_to = path.waypoints[i];
		map->drawLine(draw_from, draw_to);
		draw_from = draw_to;
		map->drawPoint(draw_from);
	}
}

















