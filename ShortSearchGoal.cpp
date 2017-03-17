// Fill out your copyright notice in the Description page of Project Settings.

#include "aimasa2.h"
#include "ShortSearchGoal.h"


// Sets default values
AShortSearchGoal::AShortSearchGoal() {
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void AShortSearchGoal::BeginPlay() {
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

	map->setGoalVisibility(true);

	print("Map initializing...", 50);
	print_log("Map initializing...");
}

// Called every frame
void AShortSearchGoal::Tick(float DeltaTime) {
	Super::Tick(DeltaTime);
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

	t_now += DeltaTime;

	for (int i = 0; i < n_guards; ++i) {
		float my_dist = t_now * map->v_max;

		for (int j = 1; j < play_path[i].size(); ++j) {
			if (my_dist < play_path[i][j].total_dist) {

				for (auto it = final_path[i][j - 1]->items.ints.begin(); it != final_path[i][j - 1]->items.ints.end(); ++it) {
					map->items[*it]->SetActorHiddenInGame(true);
				}

				FVector base = play_path[i][j - 1].waypoint;
				FVector next = play_path[i][j].waypoint;
				FVector direction = next - base;
				direction.Normalize();
				float dist_traveled = my_dist - play_path[i][j - 1].total_dist;
				FVector target_position = base + dist_traveled * direction;
				map->drawPoint(target_position, 5);
				map->cars[i]->SetActorLocation(target_position);
				break;
			} else if (j == play_path[i].size() - 1 && my_dist >= play_path[i][j].total_dist) {
				FVector target_position = play_path[i][j].waypoint;
				map->drawPoint(target_position, 5);
				map->cars[i]->SetActorLocation(target_position);
			} else {

			}
		}
	}

	for (int i = 0; i < map->items.Num(); ++i) {
		FVector item_pos = map->items[i]->GetActorLocation();
		for (int j = 0; j < n_guards; ++j) {
			FVector car_pos = map->cars[j]->GetActorLocation();

			if (FVector::DistSquared(item_pos, car_pos) < map->sensor_range2) {
				if (map->Trace(item_pos, car_pos)) {
					map->items[i]->SetActorHiddenInGame(true);
				}
			}
		}
	}



}

void AShortSearchGoal::init() {
	planner = PathPlanner_KP(map);
	//planner.draw_vgraph();
	//test_random_path();
	//test_2opt();

	IntSet start_items;
	IntSet goal_items;

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

	for (int i = 0; i < n_guards; ++i) {
		Guard g;
		g.pos = map->goalPoints[i];
		g.is_goal = true;

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
		goal_items += items_in_sight;
		goal_guards.push_back(g);
	}

	// create random guards


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

	IntSet lost_items = all_items;
	lost_items -= start_items;
	lost_items -= goal_items; 

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

	std::vector<Guard*> pgoal_guards;
	pgoal_guards.reserve(goal_guards.size());
	for (int i = 0; i < goal_guards.size(); ++i) {
		pgoal_guards.push_back(&goal_guards[i]);
	}

	std::vector<Guard*> new_path = randomCover(pgenerated_guards, lost_items);
	//std::vector<Guard*> nonstart_guards = rcover;


	new_path.insert(new_path.begin(), pstart_guards.begin(), pstart_guards.end());
	if (new_path.size() > 1) {
		std::random_shuffle(new_path.begin() + 1, new_path.end());
	}



	for (int i = 0; i < pstart_guards.size(); ++i) {
		start_goal_mapping[pstart_guards[i]] = pgoal_guards[i];
	}




	//for (int i = 0; i < new_path.size(); ++i) {
	//	if (new_path[i]->is_start) {
	//		print_log("START");
	//	} else if (new_path[i]->is_goal) {
	//		print_log("GOAL");
	//	} else {
	//		print_log("X");
	//	}		
	//}


	//make_play_path(new_path);
	//return;

	std::vector<Guard*> old_path = new_path;
	std::vector<Guard*> temp_cover;

	bool mutate;

	for (int i = 0; i < n_iterations; ++i) {
		//print_log(path_len(new_path));

		// mutate?
		mutate = FMath::FRandRange(0.0, 1.0) < p_mut;

		if (mutate) {
			int mut_idx;
			do {
				mut_idx = FMath::RandRange(0, new_path.size() - 1);
			} while (new_path[mut_idx]->is_start || new_path[mut_idx]->is_goal);

			temp_cover = randomCover(pgenerated_guards, new_path[mut_idx]->items);
			new_path.erase(std::next(new_path.begin(), mut_idx));
			new_path.insert(std::next(new_path.begin(), mut_idx), temp_cover.begin(), temp_cover.end());
		}

		// random cover
		new_path = randomCoverSpecial(new_path, lost_items);

		// two opt
		two_opt_mtsp(new_path);

		//print_log(mutate);
		//print_log(new_path.size());

		// compare
		if (path_len(new_path,start_goal_mapping) < path_len(old_path,start_goal_mapping)) {
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


	print_log(path_len(new_path,start_goal_mapping));
	make_play_path(new_path);


	print_log("TIME TAKEN: " + FString::SanitizeFloat(path_len(new_path,start_goal_mapping) / map->v_max));

	draw_path(new_path);
}

void AShortSearchGoal::make_play_path(std::vector<Guard*> & path) {

	//Guard * start;
	//for (int i = 0; i < final_path.size(); ++i) {
	//	start = final_path[i][0];
	//	final_path[i].push_back(start_goal_mapping[start]);
	//	g.waypoint = start_goal_mapping[start]->pos;
	//	g.total_dist = 0;
	//	play_path.back().push_back(g);
	//}

	Guard * last_goal = start_goal_mapping[path[0]];
	Guard * temp;
	int goals_placed = 0;
	while (goals_placed < n_guards-1) {
		for (int i = 1; i < path.size(); ++i) {
			if (path[i]->is_start && !path[i-1]->is_goal) {
				//print_log(start_goal_mapping.count(path[i]));
				temp = path[i];
				
				auto it = path.begin();
				std::advance(it, i);
				path.insert(it, last_goal);		
				last_goal = start_goal_mapping[temp];
				++goals_placed;
				break;
			}
		}
	}
	path.insert(path.end(),last_goal);



	play_path.clear();
	final_path.clear();
	PathSegment g;
	for (int i = 0; i < path.size() - 1; ++i) {
		if (path[i]->is_start) {
			play_path.push_back(std::vector<PathSegment>());
			final_path.push_back(std::vector<Guard*>());
		}

		g.waypoint = path[i]->pos;
		g.total_dist = 0;
		play_path.back().push_back(g);
		final_path.back().push_back(path[i]);

		if (!path[i + 1]->is_start) {
			Path_KP pkp = planner.find_path(path[i]->pos, path[i + 1]->pos);
			for (int j = 1; j < pkp.waypoints.size() - 1; ++j) {
				g.waypoint = pkp.waypoints[j];
				g.total_dist = 0;
				play_path.back().push_back(g);
				final_path.back().push_back(path[i]);
			}
		}
	}

	g.waypoint = path.back()->pos;
	g.total_dist = 0;
	play_path.back().push_back(g);
	final_path.back().push_back(path.back());


	float d;
	for (int i = 0; i < play_path.size(); ++i) {
		d = 0;
		for (int j = 1; j < play_path[i].size(); ++j) {
			d += FVector::Dist(play_path[i][j - 1].waypoint, play_path[i][j].waypoint);
			play_path[i][j].total_dist = d;
		}
	}
}

void AShortSearchGoal::draw_path(const std::vector<Guard*> & path) const {
	reset_log_file();

	for (int i = 0; i < path.size() - 1; ++i) {
		if (path[i]->is_start) {
			map->drawPoint(path[i]->pos, 30.0, FColor(0, 255, 0));
			//print_log(path[i]->pos);
			//file_log("NEW PATH");
			//file_log(FString::SanitizeFloat(path[i]->pos.X) + "," + FString::SanitizeFloat(path[i]->pos.Y));
		}
		if (!path[i + 1]->is_start) {
			map->drawPoint(path[i + 1]->pos);
			//print_log(path[i+1]->pos);
			//file_log(FString::SanitizeFloat(path[i + 1]->pos.X) + "," + FString::SanitizeFloat(path[i + 1]->pos.Y));
			Path_KP pkp = planner.find_path(path[i]->pos, path[i + 1]->pos);
			for (int j = 0; j < pkp.waypoints.size() - 1; ++j) {
				//print_log(pkp.waypoints[j]);
				map->drawLine(pkp.waypoints[j], pkp.waypoints[j + 1]);
				//print_log(path[j + 1]->pos);
				//file_log(FString::SanitizeFloat(pkp.waypoints[j + 1].X) + "," + FString::SanitizeFloat(pkp.waypoints[j + 1].Y));
			}
		}
	}
}



float AShortSearchGoal::path_len(const std::vector<Guard*> & path, std::unordered_map<Guard*, Guard*> sg_map) {
	float max_path_len = 0;
	float current_path_len = 0;
	Guard * last_start = path[0];
	for (int i = 0; i < path.size() - 1; ++i) {
		if (path[i]->is_start) {			
			last_start = path[i];
			max_path_len = std::max(max_path_len, current_path_len);
			current_path_len = 0;
		}

		if (!path[i + 1]->is_start) {
			current_path_len += edge_dist(path[i], path[i + 1]);
		} else {
			current_path_len += edge_dist(path[i], sg_map[last_start]);
		}
	}
	current_path_len += edge_dist(path.back(), sg_map[last_start]);
	max_path_len = std::max(max_path_len, current_path_len);
	return max_path_len;
}


float AShortSearchGoal::edge_dist(Guard* g1, Guard* g2) {
	PGuardPair p(g1, g2);
	if (edge_dists.count(p) == 0) {
		edge_dists[p] = planner.find_path(g1->pos, g2->pos).dist;
	}

	return edge_dists[p];
}



void AShortSearchGoal::test_2opt() {
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
	print_log(path_len(test_path, start_goal_mapping));

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
	print_log(path_len(test_path, start_goal_mapping));


}

void AShortSearchGoal::two_opt_mtsp(std::vector<Guard*> & path) {

	std::vector<Guard*> temp_path1;
	std::vector<Guard*> temp_path2;

	std::unordered_map<Guard*, Guard*> best_sg_map = start_goal_mapping;
	std::unordered_map<Guard*, Guard*> sg_map = start_goal_mapping;

	int best_i = 0;
	int best_j = 0;
	int best_swap;

	float old_path_len = path_len(path,best_sg_map);
	float new_path_len1;
	float new_path_len2;

	//old_path_len = path_len(path);

	std::vector<Guard *> pstart_guards;
	//std::vector<Guard *> pgoal_guards;

	for (auto kv : start_goal_mapping) {
		pstart_guards.push_back(kv.first);
		//pgoal_guards.push_back(kv.second);
	}



	do {
		
		best_swap = 0;

		for (auto it1 : pstart_guards) {
			for (auto it2 : pstart_guards) {
				sg_map = start_goal_mapping;

				Guard * temp;
				temp = sg_map[it1];
				sg_map[it1] = sg_map[it2];
				sg_map[it2] = temp;

				new_path_len1 = path_len(path,sg_map);

				if (new_path_len1 < old_path_len) {
					old_path_len = new_path_len1;
					best_swap = 3;
					best_sg_map = sg_map;
				}
			}
		}


		for (int i = 0; i < path.size() - 2; ++i) {
			for (int j = i + 2; j < path.size(); ++j) {
				temp_path1 = path;
				temp_path2 = path;

				two_opt_swap1(temp_path1, i, j);
				two_opt_swap2(temp_path2, i, j);

				new_path_len1 = path_len(temp_path1, start_goal_mapping);
				new_path_len2 = path_len(temp_path2, start_goal_mapping);

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
			two_opt_swap1(path, best_i, best_j);
		}

		if (best_swap == 2) {
			two_opt_swap2(path, best_i, best_j);
		}

		if (best_swap == 3) {
			start_goal_mapping = best_sg_map;
		}


	} while (best_swap != 0);
}

void AShortSearchGoal::two_opt_swap1(std::vector<Guard*> & path, int idx1, int idx2) {
	auto it1 = std::next(path.begin(), 1 + std::min(idx1, idx2));
	auto it2 = std::next(path.begin(), 1 + std::max(idx1, idx2));

	std::reverse(it1, it2);
}


void AShortSearchGoal::two_opt_swap2(std::vector<Guard*> & path, int idx1, int idx2) {
	auto it1 = std::next(path.begin(), 1 + std::min(idx1, idx2));
	auto it2 = std::next(path.begin(), 1 + std::max(idx1, idx2));


	std::vector<Guard*> second_loop(it1, it2);

	auto mid = second_loop.begin();
	for (auto it = second_loop.begin(); it != second_loop.end(); ++it) {
		if ((*it)->is_start) {
			mid = it;
			break;
		}
	}

	std::rotate(second_loop.begin(), mid, second_loop.end());
	path.erase(it1, it2);
	path.insert(path.end(), second_loop.begin(), second_loop.end());
}



void AShortSearchGoal::test_random_path() {
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

















