// Fill out your copyright notice in the Description page of Project Settings.

#include "aimasa2.h"
#include "FormationDynamicCar.h"


// Sets default values
AFormationDynamicCar::AFormationDynamicCar()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void AFormationDynamicCar::BeginPlay()
{
	Super::BeginPlay();
	const UWorld * world = GetWorld();

	if (world) {
		FActorSpawnParameters spawnParams;
		spawnParams.Owner = this;
		spawnParams.Instigator = Instigator;

		map = GetWorld()->SpawnActor<AMapGen>(FVector(0, 0, 0), FRotator::ZeroRotator, spawnParams);
	}

	n_agents = map->startPoints.Num();
	v_max = map->v_max;
	a_max = map->a_max;

	pidx = std::vector<float>(n_agents,0);
	t_now = std::vector<float>(n_agents,0);
	is_driving = std::vector<bool>(n_agents,0);
	abs_time = 0;
	cost = 0;

	print("Map initializing...", 50);
	print_log("Map initializing...");
}

// Called every frame
void AFormationDynamicCar::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );
	if(stop) return;

	if (!has_initialized) {
		init();
		has_initialized = true;
		print_log("Map initialized!");		

		for (int i = 0; i < n_agents; ++i) {
			pidx[i] = 0;
			t_now[i] = 0;
			is_driving[i] = true;
		}

		//if (my_path.size() > 0) {
		//	my_path[pidx]->path->reset();
		//	is_driving = true;
		//}
	}

	if (buffer_ticks > 0) {
		--buffer_ticks;
		return;
	}		
		
	for (int i = 0; i < n_agents; ++i) { 
		if(!is_driving[i]) continue;
		float dt;

		t_now[i] += DeltaTime;
		if (t_now[i] > my_path[i][pidx[i]]->path->path_time()) {
			dt = t_now[i] - my_path[i][pidx[i]]->path->path_time();
			if (pidx[i] + 1 >= my_path[i].size()) {
				is_driving[i] = false;
				map->cars[i]->SetActorLocationAndRotation(my_path[i][pidx[i]]->path->pos_1(), my_path[i][pidx[i]]->path->vel_1().Rotation());
				//file_log("FINISHED " + FString::FromInt(i));
				continue;
			} else {
				pidx[i] += 1;
				t_now[i] = 0;
			}
		} else {
			dt = DeltaTime;
		}
		//file_log(FString::FromInt(i) + ": " + FString::FromInt(pidx[i]) + " / " + FString::FromInt(my_path[i].size()));
		State s = my_path[i][pidx[i]]->path->step(dt);

		map->cars[i]->SetActorLocationAndRotation(s.pos, s.vel.Rotation());
		DrawDebugPoint(GetWorld(), s.pos + FVector(0, 0, 30), 2.5, FColor::Red, true);
	}
		
	// if someone is still driving
	if (std::find(is_driving.begin(), is_driving.end(), true) != is_driving.end()) {
		abs_time += DeltaTime;
		float ferror = formation_error();
		cost += DeltaTime + ferror*DeltaTime;
	}

	// if no one is driving
	if (std::find(is_driving.begin(), is_driving.end(), true) == is_driving.end()) {
		stop = true;
		print_log("TOTAL COST: " + FString::SanitizeFloat(cost));
		print("TOTAL COST: " + FString::SanitizeFloat(cost));
	}

}

void AFormationDynamicCar::init() {
	//reset_log_file();
	map_goals();

	RRT rrt(500, map, std::bind(&AFormationDynamicCar::calc_path, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4), v_max, a_max);

	FVector virtual_start_pos = map->startPoints[0];
	FVector virtual_goal_pos = map->goalPoints[start_goal_mapping[0]];

	FVector goalVel = (virtual_goal_pos - virtual_start_pos);
	goalVel.Normalize();
	goalVel = goalVel * v_max;


	std::vector<std::shared_ptr<RRTNode>> virtual_path = rrt.get_full_path(virtual_start_pos, map->startVel, virtual_goal_pos, goalVel);

	float virtual_time_taken = virtual_path.back()->cost;

	//std::shared_ptr<RRTNode> a = virtual_path.back();

	int resolution = 100;
	for (int i = 0; i <= resolution; ++i) {
		float t = virtual_time_taken * (float(i) / resolution);
		State s = state_at(virtual_path,t);
		map->drawPoint(s.pos,5);
	}
	
	for (int i = 0; i < virtual_path.size(); ++i) {
		map->drawPoint(virtual_path[i]->pos,15,FColor(0,255,0));
	}
	
	rrt.max_iterations = 500;

	//file_log("asdf1");

	for (int i = 0; i < n_agents; ++i) {
		my_path.push_back(rrt.get_full_path_hint(map->startPoints[i], map->startVel, map->goalPoints[start_goal_mapping[i]], map->goalVel,virtual_path));
		//my_path.push_back(rrt.get_full_path(map->startPoints[i], map->startVel, map->goalPoints[start_goal_mapping[i]], FVector(0,0,0)));
	}
	
	//file_log("asdf2");

	for (int i = 0; i < n_agents; ++i) {
		if (my_path[i].size() == 0) {
			stop = true;
		}
	}

	if(!stop) random_optimization();

	for (int i = 0; i < n_agents; ++i) {
		if (my_path[i].size() > 0) {
			for (int j = 0; j < my_path[i].size(); ++j) {
				DrawDebugPoint(GetWorld(), my_path[i][j]->pos + FVector(0, 0, 30), 15, FColor::Magenta, true);
			}
			DrawDebugPoint(GetWorld(), map->startPoints[i] + FVector(0, 0, 30), 15, FColor::Magenta, true);
			print("TIME TAKEN: " + FString::SanitizeFloat(my_path[i].back()->cost));
			print_log("TIME TAKEN: " + FString::SanitizeFloat(my_path[i].back()->cost));
		} else {
			stop = true;
			print("PATH NOT FOUND");
			print_log("PATH NOT FOUND");
		}
	}

	

	//my_path.push_back(std::make_shared<RRTNode>(RRTNode(std::make_shared<RRTNode>(RRTNode(pos0,vel0)), calc_path(pos0, vel0, pos1, vel1),pos0)));

	//print_log("INIT DONE");
}


std::shared_ptr<Path> AFormationDynamicCar::calc_path(FVector pos0, FVector vel0, FVector pos1, FVector vel1) {
	DynamicCarPaths* dp = new DynamicCarPaths(pos0, vel0, pos1, vel1, v_max, map->phi_max, map->L_car, a_max);

	for (int j = 0; j < dp->n_paths(); ++j) {
		int resolution = 10 * dp->path_time(j);

		dp->valid = true;
		for (int i = 0; i <= resolution; ++i) {
			float time = i * dp->path_time(j) / resolution;
			State s = dp->state_at(j, time);
			if (isInAnyPolygon(s.pos, map->allGroundPoints) || !isInPolygon(s.pos, map->wallPoints)) {
				dp->valid = false;
				break;
			}
		}
		if (dp->isValid()) {
			dp->path_index = j;
			break;
		}
	}

	return std::shared_ptr<Path>(dp);
}



float AFormationDynamicCar::formation_error() {

	std::vector<int> sg_map(n_agents);
	for (int i = 0; i < sg_map.size(); ++i) {
		sg_map[i] = i;
	}

	float lowest_error = 999999999;

	do {

		float error = 0;

		FVector pi;
		FVector pj;

		FVector gi;
		FVector gj;

		for (int i = 0; i < n_agents; ++i) {
			pi = map->cars[i]->GetActorLocation() / map->scale;
			gi = map->goalPoints[sg_map[i]] / map->scale;
			for (int j = 0; j < n_agents; ++j) {
				pj = map->cars[j]->GetActorLocation() / map->scale;
				gj = map->goalPoints[sg_map[j]] / map->scale;
				error += std::pow(FVector::Dist(pi,pj) - FVector::Dist(gi, gj),2);
			}
		}

		lowest_error = std::min(lowest_error,error);
	} while(std::next_permutation(sg_map.begin(), sg_map.end()));

	return lowest_error;
}

float AFormationDynamicCar::formation_error(std::vector<std::vector<std::shared_ptr<RRTNode>>> & path, float t) {	
	std::vector<int> sg_map(n_agents);
	for (int i = 0; i < sg_map.size(); ++i) {
		sg_map[i] = i;
	}

	float lowest_error = 999999999;

	do {
	
		float error = 0;
		FVector pi;
		FVector pj;

		FVector gi;
		FVector gj;

		State s;
		for (int i = 0; i < n_agents; ++i) {
			s = state_at(path[i],t);
			pi = s.pos / map->scale;
			gi = map->goalPoints[sg_map[i]] / map->scale;
			for (int j = 0; j < n_agents; ++j) {
				s = state_at(path[j], t);
				pj = s.pos / map->scale;
				gj = map->goalPoints[sg_map[j]] / map->scale;
				error += std::pow(FVector::Dist(pi, pj) - FVector::Dist(gi, gj), 2);
			}
		}

		lowest_error = std::min(lowest_error, error);
	} while(std::next_permutation(sg_map.begin(), sg_map.end()));

	return lowest_error;
}

void AFormationDynamicCar::map_goals() {

	float max_dist = 0;
	float maxi = 0;
	float maxj = 0;
	float d;

	for (int i = 0; i < n_agents; ++i) {
		for (int j = 0; j < n_agents; ++j) {
			d = FVector::Dist(map->startPoints[i],map->goalPoints[j]);
			if (d > max_dist) {
				maxi = i;
				maxj = j;
				max_dist = d;
			}
		}		
	}

	for (int i = 0; i < n_agents; ++i) {
		if (i == maxi) {
			start_goal_mapping[i] = maxj;
		} else if (i == maxj) {
			start_goal_mapping[i] = maxi;
		} else {
			start_goal_mapping[i] = i;
		}		
	}
	
}

void AFormationDynamicCar::random_optimization() {
	float connect_radius = 0;

	for (int i = 0; i < map->goalPoints.Num(); ++i) {
		for (int j = 0; j < map->goalPoints.Num(); ++j) {
			float d = FVector::Dist(map->goalPoints[i], map->goalPoints[j]);
			connect_radius = std::max(connect_radius, d);
		}
	}
	
	connect_radius *= 0.25;

	std::vector<std::vector<std::shared_ptr<RRTNode>>> temp_path;
	float best_cost = total_cost(my_path);

	for (int c = 0; c < n_iterations; ++c) {
		temp_path = my_path;	
		// pick random waypoint
		int path_idx = FMath::RandRange(0,n_agents-1);
		int wp_idx = FMath::RandRange(1,temp_path[path_idx].size() - 2);
		FVector random_wp = temp_path[path_idx][wp_idx]->pos;

		// create random node around waypoint
		FVector random_point = map->randomPoint(random_wp,connect_radius);
		FVector random_vel = randVel(v_max);

		// swap random node with waypoint
		// recalculate path to new waypoint
		std::shared_ptr<Path> before_segment = calc_path(temp_path[path_idx][wp_idx-1]->pos, temp_path[path_idx][wp_idx - 1]->vel, random_point, random_vel);
		if (!before_segment->isValid() || !before_segment->pathExists()) continue;
		std::shared_ptr<RRTNode> before_node = std::make_shared<RRTNode>(temp_path[path_idx][wp_idx - 1], before_segment, random_point);

		// recalculate path from new waypoint
		std::shared_ptr<Path> after_segment = calc_path(random_point, random_vel, temp_path[path_idx][wp_idx + 1]->pos, temp_path[path_idx][wp_idx + 1]->vel);
		if (!after_segment->isValid() || !after_segment->pathExists()) continue;
		std::shared_ptr<RRTNode> after_node = std::make_shared<RRTNode>(before_node, after_segment, temp_path[path_idx][wp_idx + 1]->pos);

		temp_path[path_idx][wp_idx] = before_node;
		temp_path[path_idx][wp_idx+1] = after_node;

		// adjust cost/time for rest of the path
		for (int i = wp_idx+2; i < temp_path[path_idx].size(); ++i) {
			temp_path[path_idx][i]->cost = temp_path[path_idx][i-1]->cost + temp_path[path_idx][i]->path->path_time();
		}

		float new_cost = total_cost(temp_path);

		if (new_cost < best_cost) {
			my_path = temp_path;
			best_cost = new_cost;
		} else {
			// revert
			for (int i = wp_idx + 2; i < temp_path[path_idx].size(); ++i) {
				my_path[path_idx][i]->cost = my_path[path_idx][i - 1]->cost + my_path[path_idx][i]->path->path_time();
			}
		}
	}
	print_log(best_cost);
}


float AFormationDynamicCar::total_cost(std::vector<std::vector<std::shared_ptr<RRTNode>>> & path) {
	
	float time_taken = 0;
	for (int i = 0; i < n_agents; ++i) {
		time_taken = std::max(time_taken, path[i].back()->cost);
	}

	int resolution = 100;
	float dt = time_taken / resolution;
	float error = 0;
	for (int i = 0; i <= resolution; ++i) {
		float t = time_taken * (float(i) / resolution);

		error += dt + formation_error(path,t) * dt;
	}
	return error;
}







