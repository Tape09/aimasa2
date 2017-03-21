// Fill out your copyright notice in the Description page of Project Settings.

#include "aimasa2.h"
#include "SimpleFormationDP.h"


// Sets default values
ASimpleFormationDP::ASimpleFormationDP() {
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void ASimpleFormationDP::BeginPlay() {
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

	pidx = std::vector<float>(n_agents, 0);
	t_now = std::vector<float>(n_agents, 0);
	is_driving1 = std::vector<bool>(n_agents, false);
	is_driving2 = false;
	abs_time = 0;
	cost = 0;

	start_goal_mapping = std::vector<int>(n_agents);
	formation_offset = std::vector<FVector>(n_agents);

	print("Map initializing...", 50);
	print_log("Map initializing...");
}

// Called every frame
void ASimpleFormationDP::Tick(float DeltaTime) {
	Super::Tick(DeltaTime);
	if (stop) return;

	if (!has_initialized) {
		init();
		has_initialized = true;
		print_log("Map initialized!");

		for (int i = 0; i < n_agents; ++i) {
			pidx[i] = 0;
			t_now[i] = 0;
			is_driving1[i] = true;
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

	//stage1
	for (int i = 0; i < n_agents; ++i) {
		if (!is_driving1[i]) continue;
		float dt;

		t_now[i] += DeltaTime;
		if (t_now[i] > my_path1[i][pidx[i]]->path->path_time()) {
			dt = t_now[i] - my_path1[i][pidx[i]]->path->path_time();
			if (pidx[i] + 1 >= my_path1[i].size()) {
				is_driving1[i] = false;
				map->cars[i]->SetActorLocationAndRotation(my_path1[i][pidx[i]]->path->pos_1(), my_path1[i][pidx[i]]->path->vel_1().Rotation());
				if (std::find(is_driving1.begin(), is_driving1.end(), true) == is_driving1.end()) {
					for (int j = 0; j < n_agents; ++j) {
						pidx[j] = 0;
						t_now[j] = 0;
						is_driving2 = true;
					}
					print_log("stage 1 cost: " + FString::SanitizeFloat(cost));
				}
				continue;
			} else {
				pidx[i] += 1;
				t_now[i] = 0;
			}
		} else {
			dt = DeltaTime;
		}
		//file_log(FString::FromInt(i) + ": " + FString::FromInt(pidx[i]) + " / " + FString::FromInt(my_path[i].size()));
		State s = my_path1[i][pidx[i]]->path->step(dt);

		map->cars[i]->SetActorLocationAndRotation(s.pos, s.vel.Rotation());
		DrawDebugPoint(GetWorld(), s.pos + FVector(0, 0, 30), 2.5, FColor::Red, true);
	}

	//stage2
	if (is_driving2) {
		float dt;

		t_now[0] += DeltaTime;
		if (t_now[0] > my_path2[pidx[0]]->path->path_time()) {
			dt = t_now[0] - my_path2[pidx[0]]->path->path_time();
			if (pidx[0] + 1 >= my_path2.size()) {
				is_driving2 = false;
				for (int i = 0; i < n_agents; ++i) {
					map->cars[i]->SetActorLocationAndRotation(my_path2[pidx[0]]->path->pos_1() + formation_offset[start_goal_mapping[i]], my_path2[pidx[0]]->path->vel_1().Rotation());
				}


			} else {
				pidx[0] += 1;
				t_now[0] = 0;
			}
		} else {
			dt = DeltaTime;
		}

		if (is_driving2) {
			//file_log(FString::FromInt(i) + ": " + FString::FromInt(pidx[i]) + " / " + FString::FromInt(my_path[i].size()));
			State s = my_path2[pidx[0]]->path->step(dt);
			for (int i = 0; i < n_agents; ++i) {
				map->cars[i]->SetActorLocationAndRotation(s.pos + formation_offset[start_goal_mapping[i]], s.vel.Rotation());
				DrawDebugPoint(GetWorld(), s.pos + formation_offset[start_goal_mapping[i]] + FVector(0, 0, 30), 2.5, FColor::Red, true);
			}
		}
	}




	// if someone is still driving
	if ((std::find(is_driving1.begin(), is_driving1.end(), true) != is_driving1.end()) || is_driving2) {
		abs_time += DeltaTime;
		float ferror = formation_error();
		cost += DeltaTime + ferror*DeltaTime;
	}

	// if no one is driving
	if ((std::find(is_driving1.begin(), is_driving1.end(), true) == is_driving1.end()) && !is_driving2) {
		stop = true;
		print_log("TOTAL COST: " + FString::SanitizeFloat(cost));
		print("TOTAL COST: " + FString::SanitizeFloat(cost));
	}

}

void ASimpleFormationDP::init() {

	// stage 1: get into formation
	FVector center_start(0, 0, 0);
	FVector center_goal(0, 0, 0);
	start_goal_mapping = std::vector<int>(n_agents);
	formation_offset = std::vector<FVector>(n_agents);

	for (int i = 0; i < n_agents; ++i) {
		center_start += map->startPoints[i];
		center_goal += map->goalPoints[i];
		formation_offset[i] = map->goalPoints[i] - map->goalPoints[0];
		//formation_offset[i] = map->goalPoints[i] - map->goalPoints[0];
		start_goal_mapping[i] = i;
	}

	center_start /= n_agents;
	center_goal /= n_agents;

	//for (int i = 0; i < n_agents; ++i) {
	//	formation_offset[i] = map->goalPoints[i] - center_goal;
	//}


	FVector start_goal_vec = center_goal - center_start;
	start_goal_vec.Normalize();

	float formation_diameter = 0;
	for (int i = 0; i < map->goalPoints.Num(); ++i) {
		for (int j = 0; j < map->goalPoints.Num(); ++j) {
			float d = FVector::Dist(map->goalPoints[i], map->goalPoints[j]);
			formation_diameter = std::max(formation_diameter, d);
		}
	}

	FVector offset = center_goal - map->goalPoints[0];
	FVector initial_formation_center = center_start;
	FVector initial_leader_point;
	FVector best_leader_point;
	bool collision_exists;

	std::vector<std::vector<std::shared_ptr<RRTNode>>> temp_path;
	float best_cost = 9999999;
	RRT rrt1(250, map, std::bind(&ASimpleFormationDP::calc_path, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4), v_max, a_max);
	rrt1.sigma2 = formation_diameter / 2;
	for (int k = 0; k < stage1_iterations; ++k) {

		do {
			collision_exists = false;
			initial_leader_point = map->randomPointNoCollision(initial_formation_center, formation_diameter / 2);

			for (int i = 0; i < n_agents; ++i) {
				if (!map->isValidPoint(initial_leader_point + formation_offset[i])) {
					collision_exists = true;
					break;
				}
			}
		} while (collision_exists);

		std::vector<int> temp_sg_map = start_goal_mapping;
		float min_max_dist = 99999999;
		do {
			float max_dist = 0;
			for (int i = 0; i < n_agents; ++i) {
				max_dist = std::max(max_dist, FVector::Dist(map->startPoints[i], initial_leader_point + formation_offset[temp_sg_map[i]]));
			}

			if (max_dist < min_max_dist) {
				start_goal_mapping = temp_sg_map;
				min_max_dist = max_dist;
			}

		} while (std::next_permutation(temp_sg_map.begin(), temp_sg_map.end()));



		for (int i = 0; i < n_agents; ++i) {
			temp_path.push_back(rrt1.get_full_path(map->startPoints[i], map->startVel, initial_leader_point + formation_offset[start_goal_mapping[i]], start_goal_vec * 0.0000001));
		}

		bool good_path = true;
		for (int i = 0; i < n_agents; ++i) {
			if (temp_path[i].size() == 0) {
				good_path = false;
				break;
			}
		}

		if (good_path) {
			float temp_cost = total_cost(temp_path, 10);

			if (temp_cost < best_cost) {
				my_path1 = temp_path;
				best_cost = temp_cost;
				best_leader_point = initial_leader_point;
			}
		}
	}

	if (my_path1.size() == 0) {
		stop = true;
		print_log("FAILED TO FIND PATH FOR STAGE 1");
		return;
	}


	RRT rrt2(1500, map, std::bind(&ASimpleFormationDP::calc_path_formation, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4), v_max, a_max);
	rrt2.sigma2 = formation_diameter / 2;


	FVector goal_orientation = map->goalPoints[0] - initial_leader_point;
	goal_orientation.Normalize();


	my_path2 = rrt2.get_full_path(best_leader_point, start_goal_vec * 0.0000001, map->goalPoints[0], goal_orientation*v_max);

	if (my_path2.size() == 0) {
		stop = true;
		print_log("FAILED TO FIND PATH FOR STAGE 2");
		return;
	}

	//FVector virtual_start_pos = map->startPoints[0];
	//FVector virtual_goal_pos = map->goalPoints[start_goal_mapping[0]];

	//FVector goalVel = (virtual_goal_pos - virtual_start_pos);
	//goalVel.Normalize();
	//goalVel = goalVel * v_max;


	//std::vector<std::shared_ptr<RRTNode>> virtual_path = rrt.get_full_path(virtual_start_pos, map->startVel, virtual_goal_pos, goalVel);

	//float virtual_time_taken = virtual_path.back()->cost;

	////std::shared_ptr<RRTNode> a = virtual_path.back();

	//int resolution = 100;
	//for (int i = 0; i <= resolution; ++i) {
	//	float t = virtual_time_taken * (float(i) / resolution);
	//	State s = state_at(virtual_path, t);
	//	map->drawPoint(s.pos, 5);
	//}

	//for (int i = 0; i < virtual_path.size(); ++i) {
	//	map->drawPoint(virtual_path[i]->pos, 15, FColor(0, 255, 0));
	//}

	//rrt.max_iterations = 500;

	////file_log("asdf1");

	//for (int i = 0; i < n_agents; ++i) {
	//	my_path.push_back(rrt.get_full_path_hint(map->startPoints[i], map->startVel, map->goalPoints[start_goal_mapping[i]], map->goalVel, virtual_path));
	//	//my_path.push_back(rrt.get_full_path(map->startPoints[i], map->startVel, map->goalPoints[start_goal_mapping[i]], FVector(0,0,0)));
	//}

	////file_log("asdf2");

	//for (int i = 0; i < n_agents; ++i) {
	//	if (my_path[i].size() == 0) {
	//		stop = true;
	//	}
	//}

	//if (!stop) random_optimization();

	//for (int i = 0; i < n_agents; ++i) {
	//	if (my_path[i].size() > 0) {
	//		for (int j = 0; j < my_path[i].size(); ++j) {
	//			DrawDebugPoint(GetWorld(), my_path[i][j]->pos + FVector(0, 0, 30), 15, FColor::Magenta, true);
	//		}
	//		DrawDebugPoint(GetWorld(), map->startPoints[i] + FVector(0, 0, 30), 15, FColor::Magenta, true);
	//		print("TIME TAKEN: " + FString::SanitizeFloat(my_path[i].back()->cost));
	//		print_log("TIME TAKEN: " + FString::SanitizeFloat(my_path[i].back()->cost));
	//	} else {
	//		stop = true;
	//		print("PATH NOT FOUND");
	//		print_log("PATH NOT FOUND");
	//	}
	//}



	//my_path.push_back(std::make_shared<RRTNode>(RRTNode(std::make_shared<RRTNode>(RRTNode(pos0,vel0)), calc_path(pos0, vel0, pos1, vel1),pos0)));

	//print_log("INIT DONE");
}


std::shared_ptr<Path> ASimpleFormationDP::calc_path(FVector pos0, FVector vel0, FVector pos1, FVector vel1) {
	DynamicPath* dp = new DynamicPath(pos0, vel0, pos1, vel1, v_max, a_max);

	int resolution = 10 * dp->path_time();
	//int resolution = 50;
	dp->valid = true;
	for (int i = 0; i <= resolution; ++i) {
		float time = i * dp->path_time() / resolution;
		State s = dp->state_at(time);

		if (isInAnyPolygon(s.pos, map->allGroundPoints) || !isInPolygon(s.pos, map->wallPoints)) {
			dp->valid = false;
			break;
		}
	}

	return std::shared_ptr<Path>(dp);
}

std::shared_ptr<Path> ASimpleFormationDP::calc_path_formation(FVector pos0, FVector vel0, FVector pos1, FVector vel1) {
	DynamicPath* dp = new DynamicPath(pos0, vel0, pos1, vel1, v_max, a_max);


	int resolution = 10 * dp->path_time();

	dp->valid = true;
	for (int i = 0; i <= resolution; ++i) {
		float time = i * dp->path_time() / resolution;
		State s = dp->state_at(time);

		for (int k = 0; k < n_agents; ++k) {
			if (!map->isValidPoint(s.pos + formation_offset[start_goal_mapping[k]])) {
				dp->valid = false;
				break;
			}
		}
	}

	return std::shared_ptr<Path>(dp);
}

float ASimpleFormationDP::formation_error() {
	float error = 0;

	FVector pi;
	FVector pj;

	FVector gi;
	FVector gj;

	for (int i = 0; i < n_agents; ++i) {
		pi = map->cars[i]->GetActorLocation() / map->scale;
		gi = map->goalPoints[start_goal_mapping[i]] / map->scale;
		for (int j = 0; j < n_agents; ++j) {
			pj = map->cars[j]->GetActorLocation() / map->scale;
			gj = map->goalPoints[start_goal_mapping[j]] / map->scale;
			error += std::pow(FVector::Dist(pi, pj) - FVector::Dist(gi, gj), 2);
		}
	}


	return error;
}

float ASimpleFormationDP::formation_error(std::vector<std::vector<std::shared_ptr<RRTNode>>> & path, float t) {


	float error = 0;
	FVector pi;
	FVector pj;

	FVector gi;
	FVector gj;

	State s;
	for (int i = 0; i < n_agents; ++i) {
		s = state_at(path[i], t);
		pi = s.pos / map->scale;
		gi = map->goalPoints[start_goal_mapping[i]] / map->scale;
		for (int j = 0; j < n_agents; ++j) {
			s = state_at(path[j], t);
			pj = s.pos / map->scale;
			gj = map->goalPoints[start_goal_mapping[j]] / map->scale;
			error += std::pow(FVector::Dist(pi, pj) - FVector::Dist(gi, gj), 2);
		}
	}


	return error;
}

void ASimpleFormationDP::map_goals() {

	float max_dist = 0;
	float maxi = 0;
	float maxj = 0;
	float d;

	for (int i = 0; i < n_agents; ++i) {
		for (int j = 0; j < n_agents; ++j) {
			d = FVector::Dist(map->startPoints[i], map->goalPoints[j]);
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



float ASimpleFormationDP::total_cost(std::vector<std::vector<std::shared_ptr<RRTNode>>> & path, int resolution) {

	float time_taken = 0;
	for (int i = 0; i < n_agents; ++i) {
		time_taken = std::max(time_taken, path[i].back()->cost);
	}

	float dt = time_taken / resolution;
	float error = 0;
	for (int i = 0; i <= resolution; ++i) {
		float t = time_taken * (float(i) / resolution);

		error += dt + formation_error(path, t) * dt;
	}
	return error;
}







