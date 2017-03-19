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

	//file_log("asdf1");

	for (int i = 0; i < n_agents; ++i) {
		my_path.push_back(rrt.get_full_path(map->startPoints[i], map->startVel, map->goalPoints[start_goal_mapping[i]], map->goalVel));
	}
	
	//file_log("asdf2");

	for (int i = 0; i < n_agents; ++i) {
		if (my_path.size() > 0) {
			for (int j = 0; j < my_path[i].size(); ++j) {
				//print_log(my_path[i]->pos.ToString());
				//print_log(FString::SanitizeFloat(my_path[i]->path->path_time()));
				//print_log(FString::SanitizeFloat(my_path[i]->path->pathExists()));
				DrawDebugPoint(GetWorld(), my_path[i][j]->pos + FVector(0, 0, 30), 15, FColor::Magenta, true);
			}
			DrawDebugPoint(GetWorld(), map->startPoints[i] + FVector(0, 0, 30), 15, FColor::Magenta, true);
			print("TIME TAKEN: " + FString::SanitizeFloat(my_path[i].back()->cost));
			print_log("TIME TAKEN: " + FString::SanitizeFloat(my_path[i].back()->cost));
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
			error += std::pow(FVector::Dist(pi,pj) - FVector::Dist(gi, gj),2);
		}
	}

	return error;
}

void AFormationDynamicCar::map_goals() {
	for (int i = 0; i < n_agents; ++i) {
		start_goal_mapping[i] = i;
	}
}












