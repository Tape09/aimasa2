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

	print("Map initializing...", 50);
	print_log("Map initializing...");
}

// Called every frame
void AFormationDynamicCar::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );
	if (!has_initialized) {
		init();
		has_initialized = true;
		print_log("Map initialized!");
		t_now = 0;
		pidx = 0;

		if (my_path.size() > 0) {
			my_path[pidx]->path->reset();
			is_driving = true;
		}
	}

	if (buffer_ticks > 0) {
		--buffer_ticks;
		return;
	}

	if (is_driving) {
		float dt;
		t_now += DeltaTime;
		if (t_now > my_path[pidx]->path->path_time()) {
			dt = t_now - my_path[pidx]->path->path_time();
			if (pidx + 1 >= my_path.size()) {
				is_driving = false;
				map->cars[0]->SetActorLocationAndRotation(my_path[pidx]->path->pos_1(), my_path[pidx]->path->vel_1().Rotation());
				return;
			} else {
				++pidx;
				t_now = 0;
			}
		} else {
			dt = DeltaTime;
		}

		State s = my_path[pidx]->path->step(dt);

		map->cars[0]->SetActorLocationAndRotation(s.pos, s.vel.Rotation());
		DrawDebugPoint(GetWorld(), s.pos + FVector(0, 0, 30), 2.5, FColor::Red, true);

	}
}

void AFormationDynamicCar::init() {
	RRT rrt(100, map, std::bind(&AFormationDynamicCar::calc_path, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4), v_max, a_max);

	//file_log("asdf1");
	my_path = rrt.get_full_path(map->startPoints[0], map->startVel, map->goalPoints[0],map->goalVel);
	//file_log("asdf2");

	if (my_path.size() > 0) {
		for (int i = 0; i < my_path.size(); ++i) {

			print_log(my_path[i]->pos.ToString());
			print_log(FString::SanitizeFloat(my_path[i]->path->path_time()));
			print_log(FString::SanitizeFloat(my_path[i]->path->pathExists()));
			DrawDebugPoint(GetWorld(), my_path[i]->pos + FVector(0, 0, 30), 15, FColor::Magenta, true);
		}
		DrawDebugPoint(GetWorld(), map->startPoints[0] + FVector(0, 0, 30), 15, FColor::Magenta, true);
		print("TIME TAKEN: " + FString::SanitizeFloat(my_path.back()->cost));
		print_log("TIME TAKEN: " + FString::SanitizeFloat(my_path.back()->cost));
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


















