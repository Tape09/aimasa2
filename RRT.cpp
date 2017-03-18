// Fill out your copyright notice in the Description page of Project Settings.

#include "aimasa2.h"
#include "RRT.h"

RRT::RRT(int max_iterations_, AMapGen * map_, PathFcnType pathFcn_, float v_max_, float a_max_) {
	map = map_;
	pathFcn = pathFcn_;
	v_max = v_max_;
	a_max = a_max_;
	max_iterations = max_iterations_;
	generator = std::default_random_engine(std::chrono::system_clock::now().time_since_epoch().count());
	reset_log_file();
}

RRT::~RRT()
{
}


std::vector<std::shared_ptr<RRTNode>> RRT::get_full_path() {
	std::vector<std::shared_ptr<RRTNode>> out_path;

	std::shared_ptr<RRTNode> best_path = NULL;
	float best_total_cost = 99999999;
	bool found_path = false;

	//int node_idx;
	//int corner_idx;
	bool visible;
	
	FVector start_pos = map->startPoints[0];
	FVector start_vel = map->startVel;
	FVector goal_pos = map->goalPoints[0];
	FVector goal_vel = map->goalVel;

	if (start_vel.Size() > v_max) {
		start_vel = (start_vel / start_vel.Size()) * v_max;
		start_vel = start_vel * 0.999;
	}

	if (goal_vel.Size() > v_max) {
		goal_vel = (goal_vel / goal_vel.Size()) * v_max;
		goal_vel = goal_vel * 0.999;
	}

	TArray<FVector> cornerPoints = map->cornerPoints;
	cornerPoints.Add(goal_pos);

	nodes.clear();
	nodes.push_back(std::make_shared<RRTNode>(start_pos, start_vel));

	for(int j = 0; j<max_iterations; ++j) {

		// pick random visible corner
		FVector random_corner;
		std::vector<std::shared_ptr<RRTNode>> visible_nodes;

		while (visible_nodes.empty()) {
			random_corner = map->cornerPoints[FMath::RandRange(0, map->cornerPoints.Num() - 1)];
			for (int i = 0; i < nodes.size(); ++i) {
				visible = map->Trace(nodes[i]->pos, random_corner, -1);
				if (visible) visible_nodes.push_back(nodes[i]);
			}
		}		
		
		// random point around corner
		float meanx = random_corner.X;
		float meany = random_corner.Y;

		std::normal_distribution<double> distributionx(meanx, sigma2);
		std::normal_distribution<double> distributiony(meany, sigma2);

		FVector random_point = FVector(distributionx(generator), distributiony(generator), 0);
		while (isInAnyPolygon(random_point, map->allGroundPoints) || !isInPolygon(random_point, map->wallPoints)) {
			random_point = FVector(distributionx(generator), distributiony(generator), 0);
		}

		FVector random_vel = randVel(v_max);

		// find paths from points to random point
		float best_cost = 999999;
		std::shared_ptr<RRTNode> best_segment;
		bool found_segment = false;
		for (int i = 0; i < visible_nodes.size(); ++i) {
			std::shared_ptr<Path> temp_path = pathFcn(visible_nodes[i]->pos, visible_nodes[i]->vel, random_point, random_vel);
			if (temp_path->isValid() && temp_path->pathExists()) {
				std::shared_ptr<RRTNode> temp_node = std::make_shared<RRTNode>(visible_nodes[i], temp_path, random_corner);
				if (temp_node->cost < best_cost) {
					best_cost = temp_node->cost;
					found_segment = true;
					best_segment = temp_node;
				}
			}
		}

		if (found_segment) {
			nodes.push_back(best_segment);

			// linetrace random_point to goal => check path to goal
			visible = map->Trace(nodes.back()->pos, goal_pos, -1);
			if (visible) {
				std::shared_ptr<Path> temp_path = pathFcn(nodes.back()->pos, nodes.back()->vel, goal_pos, goal_vel);
				if (temp_path->isValid() && temp_path->pathExists()) {
					std::shared_ptr<RRTNode> temp_node = std::make_shared<RRTNode>(nodes.back(), temp_path, goal_pos);
					if (temp_node->cost < best_total_cost) {
						best_path = temp_node;
						best_total_cost = temp_node->cost;
						found_path = true;
					}
				}
			}
		}
		
	}

	if(found_path) {
		std::shared_ptr<RRTNode> asdf = best_path;
		while (asdf->cost != 0) {
			out_path.push_back(asdf);
			asdf = asdf->child;
		}

		std::reverse(out_path.begin(), out_path.end());
	}	

	return out_path;
}


std::vector<std::shared_ptr<RRTNode>> RRT::get_full_path2() {
	std::vector<std::shared_ptr<RRTNode>> out_path;

	std::shared_ptr<RRTNode> best_path = NULL;
	float best_total_cost = 99999999;
	bool found_path = false;

	int node_idx;
	int corner_idx;
	bool visible;

	FVector start_pos = map->startPoints[0];
	FVector start_vel = map->startVel;
	FVector goal_pos = map->goalPoints[0];
	FVector goal_vel = map->goalVel;

	if (start_vel.Size() > v_max) {
		start_vel = (start_vel / start_vel.Size()) * v_max;
		start_vel = start_vel * 0.999;
	}

	if (goal_vel.Size() > v_max) {
		goal_vel = (goal_vel / goal_vel.Size()) * v_max;
		goal_vel = goal_vel * 0.999;
	}

	nodes.clear();
	nodes.push_back(std::make_shared<RRTNode>(start_pos, start_vel));

	for (int j = 0; j<max_iterations; ++j) {
		// pick random node in nodes
		node_idx = FMath::RandRange(0, nodes.size() - 1);

		// linetrace corner to goal => check path to goal
		visible = map->Trace(nodes[node_idx]->corner, goal_pos, -1);
		if (visible) {
			std::shared_ptr<Path> temp_path = pathFcn(nodes[node_idx]->pos, nodes[node_idx]->vel, goal_pos, goal_vel);
			if (temp_path->isValid() && temp_path->pathExists()) {
				std::shared_ptr<RRTNode> temp_node = std::make_shared<RRTNode>(nodes[node_idx], temp_path, goal_pos);
				if (temp_node->cost < best_total_cost) {
					best_path = temp_node;
					best_total_cost = temp_node->cost;
					found_path = true;
				}
			}
		}
		// line trace corner to all corners	
		std::vector<FVector> visible_corners;
		for (int i = 0; i < map->cornerPoints.Num(); ++i) {
			visible = map->Trace(nodes[node_idx]->pos, map->cornerPoints[i], -1);
			if (visible) visible_corners.push_back(map->cornerPoints[i]);
		}

		// pick corner - can pick own corner
		corner_idx = FMath::RandRange(0, visible_corners.size() - 1);

		// random point around corner
		float meanx = visible_corners[corner_idx].X;
		float meany = visible_corners[corner_idx].Y;

		std::normal_distribution<double> distributionx(meanx, sigma2);
		std::normal_distribution<double> distributiony(meany, sigma2);

		FVector random_point = FVector(distributionx(generator), distributiony(generator), 0);
		while (isInAnyPolygon(random_point, map->allGroundPoints) || !isInPolygon(random_point, map->wallPoints)) {
			random_point = FVector(distributionx(generator), distributiony(generator), 0);
		}

		FVector random_vel = randVel(v_max);


		// find points in nodes that can see corner
		std::vector<std::shared_ptr<RRTNode>> visible_nodes;
		for (int i = 0; i < nodes.size(); ++i) {
			visible = map->Trace(nodes[i]->corner, visible_corners[corner_idx], -1);
			if (visible) visible_nodes.push_back(nodes[i]);
		}


		// find paths from points to random point
		//file_log("S");
		float best_cost = 999999;
		std::shared_ptr<RRTNode> best_segment;
		bool found_segment = false;
		for (int i = 0; i < visible_nodes.size(); ++i) {
			std::shared_ptr<Path> temp_path = pathFcn(visible_nodes[i]->pos, visible_nodes[i]->vel, random_point, random_vel);
			if (temp_path->isValid() && temp_path->pathExists()) {
				std::shared_ptr<RRTNode> temp_node = std::make_shared<RRTNode>(visible_nodes[i], temp_path, visible_corners[corner_idx]);
				if (temp_node->cost < best_cost) {
					best_cost = temp_node->cost;
					found_segment = true;
					best_segment = temp_node;
				}
			}
		}

		if (found_segment) {
			nodes.push_back(best_segment);
		}
	}
	if (found_path) {
		std::shared_ptr<RRTNode> asdf = best_path;
		while (asdf->cost != 0) {
			out_path.push_back(asdf);
			asdf = asdf->child;
		}

		std::reverse(out_path.begin(), out_path.end());
	}

	return out_path;
}




















