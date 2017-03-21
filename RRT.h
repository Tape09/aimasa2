// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include <functional>
#include <algorithm>
#include <vector>
#include "Path.h"
#include <memory>
#include "MapGen.h"
#include <random>
#include <fstream>
#include <iostream>
#include <chrono>
/**
 * 
 */
typedef std::function<std::shared_ptr<Path>(FVector pos0, FVector vel0, FVector pos1, FVector vel1)> PathFcnType;

struct RRTNode {
	RRTNode(FVector pos_, FVector vel_) {
		child = NULL;
		path = NULL;
		corner = pos_;
		cost = 0;
		pos = pos_;
		vel = vel_;
	}

	RRTNode(std::shared_ptr<RRTNode> child_, std::shared_ptr<Path> path_, FVector corner_) {
		child = child_;
		path = path_;
		corner = corner_;
		cost = child->cost + path->path_time();
		pos = path->pos_1();
		vel = path->vel_1();
	}

	float cost;
	FVector corner;
	FVector pos;
	FVector vel;

	std::shared_ptr<RRTNode> child;
	std::shared_ptr<Path> path; // path from child to me
	
};

State state_at(std::vector<std::shared_ptr<RRTNode>> path, float time);

class AIMASA2_API RRT
{
public:
	RRT(int max_iterations, AMapGen * map_, PathFcnType pathFcn_, float v_max, float a_max);
	~RRT();

	
	std::vector<std::shared_ptr<RRTNode>> get_full_path(FVector start_pos, FVector start_vel, FVector goal_pos, FVector goal_vel);
	std::vector<std::shared_ptr<RRTNode>> get_full_path_hint(FVector start_pos, FVector start_vel, FVector goal_pos, FVector goal_vel, std::vector<std::shared_ptr<RRTNode>> & hint_path);
	std::vector<std::shared_ptr<RRTNode>> get_full_path2();
	std::vector<std::shared_ptr<RRTNode>> nodes;

	std::default_random_engine generator;

	float v_max;
	float a_max;

	float sigma2 = 100;
	int max_iterations;

	PathFcnType pathFcn;
	AMapGen * map;
};
