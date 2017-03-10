// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "MapGen.h"
#include "MyMath.h"
#include <unordered_map>
#include <vector>



struct Path_KP {
	float dist;
	std::vector<FVector> waypoints;
};

class AIMASA2_API PathPlanner_KP
{
public:

	struct VisibilityGraphNode {
		int pos;
		std::unordered_map<int, float> edges;
	};

	struct VisibilityGraph {
		std::vector<VisibilityGraphNode> nodes;
	};

	struct DNode {
		float dist;
		std::vector<int> path;

		bool operator<(const DNode & other) const {
			return dist > other.dist;
		}
	};

	Path_KP find_path(FVector from, FVector to) const ;
	Path_KP find_path2(FVector from, FVector to) const;



	PathPlanner_KP(AMapGen * map);
	PathPlanner_KP() {};

	void init(AMapGen * map);

	~PathPlanner_KP();

	void build_vgraph();
	void draw_vgraph();

	std::vector< std::vector<Path_KP> > dmat;

	VisibilityGraph vgraph;
	AMapGen * map;


};
