// Fill out your copyright notice in the Description page of Project Settings.

#include "aimasa2.h"
#include "PathPlanner_KP.h"

PathPlanner_KP::PathPlanner_KP(AMapGen * map_) {	
	init(map_);
}

PathPlanner_KP::~PathPlanner_KP()
{
}

void PathPlanner_KP::init(AMapGen * map_) {
	map = map_;
	build_vgraph();

	dmat = std::vector< std::vector<Path_KP> >(map->allPoints.Num());
	for (int i = 0; i < map->allPoints.Num(); ++i) {
		dmat[i] = std::vector<Path_KP>(map->allPoints.Num());
		for (int j = 0; j < map->allPoints.Num(); ++j) {
			if (i == j) {
				dmat[i][j].dist = 0;
				dmat[i][j].waypoints.push_back(map->allPoints[j]);
			}

			dmat[i][j] = find_path2(map->allPoints[i], map->allPoints[j]);
		}
	}
}


void PathPlanner_KP::build_vgraph() {
	vgraph.nodes.clear();

	for (int i = 0; i < map->allPoints.Num(); ++i) {
		VisibilityGraphNode vgn;
		vgn.pos = i;

		for (int j = 0; j < map->allPoints.Num(); ++j) {
			if(i==j) continue;

			//check if visible
			if (map->Trace(map->allPoints[i], map->allPoints[j])) {
				vgn.edges[j] = (FVector::Dist(map->allPoints[i], map->allPoints[j]));
			}
		}
		vgraph.nodes.push_back(vgn);
	}
	
	
}


void PathPlanner_KP::draw_vgraph() {
	for (int i = 0; i < vgraph.nodes.size(); ++i) {
		FVector from = map->allPoints[vgraph.nodes[i].pos];
		for (auto it = vgraph.nodes[i].edges.begin(); it != vgraph.nodes[i].edges.end(); ++it) {
			FVector to = map->allPoints[it->first];
			map->drawLine(from,to);
		}
	}

}


Path_KP PathPlanner_KP::find_path(FVector from, FVector to) const {
	if (from == to) {
		Path_KP out;
		out.dist = 0.0;
		out.waypoints.push_back(to);
		return out;
	}

	if (map->Trace(from, to)) {
		Path_KP out;
		out.dist = FVector::Dist(from, to);;
		out.waypoints.push_back(from);
		out.waypoints.push_back(to);
		return out;
	}

	std::vector<std::pair<int, float>> visible_from;
	std::vector<std::pair<int, float>> visible_to;

	for (int i = 0; i < map->allPoints.Num(); ++i) {
		if (map->Trace(from, map->allPoints[i])) {
			std::pair<int,float> temp;
			temp.first = i;
			temp.second = FVector::Dist(from,map->allPoints[i]);
			visible_from.push_back(temp);
		}

		if (map->Trace(to, map->allPoints[i])) {
			std::pair<int, float> temp;
			temp.first = i;
			temp.second = FVector::Dist(map->allPoints[i],to);
			visible_to.push_back(temp);
		}
	}

	int best_vfrom = 0;
	int best_vto = 0;
	float best_dist = 99999999;
	float new_dist;

	for (int i = 0; i < visible_from.size(); ++i) {
		for (int j = 0; j < visible_to.size(); ++j) {
			new_dist = dmat[visible_from[i].first][visible_to[j].first].dist + visible_from[i].second + visible_to[j].second;
			if (new_dist < best_dist) {
				best_vfrom = visible_from[i].first;
				best_vto = visible_to[j].first;
				best_dist = new_dist;
			}
		}
	}

	Path_KP out;
	
	out.dist = best_dist;
	out.waypoints.push_back(from);
	out.waypoints.push_back(map->allPoints[best_vfrom]);

	if (best_vfrom != best_vto) {
		out.waypoints.insert(out.waypoints.end(), dmat[best_vfrom][best_vto].waypoints.begin(), dmat[best_vfrom][best_vto].waypoints.end());
	} 	
	
	out.waypoints.push_back(to);
		
	return out;
}


Path_KP PathPlanner_KP::find_path2(FVector from, FVector to) const {
	if (map->Trace(from, to)) {
		Path_KP out;
		out.dist = FVector::Dist(from, to);;
		out.waypoints.push_back(to);
		return out;
	}

	std::priority_queue<DNode> Q;

	for (int i = 0; i < map->allPoints.Num(); ++i) {
		if (map->Trace(from, map->allPoints[i])) {
			DNode temp;
			temp.path.reserve(7);
			temp.dist = FVector::Dist(from, map->allPoints[i]);
			temp.path.push_back(i);
			Q.push(temp);
		}
	}

	//int no_crash = 1000;
	DNode current;
	while (!Q.empty()) {
		//if(no_crash-- < 0) return Path_KP();


		current = Q.top();
		Q.pop();

		int idx = current.path.back();

		if (idx == -1) {
			Path_KP out;
			out.dist = current.dist;
			out.waypoints.reserve(current.path.size());
			for (int i = 0; i < current.path.size() - 1; ++i) {
				out.waypoints.push_back(map->allPoints[current.path[i]]);
			}
			out.waypoints.push_back(to);
			return out;
		}

		DNode new_node;
		for (auto it = vgraph.nodes[idx].edges.begin(); it != vgraph.nodes[idx].edges.end(); ++it) {
			if (std::find(current.path.begin(), current.path.end(), it->first) != current.path.end()) continue;
			new_node.dist = current.dist + it->second;
			new_node.path = current.path;
			new_node.path.push_back(it->first);
			Q.push(new_node);
		}

		if (map->Trace(map->allPoints[current.path.back()], to)) {
			new_node.dist = current.dist + FVector::Dist(map->allPoints[current.path.back()], to);
			new_node.path = current.path;
			new_node.path.push_back(-1);
			Q.push(new_node);
		}

	}

	return Path_KP();
}



























