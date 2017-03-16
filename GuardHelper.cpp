// Fill out your copyright notice in the Description page of Project Settings.

#include "aimasa2.h"
#include "GuardHelper.h"

std::vector<Guard*> randomCover(std::vector<Guard*> guards, IntSet cover) {
	std::vector<Guard*> out;

	int oldsize = cover.ints.size();
	std::random_shuffle(guards.begin(),guards.end());
	for (int i = 0; i < guards.size(); ++i) {
		cover -= guards[i]->items;
		if (cover.ints.size() != oldsize) {
			oldsize = cover.ints.size();
			out.push_back(guards[i]);
			if (cover.ints.empty()) {
				return out;
			}
		}		
	}
	return std::vector<Guard*>();
}


std::vector<Guard*> randomCoverSpecial(std::vector<Guard*> guards, IntSet cover) {
	std::vector<Guard*> original = guards;
	std::unordered_set<Guard*> new_cover;

	int oldsize = cover.ints.size();
	std::random_shuffle(guards.begin(), guards.end());
	for (int i = 0; i < guards.size(); ++i) {
		cover -= guards[i]->items;

		if (cover.ints.size() != oldsize) {
			oldsize = cover.ints.size();
			new_cover.insert(guards[i]);
			if (cover.ints.empty()) {
				break;
			}
		}
	}

	std::vector<Guard*> out;
	for (int i = 0; i < original.size(); ++i) {
		if (original[i]->is_start) {
			out.push_back(original[i]);
			continue;
		}

		if (original[i]->is_goal) {
			out.push_back(original[i]);
			continue;
		}

		if (new_cover.count(original[i]) == 1) {
			out.push_back(original[i]);
		}
	}

	return out;
}

//std::vector<Guard*> randomCoverSpecial(std::vector<Guard*> guards, IntSet cover) {
//	std::vector<int> idxs(guards.size());
//	for (int i = 0; i < idxs.size(); ++i) {
//		idxs[i] = i;
//	}
//
//	std::vector<int> out_idxs;
//	std::random_shuffle(idxs.begin(), idxs.end());
//
//	int oldsize = cover.ints.size();
//	for (int i = 0; i < idxs.size(); ++i) {		
//		cover -= guards[idxs[i]]->items;
//
//		if (guards[idxs[i]]->is_start) {
//			out_idxs.push_back(idxs[i]);
//		}
//
//		if (cover.ints.size() != oldsize) {
//			oldsize = cover.ints.size();
//			out_idxs.push_back(idxs[i]);
//			if (cover.ints.empty()) {
//				break;
//			}
//		}
//	}
//
//	std::sort(out_idxs.begin(), out_idxs.end());
//	std::vector<Guard*> out;
//
//	for (int i = 0; i<out_idxs.size(); ++i) {
//		out.push_back(guards[out_idxs[i]]);
//	}
//
//	return out;
//}

//std::vector<Edge_MTSP> randomCoverSpecial(std::vector<Edge_MTSP> edges, IntSet cover, const PathPlanner_KP & planner) {
//
//	std::unordered_map<Guard*,Guard*> path;
//
//	std::unordered_set<Guard*> used_guards;
//
//	std::vector<int> idxs(edges.size());
//	for (int i = 0; i < idxs.size(); ++i) {
//		idxs[i] = i;
//		path[edges[i].from] = edges[i].to;
//	}
//
//	std::vector<int> out_idxs;
//	std::random_shuffle(idxs.begin(), idxs.end());
//
//	int oldsize = cover.ints.size();
//	for (int i = 0; i < idxs.size(); ++i) {
//		cover -= edges[idxs[i]].from->items;
//
//		if (edges[idxs[i]].from->is_start) {
//			out_idxs.push_back(idxs[i]);
//			used_guards.insert(edges[idxs[i]].from);
//		}
//
//		if (cover.ints.size() != oldsize) {
//			oldsize = cover.ints.size();
//			out_idxs.push_back(idxs[i]);
//			used_guards.insert(edges[idxs[i]].from);
//			if (cover.ints.empty()) {
//				break;
//			}
//		}
//	}
//
//	std::sort(out_idxs.begin(), out_idxs.end());
//	std::vector<Guard*> out;
//
//	//for (int i = 0; i<out_idxs.size(); ++i) {
//	//	out.push_back(guards[out_idxs[i]]);
//	//}
//
//	return std::vector<Edge_MTSP>();
//
//
//}
//
//
//
//
//
//std::vector<Edge_MTSP> create_path(std::vector<Guard*> guards) {
//	std::vector<Edge_MTSP> out;
//	for (int i = 0; i < guards.size()-1; ++i) {
//		//Edge_MTSP temp;
//		//temp.
//	}
//	return std::vector<Edge_MTSP>();
//}








