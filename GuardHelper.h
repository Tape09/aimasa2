// Fill out your copyright notice in the Description page of Project Settings.

#pragma once


#include "Math.h"
#include <cmath>
#include <vector>
#include <string>
#include <fstream>
#include <cstdio>
#include <unordered_set>
#include <algorithm>
#include "PathPlanner_KP.h"

struct IntSet {
	std::unordered_set<int> ints;

	IntSet operator-(const IntSet & other) {
		IntSet out;
		for (const int & i : ints) {
			if (other.ints.count(i) == 0) {
				out.ints.insert(i);
			}
		}
		return out;
	}

	IntSet operator+(const IntSet & other) {
		IntSet out;
		out.ints.insert(ints.begin(), ints.end());
		out.ints.insert(other.ints.begin(), other.ints.end());
		return out;
	}

	IntSet & operator+=(const IntSet & other) {
		ints.insert(other.ints.begin(), other.ints.end());
		return *this;
	}

	IntSet & operator-=(const IntSet & other) {
		for (const int & i : other.ints) {
			ints.erase(i);
		}
		return *this;
	}


};

struct Guard {
	FVector pos;
	IntSet items;
	bool is_start = false;
	bool is_goal = false;
};

typedef std::pair<Guard*, Guard*> PGuardPair;


// Random set of "guards" that cover "cover".
std::vector<Guard*> randomCover(std::vector<Guard*> guards, IntSet cover);

// random cover that always keeps start and goal points
std::vector<Guard*> randomCoverSpecial(std::vector<Guard*> guards, IntSet cover);


struct UnorderedEqual {
	template <class T1, class T2>
	bool operator() (const std::pair<T1, T2> &lhs, const std::pair<T1, T2> &rhs) const {
		return lhs == rhs || (
			lhs.second == rhs.first && rhs.second == lhs.first
			);
	}
};

template<typename T>
void hash_combine(std::size_t &seed, T const &key) {
	std::hash<T> hasher;
	seed ^= hasher(key) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

namespace std {
	template<typename T1, typename T2>
	struct hash<std::pair<T1, T2>> {
		std::size_t operator()(std::pair<T1, T2> const &p) const {
			std::size_t seed1(0);
			::hash_combine(seed1, p.first);
			::hash_combine(seed1, p.second);

			std::size_t seed2(0);
			::hash_combine(seed2, p.second);
			::hash_combine(seed2, p.first);

			return std::min(seed1, seed2);
		}
	};
}








