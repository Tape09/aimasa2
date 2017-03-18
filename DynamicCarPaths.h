// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "Path.h"
#include <vector>
#include "MyMath.h"
#include <functional>
#include <algorithm>





class AIMASA2_API DynamicCarPaths : public Path {
public:

	enum Turn { S, L, R };

	struct RSComponent {
		RSComponent(Turn turn_, int gear_, float angle_) : turn(turn_), gear(gear_), angle(angle_) {
			if (angle*gear < 0) {
				dist = pii + pii - abs(angle);
			} else {
				dist = abs(angle);
			}

		}

		Turn turn;
		int gear;
		float angle;
		float dist;



		void reverse() { gear = -gear; }

		void reflect() {
			if (turn == L) {
				turn = R;
			} else if (turn == R) {
				turn = L;
			}
		}

		FString toString() {
			FString trn;
			if (turn == L) {
				trn = "L";
			} else if (turn == S) {
				trn = "S";
			} else {
				trn = "R";
			}

			FString out = trn + "::" + FString::FromInt(gear) + "::" + FString::SanitizeFloat(dist) + "::" + FString::SanitizeFloat(angle);
			return out;
		}
	};

	struct RSState {
		RSState(FVector pos_ = FVector(0, 0, 0), float theta_ = 0, int gear_ = 1) : pos(pos_), theta(theta_), gear(gear_) {}

		FVector pos;
		float theta;
		//float phi;
		int gear;

		void reverse() {
			pos.X = -pos.X;
			theta = -theta;
		}

		void reflect() {
			pos.Y = -pos.Y;
			theta = -theta;
		}
	};

	struct RSPath {
		std::vector<RSComponent> components;
		bool is_valid;
		float dist;
		//float fdist;
		float time;
		std::vector<float> times;
		std::vector<float> dists;

		std::vector<AccelerationInfo> ais;

		int get_rsc_idx(float t) {
			if (t >= time) {
				return dist;
			} else {
				for (int i = 1; i < times.size(); ++i) {
					if (t <= times[i]) {
						return i - 1;						
					}
				}
			}
		}

		float calc_time(float v0, float v3, float turn_rate, float v_max, float a) {
			std::vector<float> dist_breakpoints;
			std::vector<float> speeds;

			float running_dist = 0;
			dist_breakpoints.push_back(running_dist);
			speeds.push_back(v0);
			for (int i = 0; i < components.size()-1; ++i) {
				running_dist += components[i].dist * turn_rate;
				if (components[i].gear != components[i+1].gear) {
					dist_breakpoints.push_back(running_dist);
					speeds.push_back(0);
				}
			}
			running_dist += components.back().dist * turn_rate;
			dist_breakpoints.push_back(running_dist);
			speeds.push_back(v3);
			//fdist = running_dist;

			time = 0;
			times.push_back(0);
			for (int i = 0; i < dist_breakpoints.size() - 1; ++i) {
				ais.push_back(accelerate_between(dist_breakpoints[i],speeds[i],dist_breakpoints[i+1],speeds[i+1],v_max,a));
				time += ais.back().t1 + ais.back().t2 + ais.back().t3;
				times.push_back(time);
				if (!ais.back().isValid) {
					is_valid = false;
					break;
				}
			}
			
			return time;
		}

		float dist_at(float t) {
			//float d;

			int ais_idx = ais.size()-1;
			float rel_t = times.back();

			if (t >= time) {
				return dist;
			} else {
				for (int i = 1; i < times.size(); ++i) {
					if (t <= times[i]) {
						ais_idx = i-1;
						rel_t = t-times[i-1];	
						break;				
					}
				}
			}

			//print_log(FString::SanitizeFloat(ais[0].t1));
			//print_log(FString::SanitizeFloat(ais[0].t2));
			//print_log(FString::SanitizeFloat(ais[0].t3));
			//print_log(FString::SanitizeFloat(ais[0].p1));
			//print_log(FString::SanitizeFloat(dists.size()));
			

			PosVel pv = ais[ais_idx].pos_vel_at(rel_t);

			//float x1 = ais[ais_idx].a * ais[ais_idx].t1 * ais[ais_idx].t1 / 2 + ais[ais_idx].v0 * ais[ais_idx].t1 + ais[ais_idx].p0;
			//float x2 = ais[ais_idx].v1 * ais[ais_idx].t2 + x1;
			//float x3 = -ais[ais_idx].a * ais[ais_idx].t3 * ais[ais_idx].t3 / 2 + ais[ais_idx].v2 * ais[ais_idx].t3 + x2;

			//print_log(FString::SanitizeFloat(x3));
			//print_log(FString::SanitizeFloat(ais[ais_idx].p3));

			

			return pv.pos;
		}

		float calc_dist() {
			float d = 0;
			dists.push_back(d);
			for (int i = 0; i < components.size(); ++i) {
				d += components[i].dist;
				dists.push_back(d);
			}
			dist = d;
			return d;
		}

		void reverse() {
			for (int i = 0; i < components.size(); ++i) {
				components[i].reverse();
			}
		}

		void reflect() {
			for (int i = 0; i < components.size(); ++i) {
				components[i].reflect();
			}
		}

		bool operator<(const RSPath & other) const {
			return time<other.time;
		}

		int size() { return components.size(); }

		FString word() {
			FString out = "";
			for (int i = 0; i < components.size(); ++i) {
				if (components[i].turn == L) {
					out += "L";
				} else if (components[i].turn == S) {
					out += "S";
				} else {
					out += "R";
				}

				if (components[i].gear == 1) {
					out += "f";
				} else {
					out += "b";
				}
			}
			return out;
		}

	};

	float theta(float t) const;

	State drive_R(State istate, RSComponent rsc, float dist) const;
	State drive_L(State istate, RSComponent rsc, float dist) const;
	State drive_S(State istate, RSComponent rsc, float dist) const;

	float path_time(int idx) const;
	virtual float path_time() const;
	State state_at(State istate, RSComponent rsc, float time) const;
	State state_at(RSPath rsp, float time) const;
	State state_at(int idx, float time) const;
	int n_paths() const;

	typedef std::function<RSPath(const RSState & goal)> pathFcn;

	//1: 8.1
	RSPath get_path_LSL(const RSState & goal);
	//2: 8.2
	RSPath get_path_LSR(const RSState & goal);
	//3: 8.3
	RSPath get_path_LGRGL(const RSState & goal);
	//4: 8.4
	RSPath get_path_LGRL(const RSState & goal);
	//5: 8.4
	RSPath get_path_LRGL(const RSState & goal);
	//6: 8.7
	RSPath get_path_LRGLR(const RSState & goal);
	//7: 8.8
	RSPath get_path_LGRLGR(const RSState & goal);
	//8: 8.9
	RSPath get_path_LGR90SL(const RSState & goal);
	//9: 8.9
	RSPath get_path_LSR90GL(const RSState & goal);
	//10: 8.10
	RSPath get_path_LGR90SR(const RSState & goal);
	//11: 8.10
	RSPath get_path_LSL90GR(const RSState & goal);
	//12: 8.11
	RSPath get_path_LGR90SL90GR(const RSState & goal);

	void addTransforms(pathFcn fptr, const RSState & goal);

	// void fixVector(FVector & fv);
	// void fixState(State & s);
	// find time intervals
	// solve time intervals

	DynamicCarPaths(FVector pos0, FVector vel0, FVector pos1, FVector vel1, float v_max, float phi_max, float L_car, float a_max);
	~DynamicCarPaths();

	virtual State step(float delta_time);
	virtual State state_at(float time);

	std::vector<RSPath> all_paths;
	float time_taken();
	float time_taken(int idx);

	//FVector pos0;
	//FVector pos1;

	//FVector vel0;
	//FVector vel1;

	float v_max;
	float phi_max;
	float L_car;
	float turn_radius;
	float a_max;

	int path_index;
};
