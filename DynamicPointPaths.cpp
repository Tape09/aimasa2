// Fill out your copyright notice in the Description page of Project Settings.

#include "aimasa2.h"
#include "DynamicPointPaths.h"

DynamicPointPaths::DynamicPointPaths(FVector pos0_, FVector vel0_, FVector pos1_, FVector vel1_, float v_max_, float phi_max_, float L_car_, float a_max_) {

	pos0 = pos0_;
	pos1 = pos1_;
	vel0 = vel0_;
	vel1 = vel1_;

	v_max = v_max_;
	phi_max = phi_max_;
	L_car = L_car_;
	a_max = a_max_;

	turn_radius = L_car / tan(phi_max);
	//turn_radius = 2;
	//print_log(FString::SanitizeFloat(turn_radius));
	//print_log(FString::SanitizeFloat(phi_max));

	RSState goal_state((pos1 - pos0) / turn_radius, wrapAngle(vecAngle(vel1) - vecAngle(vel0)));
	rotateVector(goal_state.pos, -vecAngle(vel0));
	//goal_state.pos = goal_state.pos.RotateAngleAxis(rad2deg(-vecAngle(vel0)),FVector(0,0,1));

	//print_log(goal_state.pos.ToString());
	//print_log(FString::SanitizeFloat(goal_state.theta));

	addTransforms(std::bind(&DynamicPointPaths::get_path_LSL, this, std::placeholders::_1), goal_state);
	addTransforms(std::bind(&DynamicPointPaths::get_path_LSR, this, std::placeholders::_1), goal_state);
	//addTransforms(std::bind(&DynamicPointPaths::get_path_LGRGL, this, std::placeholders::_1), goal_state);
	//addTransforms(std::bind(&DynamicPointPaths::get_path_LGRL, this, std::placeholders::_1), goal_state);
	//addTransforms(std::bind(&DynamicPointPaths::get_path_LRGL, this, std::placeholders::_1), goal_state);
	//addTransforms(std::bind(&DynamicPointPaths::get_path_LRGLR, this, std::placeholders::_1), goal_state);
	//addTransforms(std::bind(&DynamicPointPaths::get_path_LGRLGR, this, std::placeholders::_1), goal_state);
	//addTransforms(std::bind(&DynamicPointPaths::get_path_LGR90SL, this, std::placeholders::_1), goal_state);
	//addTransforms(std::bind(&DynamicPointPaths::get_path_LSR90GL, this, std::placeholders::_1), goal_state);
	//addTransforms(std::bind(&DynamicPointPaths::get_path_LGR90SR, this, std::placeholders::_1), goal_state);
	//addTransforms(std::bind(&DynamicPointPaths::get_path_LSL90GR, this, std::placeholders::_1), goal_state);
	//addTransforms(std::bind(&DynamicPointPaths::get_path_LGR90SL90GR, this, std::placeholders::_1), goal_state);

	std::sort(all_paths.begin(), all_paths.end());

	if (all_paths.size() == 0) exists = false;
	path_index = 0;



}

DynamicPointPaths::~DynamicPointPaths() {}

float DynamicPointPaths::time_taken(int idx) {
	return turn_radius * all_paths[idx].calc_dist() / v_max;
}

float DynamicPointPaths::time_taken() {
	return turn_radius * all_paths[0].calc_dist() / v_max;
}


State DynamicPointPaths::state_at(RSPath rsp, float time) const {
	//print_log("time: " + FString::SanitizeFloat(time));
	float d = rsp.dist_at(time);
	//int idx = rsp.get_rsc_idx(time);


	//print_log("dist: " + FString::SanitizeFloat(d));
	//print_log("time: " + FString::SanitizeFloat(time));
	//print_log("dist: " + FString::SanitizeFloat(rsp.ais.back().p3));
	//print_log("tim: " + FString::SanitizeFloat(rsp.time));

	State istate;
	istate.pos = pos0;
	istate.vel = vel0;

	//print_log(istate);

	//print_log(FString::SanitizeFloat(rsp.dist));
	//print_log(FString::SanitizeFloat(rsp.dist*turn_radius));
	//print_log(rsp.word());


	for (int i = 1; i<rsp.dists.size(); ++i) {
		//print_log(FString::SanitizeFloat(rsp.dists[i]));
		if (d <= rsp.dists[i] * turn_radius) {
			return state_at(istate, rsp.components[i - 1], (d - rsp.dists[i - 1] * turn_radius));
		} else {
			istate = state_at(istate, rsp.components[i - 1], rsp.components[i - 1].dist*turn_radius);
		}
		//print_log(istate);
	}

	return istate;

}

State DynamicPointPaths::state_at(State istate, RSComponent rsc, float dist) const {
	State s;

	if (rsc.turn == L) {
		s = drive_L(istate, rsc, dist);
	} else if (rsc.turn == R) {
		s = drive_R(istate, rsc, dist);
	} else {
		s = drive_S(istate, rsc, dist);
	}



	return s;
}

State DynamicPointPaths::drive_R(State istate, RSComponent rsc, float dist) const {
	State s;
	float theta0 = vecAngle(istate.vel);

	float theta = rsc.gear * dist / turn_radius;
	float LL = 2 * sin(theta / 2) * turn_radius;
	s.pos.X = LL * cos(theta / 2);
	s.pos.Y = -LL * sin(theta / 2);

	s.vel = istate.vel;
	rotateVector(s.vel, -theta);

	s.acc = FVector(0, turn_radius, 0) - s.pos;

	rotateVector(s.pos, theta0);
	rotateVector(s.acc, theta0);

	s.vel.Normalize();
	s.vel = s.vel * v_max;

	s.pos = s.pos + istate.pos;


	return s;
}

State DynamicPointPaths::drive_L(State istate, RSComponent rsc, float dist) const {
	State s;
	float theta0 = vecAngle(istate.vel);


	float theta = rsc.gear * dist / turn_radius;
	float LL = 2 * sin(theta / 2) * turn_radius;
	s.pos.X = LL * cos(theta / 2);
	s.pos.Y = LL * sin(theta / 2);

	s.vel = istate.vel;
	rotateVector(s.vel, theta);

	s.acc = FVector(0, turn_radius, 0) - s.pos;

	rotateVector(s.pos, theta0);
	rotateVector(s.acc, theta0);

	s.vel.Normalize();
	s.vel = s.vel * v_max;

	s.pos = s.pos + istate.pos;


	return s;
}

State DynamicPointPaths::drive_S(State istate, RSComponent rsc, float dist) const {
	State s;
	float theta0 = vecAngle(istate.vel);
	dist = rsc.gear * dist;

	s.pos.X = dist*cos(theta0) + istate.pos.X;
	s.pos.Y = dist*sin(theta0) + istate.pos.Y;

	s.vel = istate.vel;
	s.acc = FVector(0, 0, 0);

	return s;
}


State DynamicPointPaths::state_at(int idx, float time) const {
	return state_at(all_paths[idx], time);
}

int DynamicPointPaths::n_paths() const {
	return all_paths.size();
}


float DynamicPointPaths::theta(float t) const {
	return v_max*t / turn_radius;
}

State DynamicPointPaths::step(float delta_time) {
	t_now += delta_time;
	return state_at(t_now);
}

State DynamicPointPaths::state_at(float t) {
	return state_at(path_index, t);
}

//1: 8.1
DynamicPointPaths::RSPath DynamicPointPaths::get_path_LSL(const RSState & goal) {
	float t = 0;
	float u = 0;
	float v = 0;

	float x = goal.pos.X - sin(goal.theta);
	float y = goal.pos.Y - 1 + cos(goal.theta);

	t = atan2(y, x);
	u = sqrt(x * x + y * y);
	v = wrapAngle(goal.theta - t);




	RSPath out_path;
	out_path.components.push_back(RSComponent(L, 1, t));
	out_path.components.push_back(RSComponent(S, 1, u));
	out_path.components.push_back(RSComponent(L, 1, v));
	out_path.is_valid = true;
	out_path.calc_dist();
	out_path.calc_time(vel0.Size(), vel1.Size(), turn_radius, v_max, a_max);

	return out_path;
}
//2: 8.2
DynamicPointPaths::RSPath DynamicPointPaths::get_path_LSR(const RSState & goal) {

	RSPath out_path;

	float t = 0;
	float u = 0;
	float v = 0;

	float x = goal.pos.X + sin(goal.theta);
	float y = goal.pos.Y - 1 - cos(goal.theta);

	float u1_2 = x * x + y * y;
	float t1 = atan2(y, x);

	if (u1_2 < 4) {
		out_path.is_valid = false;
		return out_path;
	}

	u = sqrt(u1_2 - 4);
	t = wrapAngle(t1 + atan2(2, u));
	v = wrapAngle(t - goal.theta);



	out_path.components.push_back(RSComponent(L, 1, t));
	out_path.components.push_back(RSComponent(S, 1, u));
	out_path.components.push_back(RSComponent(R, 1, v));
	out_path.is_valid = true;
	out_path.calc_dist();
	out_path.calc_time(vel0.Size(), vel1.Size(), turn_radius, v_max, a_max);


	return out_path;
}
//3: 8.3
DynamicPointPaths::RSPath DynamicPointPaths::get_path_LGRGL(const RSState & goal) {
	RSPath out_path;

	float t = 0;
	float u = 0;
	float v = 0;

	float xi = goal.pos.X - sin(goal.theta);
	float eta = goal.pos.Y - 1 + cos(goal.theta);

	float u1 = sqrt(xi * xi + eta * eta);
	//print_log(FString::SanitizeFloat(u1));
	if (u1 > 4) {
		out_path.is_valid = false;
		return out_path;
	}


	float alpha = acos(u1 / 4);
	t = mod2pi(pii / 2 + alpha + atan2(eta, xi));
	u = mod2pi(pii - 2 * alpha);
	v = mod2pi(goal.theta - t - u);

	// if (isInvalidAngle(t) || isInvalidAngle(u) || isInvalidAngle(v))
	// return float.PositiveInfinity;

	// return t + u + v;


	out_path.components.push_back(RSComponent(L, 1, t));
	out_path.components.push_back(RSComponent(R, -1, -u));
	out_path.components.push_back(RSComponent(L, 1, v));
	out_path.is_valid = true;
	out_path.calc_dist();
	out_path.calc_time(vel0.Size(), vel1.Size(), turn_radius, v_max, a_max);


	return out_path;
}
//4: 8.4
DynamicPointPaths::RSPath DynamicPointPaths::get_path_LGRL(const RSState & goal) {
	RSPath out_path;

	float t = 0;
	float u = 0;
	float v = 0;

	float xi = goal.pos.X - sin(goal.theta);
	float eta = goal.pos.Y - 1 + cos(goal.theta);

	float u1 = sqrt(xi * xi + eta * eta);
	if (u1 > 4) {
		out_path.is_valid = false;
		return out_path;
	}


	float alpha = acos(u1 / 4);
	t = mod2pi(pii / 2 + alpha + atan2(eta, xi));
	u = mod2pi(pii - 2 * alpha);
	v = mod2pi(t + u - goal.theta);


	out_path.components.push_back(RSComponent(L, 1, t));
	out_path.components.push_back(RSComponent(R, -1, -u));
	out_path.components.push_back(RSComponent(L, -1, -v));
	out_path.is_valid = true;
	out_path.calc_dist();
	out_path.calc_time(vel0.Size(), vel1.Size(), turn_radius, v_max, a_max);


	return out_path;
}
//5: 8.4
DynamicPointPaths::RSPath DynamicPointPaths::get_path_LRGL(const RSState & goal) {
	RSPath out_path;

	float t = 0;
	float u = 0;
	float v = 0;

	float xi = goal.pos.X - sin(goal.theta);
	float eta = goal.pos.Y - 1 + cos(goal.theta);

	float u1 = sqrt(xi * xi + eta * eta);
	if (u1 > 4) {
		out_path.is_valid = false;
		return out_path;
	}


	u = acos((8 - u1 * u1) / 8);
	float va = sin(u);
	float vb = 2 * va / u1;

	if (abs(vb) > 1) {
		out_path.is_valid = false;
		return out_path;
	}

	float alpha = asin(vb);
	t = mod2pi(pii / 2 - alpha + atan2(eta, xi));
	v = mod2pi(t - u - goal.theta);

	out_path.components.push_back(RSComponent(L, 1, t));
	out_path.components.push_back(RSComponent(R, 1, u));
	out_path.components.push_back(RSComponent(L, -1, -v));
	out_path.is_valid = true;
	out_path.calc_dist();
	out_path.calc_time(vel0.Size(), vel1.Size(), turn_radius, v_max, a_max);

	return out_path;
}
//6: 8.7
DynamicPointPaths::RSPath DynamicPointPaths::get_path_LRGLR(const RSState & goal) {
	RSPath out_path;

	float t = 0;
	float u = 0;
	float v = 0;

	float xi = goal.pos.X + sin(goal.theta);
	float eta = goal.pos.Y - 1 - cos(goal.theta);

	float u1 = sqrt(xi * xi + eta * eta);
	if (u1 > 4) {
		out_path.is_valid = false;
		return out_path;
	}
	float phii = atan2(eta, xi);

	if (u1 > 2) {
		float alpha = acos(u1 / 4 - 0.5);
		t = mod2pi(pii / 2 + phii - alpha);
		u = mod2pi(pii - alpha);
		v = mod2pi(goal.theta - t + 2 * u);
	} else {
		float alpha = acos(u1 / 4 + 0.5);
		t = mod2pi(pii / 2 + phii + alpha);
		u = mod2pi(alpha);
		v = mod2pi(goal.theta - t + 2 * u);
	}


	out_path.components.push_back(RSComponent(L, 1, t));
	out_path.components.push_back(RSComponent(R, 1, u));
	out_path.components.push_back(RSComponent(L, -1, -u));
	out_path.components.push_back(RSComponent(R, -1, -v));
	out_path.is_valid = true;
	out_path.calc_dist();
	out_path.calc_time(vel0.Size(), vel1.Size(), turn_radius, v_max, a_max);

	return out_path;
}
//7: 8.8
DynamicPointPaths::RSPath DynamicPointPaths::get_path_LGRLGR(const RSState & goal) {
	RSPath out_path;

	float t = 0;
	float u = 0;
	float v = 0;

	float xi = goal.pos.X + sin(goal.theta);
	float eta = goal.pos.Y - 1 - cos(goal.theta);

	float u1 = sqrt(xi * xi + eta * eta);

	if (u1 > 6) {
		out_path.is_valid = false;
		return out_path;
	}


	float va1 = 1.25f - u1 * u1 / 16;
	if (va1 < 0 || va1 > 1) {
		out_path.is_valid = false;
		return out_path;
	}


	u = acos(va1);
	float va2 = sin(u);
	float va3 = 2 * va2 / u1;
	if (abs(va3) > 1) {
		out_path.is_valid = false;
		return out_path;
	}

	float alpha = asin(va3);
	t = mod2pi(pii / 2 + atan2(eta, xi) + alpha);
	v = mod2pi(t - goal.theta);


	out_path.components.push_back(RSComponent(L, 1, t));
	out_path.components.push_back(RSComponent(R, -1, -u));
	out_path.components.push_back(RSComponent(L, -1, -u));
	out_path.components.push_back(RSComponent(R, 1, v));
	out_path.is_valid = true;
	out_path.calc_dist();
	out_path.calc_time(vel0.Size(), vel1.Size(), turn_radius, v_max, a_max);


	return out_path;
}

//8: 8.9
DynamicPointPaths::RSPath DynamicPointPaths::get_path_LGR90SL(const RSState & goal) {
	RSPath out_path;

	float t = 0;
	float u = 0;
	float v = 0;

	float xi = goal.pos.X - sin(goal.theta);
	float eta = goal.pos.Y - 1 + cos(goal.theta);

	float u1_s = xi * xi + eta * eta;
	if (u1_s < 4) {
		out_path.is_valid = false;
		return out_path;
	}

	u = sqrt(u1_s - 4) - 2;
	if (u < 0) {
		out_path.is_valid = false;
		return out_path;
	}

	float alpha = atan2(2, u + 2);
	t = mod2pi(pii / 2 + atan2(eta, xi) + alpha);
	v = mod2pi(t + pii / 2 - goal.theta);



	out_path.components.push_back(RSComponent(L, 1, t));
	out_path.components.push_back(RSComponent(R, -1, -pii / 2));
	out_path.components.push_back(RSComponent(S, -1, -u));
	out_path.components.push_back(RSComponent(L, -1, -v));
	out_path.is_valid = true;
	out_path.calc_dist();
	out_path.calc_time(vel0.Size(), vel1.Size(), turn_radius, v_max, a_max);


	return out_path;
}
//9: 8.9
DynamicPointPaths::RSPath DynamicPointPaths::get_path_LSR90GL(const RSState & goal) {
	RSPath out_path;

	float t = 0;
	float u = 0;
	float v = 0;

	float xi = goal.pos.X - sin(goal.theta);
	float eta = goal.pos.Y - 1 + cos(goal.theta);

	float u1_s = xi * xi + eta * eta;
	if (u1_s < 4) {
		out_path.is_valid = false;
		return out_path;
	}



	u = sqrt(u1_s - 4) - 2;
	if (u < 0) {
		out_path.is_valid = false;
		return out_path;
	}

	float alpha = atan2(u + 2, 2);
	t = mod2pi(pii / 2 + atan2(eta, xi) - alpha);
	v = mod2pi(t - pii / 2 - goal.theta);


	out_path.components.push_back(RSComponent(L, 1, t));
	out_path.components.push_back(RSComponent(S, 1, u));
	out_path.components.push_back(RSComponent(R, 1, pii / 2));
	out_path.components.push_back(RSComponent(L, -1, -v));
	out_path.is_valid = true;
	out_path.calc_dist();
	out_path.calc_time(vel0.Size(), vel1.Size(), turn_radius, v_max, a_max);


	return out_path;
}
//10: 8.10
DynamicPointPaths::RSPath DynamicPointPaths::get_path_LGR90SR(const RSState & goal) {
	RSPath out_path;

	float t = 0;
	float u = 0;
	float v = 0;

	float xi = goal.pos.X + sin(goal.theta);
	float eta = goal.pos.Y - 1 - cos(goal.theta);

	float u1 = sqrt(xi * xi + eta * eta);
	if (u1 < 2) {
		out_path.is_valid = false;
		return out_path;
	}


	t = mod2pi(pii / 2 + atan2(eta, xi));
	u = u1 - 2;
	v = mod2pi(goal.theta - t - pii / 2);

	out_path.components.push_back(RSComponent(L, 1, t));
	out_path.components.push_back(RSComponent(R, -1, -pii / 2));
	out_path.components.push_back(RSComponent(S, -1, -u));
	out_path.components.push_back(RSComponent(R, -1, -v));
	out_path.is_valid = true;
	out_path.calc_dist();
	out_path.calc_time(vel0.Size(), vel1.Size(), turn_radius, v_max, a_max);


	return out_path;
}
//11: 8.10
DynamicPointPaths::RSPath DynamicPointPaths::get_path_LSL90GR(const RSState & goal) {
	RSPath out_path;

	float t = 0;
	float u = 0;
	float v = 0;

	float xi = goal.pos.X + sin(goal.theta);
	float eta = goal.pos.Y - 1 - cos(goal.theta);

	float u1 = sqrt(xi * xi + eta * eta);
	if (u1 < 2) {
		out_path.is_valid = false;
		return out_path;
	}

	t = mod2pi(atan2(eta, xi));
	u = u1 - 2;
	v = mod2pi(-t - pii / 2 + goal.theta);


	out_path.components.push_back(RSComponent(L, 1, t));
	out_path.components.push_back(RSComponent(S, 1, u));
	out_path.components.push_back(RSComponent(L, 1, pii / 2));
	out_path.components.push_back(RSComponent(R, -1, -v));
	out_path.is_valid = true;
	out_path.calc_dist();
	out_path.calc_time(vel0.Size(), vel1.Size(), turn_radius, v_max, a_max);


	return out_path;
}
//12: 8.11
DynamicPointPaths::RSPath DynamicPointPaths::get_path_LGR90SL90GR(const RSState & goal) {
	RSPath out_path;

	float t = 0;
	float u = 0;
	float v = 0;

	float xi = goal.pos.X + sin(goal.theta);
	float eta = goal.pos.Y - 1 - cos(goal.theta);

	float u1_s = xi * xi + eta * eta;
	if (u1_s < 4) { // <16
		out_path.is_valid = false;
		return out_path;
	}

	u = sqrt(u1_s - 4) - 4;
	if (u < 0) {
		out_path.is_valid = false;
		return out_path;
	}

	float alpha = atan2(2, u + 4);
	t = mod2pi(pii / 2 + atan2(eta, xi) + alpha);
	v = mod2pi(t - goal.theta);


	out_path.components.push_back(RSComponent(L, 1, t));
	out_path.components.push_back(RSComponent(R, -1, -pii / 2));
	out_path.components.push_back(RSComponent(S, -1, -u));
	out_path.components.push_back(RSComponent(L, -1, -pii / 2));
	out_path.components.push_back(RSComponent(R, 1, v));
	out_path.is_valid = true;
	out_path.calc_dist();
	out_path.calc_time(vel0.Size(), vel1.Size(), turn_radius, v_max, a_max);


	return out_path;
}



void DynamicPointPaths::addTransforms(pathFcn fptr, const RSState & goal) {
	RSPath rsp;
	RSState rs;


	rs = goal;
	rsp = fptr(rs);
	if (rsp.is_valid) {
		all_paths.push_back(rsp);
	}
	//rs = goal;
	//rs.reverse();
	//rsp = fptr(rs);
	//rsp.reverse();
	//if (rsp.is_valid) {
	//	all_paths.push_back(rsp);
	//}
	rs = goal;
	rs.reflect();
	rsp = fptr(rs);
	rsp.reflect();
	if (rsp.is_valid) {
		all_paths.push_back(rsp);
	}
	//rs = goal;
	//rs.reverse();
	//rs.reflect();
	//rsp = fptr(rs);
	//rsp.reverse();
	//rsp.reflect();
	//if (rsp.is_valid) {
	//	all_paths.push_back(rsp);
	//}
}



float DynamicPointPaths::path_time(int idx) const {
	return all_paths[idx].time;
}

float DynamicPointPaths::path_time() const {
	return all_paths[path_index].time;
}












