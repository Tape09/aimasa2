// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "Path.h"
#include <utility>
#include <cmath>
#include <algorithm>

/**
*
*/
class AIMASA2_API DynamicPath : public Path {
public:
	DynamicPath(FVector pos0, FVector vel0, FVector pos1, FVector vel1, float v_max, float a_max);
	~DynamicPath();

	virtual State step(float delta_time);
	virtual State state_at(float time);



	struct Path1D {

		float t1;
		float t2;
		float t3;
		float p0;
		float p1;
		float p2;
		float p3;
		float v0;
		float v1;
		float v2;
		float v3;
		float a0;
	};



	Path1D one_dim_quadratic(float x0, float v0, float x1, float v1);
	Path1D slow_path(float x0, float v0, float x1, float v1, float time, float oldt1, float oldt2, float oldt3);
	Path1D slow_path2(float x0, float v0, float x1, float v1, float time, float oldt1, float oldt2, float oldt3);

	void max_vel_acc(float & max_vel, float & max_acc);
	//float max_acc();

	//void applyPath(Path1D p, int idx);
	FVector final_pos();
	FVector final_vel();

	FVector v_0;
	FVector v_1;

	FVector p_0;
	FVector p_1;

	Path1D path[2];
	Path1D px;
	Path1D py;

	float a_max;
	float v_max;

	float timex;
	float timey;

private:


};
