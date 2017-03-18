// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "MyMath.h"

/**
 Parent class for all controllers. use step(dt) to get the next point to move to. reset starts at te beginning again. use pos(time) to get the point at time t.
 Maybe add step for velocity/acceleration if needed.
 */

struct State {
	
	State(FVector pos_ = FVector(0,0,0), FVector vel_ = FVector(0, 0, 0), FVector acc_ = FVector(0, 0, 0)) : pos(pos_), vel(vel_), acc(acc_) {}

	FVector pos;
	FVector vel;
	FVector acc;
 };

class AIMASA2_API Path
{
public:
	Path();


	~Path();

	virtual State step(float delta_time) = 0;
	virtual State state_at(float time) = 0;
	virtual float path_time() const;
	virtual void reset();
	virtual bool isValid() const;
	virtual bool pathExists() const;

	bool valid = true;
	bool exists = true;

	FVector pos_0() const;
	FVector pos_1() const;
	FVector vel_0() const;
	FVector vel_1() const;

protected:
	float t_now = 0;
	float time_taken;

	

	FVector pos0;
	FVector pos1;

	FVector vel0;
	FVector vel1;
};


void print_log(State s);