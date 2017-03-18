// Fill out your copyright notice in the Description page of Project Settings.

#include "aimasa2.h"
#include "Path.h"

Path::Path()
{
}

Path::~Path()
{
}

void Path::reset() {
	t_now = 0;
}

float Path::path_time() const {
	return time_taken;
}

bool Path::isValid() const {
	return valid;
}

bool Path::pathExists() const {
	return exists;
}

void print_log(State s) {
	print_log("pos: " + s.pos.ToString());
	print_log("vel: " + s.vel.ToString());
	print_log("acc: " + s.acc.ToString());
	print_log("___");
}

FVector Path::pos_0() const {
	return pos0;
}

FVector Path::pos_1() const {
	return pos1;
}

FVector Path::vel_0() const {
	return vel0;
}

FVector Path::vel_1() const {
	return vel1;
}

