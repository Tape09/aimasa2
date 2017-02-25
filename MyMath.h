// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include <cmath>
#include <vector>
#include <string>
#include <fstream>
#include <cstdio>

const float pii = 4*FGenericPlatformMath::Atan(1);
const float twopi = 8 * FGenericPlatformMath::Atan(1);
float wrapAngle(float angle);
float vecAngle(const FVector & fv);
float rad2deg(const float & rad);
float mod2pi(float angle);


float mmod(float m, float n);

void rotateVector(FVector & fv, float theta);

void print_log(FString msg);
void print_log(FVector msg);
void print_log(float msg);

void print(FString msg, float time = 999, FColor color = FColor::Red);

void reset_log_file();

//void file_log(std::string s);
void file_log(FString s);
void file_log(int s);
void file_log(float s);
void file_log(FVector s);


float getAngle(FVector a, FVector b);

struct PosVel {
	float pos;
	float vel;
};

struct AccelerationInfo {
	bool isValid;
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
	float a;

	PosVel pos_vel_at(float t) {
		//std::vector<float> times = {0, t1, t1+t2, t1+t2+t3};
		float rel_t;
		PosVel pv;

		//print_log(FString::SanitizeFloat(t));

		if (t <= t1) {
			rel_t = t;
			pv.vel = a*rel_t + v0;
			pv.pos = a*rel_t*rel_t/2 + v0*rel_t + p0;
		} else if (t <= t1 + t2) {
			rel_t = t-t1;
			pv.vel = v1;
			pv.pos = v1*rel_t + p1;
		} else if (t <= t1 + t2 + t3) {
			rel_t = t-t1-t2;
			pv.vel = -a*rel_t + v2;
			pv.pos = -a*rel_t*rel_t / 2 + v2*rel_t + p2;
		} else {
			pv.vel = v3;
			pv.pos = p3;
		}


		return pv;
	}

};


AccelerationInfo accelerate_between(float x0, float v0, float x3, float v3, float v_max, float a);

bool isInPolygon(FVector point, const TArray<FVector> & polyBounds);
bool isInAnyPolygon(FVector tempPoint, const TArray<TArray<FVector>> & polygons);

FVector randVel(float max_v);