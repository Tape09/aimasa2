// Fill out your copyright notice in the Description page of Project Settings.

#include "aimasa2.h"
#include "MyMath.h"

float wrapAngle(float angle) {
	angle = mod2pi(angle);
	return angle >= pii ? angle-2*pii : angle;
}

float vecAngle(const FVector & fv) {
	return wrapAngle(std::atan2(fv.Y,fv.X));
}

float mmod(float m, float n) {
	return m - n * floor(m / n);
}

float rad2deg(const float & rad) {
	return rad * 180 / pii;
}

//float mod2pi(float angle) {
//	while(angle < 0) angle += twopi;
//	while(angle >= twopi) angle -= twopi;
//	return angle;
//}

float mod2pi(float angle) {
	//while (angle < 0) angle += twopi;
	//while (angle >= twopi) angle -= twopi;
	return mmod(angle,twopi);
}

void rotateVector(FVector & fv, float theta) {
	fv = fv.RotateAngleAxis(rad2deg(theta), FVector(0, 0, 1));
}

void print_log(FString msg) {
	UE_LOG(LogTemp, Warning, TEXT("%s"), *msg);
}

void print_log(FVector msg) {
	print_log(msg.ToString());
}

void print_log(float msg) {
	print_log(FString::SanitizeFloat(msg));
}

AccelerationInfo accelerate_between(float x0, float v0, float x3, float v3, float v_max, float a) {
	bool valid = false;
	
	float t1;
	float t2;
	float t3;
	float v1;
	//float v2;
	//float x1;
	//float x2;

	float best_time = 999999;

	float best_t1 = -1;
	float best_t2 = -1;
	float best_t3 = -1;
	float best_v1 = -1;
	//float best_v2;
	//float best_x1;
	//float best_x2;

	// case 1
	float det = a*x0*-2.0 + a*x3*2.0 + v0*v0 + v3*v3;
	if(det >= 0) {

		t1 = -(v0 + sqrt(2.0)*sqrt(det)*(1.0 / 2.0)) / a;
		t2 = 0;
		t3 = (-v3 - sqrt(2.0)*sqrt(det)*(1.0 / 2.0)) / a;

		v1 = a*t1 + v0;

		if (((t1 + t2 + t3) < best_time) && (v1 == v_max) && t1 >= 0 && t2 >= 0 && t3 >= 0) {
			best_t1 = t1;
			best_t2 = t2;
			best_t3 = t3;
			best_v1 = v1;
			best_time = t1+t2+t3;
			valid = true;
		}
	


		t1 = -(v0 - sqrt(2.0)*sqrt(det)*(1.0 / 2.0)) / a;
		t2 = 0;
		t3 = (-v3 + sqrt(2.0)*sqrt(det)*(1.0 / 2.0)) / a;

		v1 = a*t1 + v0;

		if (((t1 + t2 + t3) < best_time) && (v1 == v_max) && t1 >= 0 && t2 >= 0 && t3 >= 0) {
			best_t1 = t1;
			best_t2 = t2;
			best_t3 = t3;
			best_v1 = v1;
			best_time = t1 + t2 + t3;
			valid = true;
		}
	}


	// case 2

	t1 = -(v0 - v_max) / a;
	t2 = (-a*x0 + a*x3 + (v0*v0)*(1.0 / 2.0) + (v3*v3)*(1.0 / 2.0) - v_max*v_max) / (a*v_max);
	t3 = -(v3 - v_max) / a;

	v1 = a*t1 + v0;

	if (((t1 + t2 + t3) < best_time) && (v1 == v_max) && t1 >= 0 && t2 >= 0 && t3 >= 0) {
		best_t1 = t1;
		best_t2 = t2;
		best_t3 = t3;
		best_v1 = v1;
		best_time = t1 + t2 + t3;
		valid = true;
	}


	AccelerationInfo ai;

	if (valid) {
		ai.a = a;

		ai.t1 = best_t1;
		ai.t2 = best_t2;
		ai.t3 = best_t3;

		ai.v0 = v0;
		ai.v1 = ai.a * ai.t1 + v0;
		ai.v2 = ai.v1;
		ai.v3 = -ai.a * ai.t3 + ai.v2;	
		
		ai.p0 = x0;
		ai.p1 = a * ai.t1 * ai.t1 / 2 + ai.v0 * ai.t1 + ai.p0;		
		ai.p2 = ai.v1 * ai.t2 + ai.p1;
		ai.p3 = -ai.a * ai.t3 * ai.t3 / 2 + ai.v2 * ai.t3 + ai.p2;

		
		ai.isValid = true;
	} else {
		ai.isValid = false;
	}

	return ai;
}




//float getAngle(FVector a, FVector b) {
//	float dot = a.X*b.X + a.Y*b.Y;
//	float det = a.X*b.Y - a.Y*b.X;
//	float angle = atan2(det, dot);
//	return std::abs(angle);
//}

float getAngle(FVector a, FVector b) {
	float dot = a.X*b.X + a.Y*b.Y;
	a.Z = 0;
	b.Z = 0;
	float angle = acos(dot/(a.Size() * b.Size()));
	return std::abs(angle);
}

bool isInPolygon(FVector point, const TArray<FVector> & polyBounds) {
	//returns true if point in polygon
	float angleSum = 0;
	for (int i = 0; i < polyBounds.Num() - 1; i++) {
		angleSum += getAngle(polyBounds[i] - point, polyBounds[i + 1] - point);
	}
	angleSum += getAngle(polyBounds[0] - point, polyBounds[polyBounds.Num() - 1] - point);

	return (std::abs(angleSum - twopi) < 0.0001);
}

bool isInAnyPolygon(FVector tempPoint, const TArray<TArray<FVector>> & polygons) {
	bool inPolygon = false;
	for (int j = 0; j < polygons.Num() - 1; j++) {
		inPolygon = isInPolygon(tempPoint, polygons[j]);
		if (inPolygon)
			break;
	}
	return inPolygon;
}


FVector randVel(float max_v) {
	float vel = FMath::FRandRange(0, max_v);
	float theta = FMath::FRandRange(0, twopi);
	return vel * FVector(cos(theta), sin(theta), 0);
}

void print(FString msg, float time, FColor color) {
	GEngine->AddOnScreenDebugMessage(-1, time, color, msg);
}



//void file_log(std::string s) {
//	std::ofstream myfile;
//	myfile.open("C:\\temp\\log.txt", std::ofstream::app | std::ios::out);
//	myfile << s << "\n";
//	myfile.close();
//}
void file_log(FString s) {
	std::ofstream myfile;
	myfile.open("C:\\temp\\log.txt", std::ofstream::app | std::ios::out);
	myfile << TCHAR_TO_UTF8(*s) << "\n";
	myfile.close();	
}

void file_log(int s) {
	std::ofstream myfile;
	myfile.open("C:\\temp\\log.txt", std::ofstream::app | std::ios::out);
	myfile << s << "\n";
	myfile.close();
}

void file_log(float s) {
	std::ofstream myfile;
	myfile.open("C:\\temp\\log.txt", std::ofstream::app | std::ios::out);
	myfile << s << "\n";
	myfile.close();
}

void file_log(FVector s) {
	file_log(s.ToString());
}

void reset_log_file() {	
	remove("C:\\temp\\log.txt");
}










