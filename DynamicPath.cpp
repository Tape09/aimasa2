// Fill out your copyright notice in the Description page of Project Settings.

#include "aimasa2.h"
#include "DynamicPath.h"

DynamicPath::DynamicPath(FVector pos0_, FVector vel0_, FVector pos1_, FVector vel1_, float v_max_, float a_max_) {
	pos0 = pos0_;
	pos1 = pos1_;
	vel0 = vel0_;
	vel1 = vel1_;


	v_max = v_max_;
	a_max = a_max_;

	p_0 = pos0;
	p_1 = pos1;

	v_0 = vel0;
	v_1 = vel1;

	path[0] = one_dim_quadratic(p_0.X, v_0.X, p_1.X, v_1.X);
	path[1] = one_dim_quadratic(p_0.Y, v_0.Y, p_1.Y, v_1.Y);



	timex = path[0].t1 + path[0].t2 + path[0].t3;
	timey = path[1].t1 + path[1].t2 + path[1].t3;


	if (timex > timey) {
		path[1] = slow_path(p_0.Y, v_0.Y, p_1.Y, v_1.Y, timex, path[1].t1, path[1].t2, path[1].t3);
	} else if (timey > timex) {
		path[0] = slow_path(p_0.X, v_0.X, p_1.X, v_1.X, timey, path[0].t1, path[0].t2, path[0].t3);
	}

	timex = path[0].t1 + path[0].t2 + path[0].t3;
	timey = path[1].t1 + path[1].t2 + path[1].t3;

	if (timex == 0 || timey == 0) {
		exists = false;
	} else {
		exists = true;
	}

	time_taken = path[0].t1 + path[0].t2 + path[0].t3;



}

DynamicPath::~DynamicPath() {}


State DynamicPath::step(float delta_time) {
	t_now += delta_time;

	return state_at(t_now);
}

State DynamicPath::state_at(float ta) {
	State s;
	float t;
	for (int i = 0; i < 2; ++i) {
		if (ta <= path[i].t1) { // first interval
			t = ta;
			s.acc[i] = path[i].a0;
			s.vel[i] = path[i].a0 * t + path[i].v0;
			s.pos[i] = path[i].a0 * t * t / 2 + path[i].v0 * t + path[i].p0;

		} else if (ta <= path[i].t1 + path[i].t2) { // second interval
			t = ta - path[i].t1;
			s.acc[i] = 0;
			s.vel[i] = path[i].v1;
			s.pos[i] = path[i].v1 * t + path[i].p1;
		} else if (ta <= path[i].t1 + path[i].t2 + path[i].t3) { // third interval
			t = ta - path[i].t1 - path[i].t2;

			s.acc[i] = -path[i].a0;
			s.vel[i] = -path[i].a0 * t + path[i].v2;
			s.pos[i] = -path[i].a0 * t * t / 2 + path[i].v2 * t + path[i].p2;
		}
	}

	return s;
}


FVector DynamicPath::final_pos() {
	FVector fv(0, 0, pos0.Z);
	fv.X = path[0].p3;
	fv.Y = path[1].p3;
	return fv;
}

FVector DynamicPath::final_vel() {
	FVector fv(0, 0, pos0.Z);
	fv.X = path[0].v3;
	fv.Y = path[1].v3;

	return fv;
}


// WORKS
DynamicPath::Path1D DynamicPath::one_dim_quadratic(float x0, float v0, float x1, float v1) {

	float a0;
	float a1;

	float vmx;

	float a;
	float b;
	float c;

	float t1 = 0;
	float t2 = 0;
	float t3 = 0;

	bool found_best = false;
	float best_time = 999999;
	float best_a0 = 99999;
	float best_t1 = 0;
	float best_t2 = 0;
	float best_t3 = 0;
	float best_vmx = 0;

	float v_mid;

	float possible_a0[] = { -a_max, a_max };
	float possible_vm[] = { -v_max,v_max };

	// ALWAYS ACCELERATE
	for (int i = 0; i < 2; ++i) {
		a0 = possible_a0[i];
		a1 = -a0;

		a = a0 / 2 - (a0*a0) / (2 * a1);
		b = v0 - (a0 / a1) * v0;
		c = (v1*v1) / (2 * a1) - (v0*v0) / (2 * a1) + x0 - x1;

		float det = b*b - 4 * a*c;
		if (det < 0) continue;

		t1 = (-b + sqrt(det)) / (2 * a);
		t3 = (v1 - v0 - a0 * t1) / a1;

		v_mid = abs(a0 * t1 + v0);

		if (t1 >= 0 && t3 >= 0 && v_mid <= v_max) {
			if (t1 + t3 < best_time) {
				best_time = t1 + t3;
				best_a0 = a0;
				best_t1 = t1;
				best_t3 = t3;
				best_vmx = v_mid;
				found_best = true;
			}
		}

		t1 = (-b - sqrt(det)) / (2 * a);
		t3 = (v1 - v0 - a0 * t1) / a1;
		v_mid = abs(a0 * t1 + v0);

		if (t1 >= 0 && t3 >= 0 && v_mid <= v_max) {
			if (t1 + t3 < best_time) {
				best_time = t1 + t3;
				best_a0 = a0;
				best_t1 = t1;
				best_t3 = t3;
				best_vmx = v_mid;
				found_best = true;
			}
		}
	}



	// WITH CONSTANT VEL;
	if (!found_best) {
		for (int i = 0; i < 2; ++i) {
			a0 = possible_a0[i];
			a1 = -a0;
			vmx = possible_vm[i];

			t1 = (vmx - v0) / a0;
			t3 = (v1 - vmx) / a1;

			t2 = (x1 - x0) / vmx - t3 - (v0 / vmx)*t1 - (a1*t3*t3) / (2 * vmx) - (a0*t1*t1) / (2 * vmx);

			if (t1 >= 0 && t2 >= 0 && t3 >= 0) {
				if (t1 + t2 + t3 < best_time) {
					best_time = t1 + t3;
					best_a0 = a0;
					best_t1 = t1;
					best_t2 = t2;
					best_t3 = t3;
					best_vmx = vmx;
				}
			}
		}
	}




	Path1D p;

	p.v0 = v0;
	p.p0 = x0;
	p.a0 = best_a0;
	p.t1 = best_t1;
	p.t2 = best_t2;
	p.t3 = best_t3;
	p.v1 = p.a0 * p.t1 + v0;
	p.p1 = p.a0 * p.t1 * p.t1 / 2 + v0 * p.t1 + x0;
	p.v2 = p.v1;
	p.p2 = p.v1 * p.t2 + p.p1;
	p.v3 = -p.a0 * p.t3 + p.v2;
	p.p3 = -p.a0 * p.t3 * p.t3 / 2 + p.v2 * p.t3 + p.p2;

	return p;



}


// WORKS?
DynamicPath::Path1D DynamicPath::slow_path(float x0, float v0, float x3, float v3, float time, float oldt1, float oldt2, float oldt3) {
	Path1D p;
	float a;


	float t1 = 0;
	float t2 = 0;
	float t3 = 0;

	bool found_best = false;
	float best_time = 999999;
	float best_a0 = 99999;
	float best_t1 = 0;
	float best_t2 = 0;
	float best_t3 = 0;
	float best_vmax = 0;

	float v_mid;

	float possible_a0[] = { -a_max, a_max };


	for (int i = 0; i < 2; ++i) {
		a = possible_a0[i];


		float det = 4 * a*x0 - 4 * a*x3 + 2 * v0*v3 + time *time * a *a - v0 *v0 - v3 * v3 + 2 * time*a*v0 + 2 * time*a*v3;

		if (det < 0) continue;

		t1 = (v3 / 2 - v0 / 2 + (time*a) / 2 + sqrt(det) / 2) / a;
		t2 = (v0 - v3 + a*time) / a - (v0 - v3 + sqrt(det) + a*time) / a;
		t3 = (v0 - v3 + sqrt(det) + a*time) / (2 * a);



		v_mid = abs(a * t1 + v0);

		if (t1 >= 0 && t2 >= 0 && t3 >= 0 && v_mid <= v_max) {
			if (t1 + t2 + t3 < best_time) {
				best_time = t1 + t2 + t3;
				best_a0 = a;
				best_t1 = t1;
				best_t2 = t2;
				best_t3 = t3;
				best_vmax = v_mid;
				found_best = true;
			}
		}

	}



	p.v0 = v0;
	p.p0 = x0;
	p.a0 = best_a0;
	p.t1 = best_t1;
	p.t2 = best_t2;
	p.t3 = best_t3;
	p.v1 = p.a0 * p.t1 + v0;
	p.p1 = p.a0 * p.t1 * p.t1 / 2 + v0 * p.t1 + x0;
	p.v2 = p.v1;
	p.p2 = p.v1 * p.t2 + p.p1;
	p.v3 = -p.a0 * p.t3 + p.v2;
	p.p3 = -p.a0 * p.t3 * p.t3 / 2 + p.v2 * p.t3 + p.p2;



	float det = time * time * v0 * v0 + time * time * v3 * v3 + 2 * time*v0*x0 - 2 * time*v0*x3 + 2 * time*v3*x0 - 2 * time*v3*x3 + 2 * x0 * x0 - 4 * x0*x3 + 2 * x3 * x3;

	if (det >= 0) {

		t1 = time - (2 * x0 - 2 * x3 + 2 * time*v0 + sqrt(2)*sqrt(det)) / (2 * (v0 - v3));
		t2 = 0;
		t3 = (2 * x0 - 2 * x3 + 2 * time*v0 + sqrt(2) *sqrt(det)) / (2 * (v0 - v3));
		a = (2 * x0 - 2 * x3 + 2 * time*v0 + sqrt(2)*sqrt(det)) / (time *time) - (4 * x0 - 4 * x3 + 3 * time*v0 + time*v3) / (time *time);


		v_mid = abs(a * t1 + v0);

		if (t1 >= 0 && t2 >= 0 && t3 >= 0 && v_mid <= v_max) {
			if (t1 + t2 + t3 < best_time) {
				best_time = t1 + t2 + t3;
				best_a0 = a;
				best_t1 = t1;
				best_t2 = t2;
				best_t3 = t3;
				best_vmax = v_mid;
				found_best = true;
			}
		}

		t1 = time - (2 * x0 - 2 * x3 + 2 * time*v0 - sqrt(2)*sqrt(det)) / (2 * (v0 - v3));
		t2 = 0;
		t3 = (2 * x0 - 2 * x3 + 2 * time*v0 - sqrt(2) *sqrt(det)) / (2 * (v0 - v3));
		a = (2 * x0 - 2 * x3 + 2 * time*v0 - sqrt(2)*sqrt(det)) / (time *time) - (4 * x0 - 4 * x3 + 3 * time*v0 + time*v3) / (time *time);

		v_mid = abs(a * t1 + v0);

		if (t1 >= 0 && t2 >= 0 && t3 >= 0 && v_mid <= v_max) {
			if (t1 + t2 + t3 < best_time) {
				best_time = t1 + t2 + t3;
				best_a0 = a;
				best_t1 = t1;
				best_t2 = t2;
				best_t3 = t3;
				best_vmax = v_mid;
				found_best = true;
			}
		}


		p.v0 = v0;
		p.p0 = x0;
		p.a0 = best_a0;
		p.t1 = best_t1;
		p.t2 = best_t2;
		p.t3 = best_t3;
		p.v1 = p.a0 * p.t1 + v0;
		p.p1 = p.a0 * p.t1 * p.t1 / 2 + v0 * p.t1 + x0;
		p.v2 = p.v1;
		p.p2 = p.v1 * p.t2 + p.p1;
		p.v3 = -p.a0 * p.t3 + p.v2;
		p.p3 = -p.a0 * p.t3 * p.t3 / 2 + p.v2 * p.t3 + p.p2;
	}

	///////////

	det = (-t2 *t2 * v0 *v0 + 2 * t2 *t2 * v0*v3 - t2 *t2 * v3 *v3 + 2 * time *time * v0 *v0 + 2 * time *time * v3 *v3 + 4 * time*v0*x0 - 4 * time*v0*x3 + 4 * time*v3*x0 - 4 * time*v3*x3 + 4 * x0 *x0 - 8 * x0*x3 + 4 * x3 *x3);

	if (det >= 0) {

		t1 = time - t2 - (2 * x0 - 2 * x3 - t2*v0 + t2*v3 + 2 * time*v0 + sqrt(det)) / (2 * (v0 - v3));
		t2 = oldt2;
		t3 = (2 * x0 - 2 * x3 - t2*v0 + t2*v3 + 2 * time*v0 + sqrt(det)) / (2 * (v0 - v3));
		a = (4 * x0 - 4 * x3 - t2*v0 + t2*v3 + 3 * time*v0 + time*v3) / (t2 * t2 - time *time) - (2 * x0 - 2 * x3 - t2*v0 + t2*v3 + 2 * time*v0 + sqrt(det)) / (t2 *t2 - time *time);

		v_mid = abs(a * t1 + v0);

		if (t1 >= 0 && t2 >= 0 && t3 >= 0 && v_mid <= v_max) {
			if (t1 + t2 + t3 < best_time) {
				best_time = t1 + t2 + t3;
				best_a0 = a;
				best_t1 = t1;
				best_t2 = t2;
				best_t3 = t3;
				best_vmax = v_mid;
				found_best = true;
			}
		}

		t1 = time - t2 - (2 * x0 - 2 * x3 - t2*v0 + t2*v3 + 2 * time*v0 - sqrt(det)) / (2 * (v0 - v3));
		t2 = oldt2;
		t3 = (2 * x0 - 2 * x3 - t2*v0 + t2*v3 + 2 * time*v0 - sqrt(det)) / (2 * (v0 - v3));
		a = (4 * x0 - 4 * x3 - t2*v0 + t2*v3 + 3 * time*v0 + time*v3) / (t2 * t2 - time *time) - (2 * x0 - 2 * x3 - t2*v0 + t2*v3 + 2 * time*v0 - sqrt(det)) / (t2 *t2 - time *time);
		v_mid = abs(a * t1 + v0);

		if (t1 >= 0 && t2 >= 0 && t3 >= 0 && v_mid <= v_max) {
			if (t1 + t2 + t3 < best_time) {
				best_time = t1 + t2 + t3;
				best_a0 = a;
				best_t1 = t1;
				best_t2 = t2;
				best_t3 = t3;
				best_vmax = v_mid;
				found_best = true;
			}
		}

		p.v0 = v0;
		p.p0 = x0;
		p.a0 = best_a0;
		p.t1 = best_t1;
		p.t2 = best_t2;
		p.t3 = best_t3;
		p.v1 = p.a0 * p.t1 + v0;
		p.p1 = p.a0 * p.t1 * p.t1 / 2 + v0 * p.t1 + x0;
		p.v2 = p.v1;
		p.p2 = p.v1 * p.t2 + p.p1;
		p.v3 = -p.a0 * p.t3 + p.v2;
		p.p3 = -p.a0 * p.t3 * p.t3 / 2 + p.v2 * p.t3 + p.p2;
	}
	//////

	t1 = -(2 * (x0 - x3 + time*v3)) / (v0 - v3);
	t2 = (2 * x0 - 2 * x3 + time*v0 + time*v3) / (v0 - v3);
	t3 = 0;
	a = (v0 - v3) * (v0 - v3) / (2 * (x0 - x3 + time*v3));

	v_mid = abs(a * t1 + v0);

	if (t1 >= 0 && t2 >= 0 && t3 >= 0 && v_mid <= v_max) {
		if (t1 + t2 + t3 < best_time) {
			best_time = t1 + t2 + t3;
			best_a0 = a;
			best_t1 = t1;
			best_t2 = t2;
			best_t3 = t3;
			best_vmax = v_mid;
			found_best = true;
		}
	}

	p.v0 = v0;
	p.p0 = x0;
	p.a0 = best_a0;
	p.t1 = best_t1;
	p.t2 = best_t2;
	p.t3 = best_t3;
	p.v1 = p.a0 * p.t1 + v0;
	p.p1 = p.a0 * p.t1 * p.t1 / 2 + v0 * p.t1 + x0;
	p.v2 = p.v1;
	p.p2 = p.v1 * p.t2 + p.p1;
	p.v3 = -p.a0 * p.t3 + p.v2;
	p.p3 = -p.a0 * p.t3 * p.t3 / 2 + p.v2 * p.t3 + p.p2;


	return p;
}


DynamicPath::Path1D DynamicPath::slow_path2(float x0, float v0, float x3, float v3, float time, float oldt1, float oldt2, float oldt3) {

	Path1D p;
	float a;


	float t1 = 0;
	float t2 = 0;
	float t3 = 0;

	bool found_best = false;
	float best_time = 999999;
	float best_a0 = 99999;
	float best_t1 = 0;
	float best_t2 = 0;
	float best_t3 = 0;
	float best_vmax = 0;

	float v_mid;



	float det = time * time * v0 * v0 + time * time * v3 * v3 + 2 * time*v0*x0 - 2 * time*v0*x3 + 2 * time*v3*x0 - 2 * time*v3*x3 + 2 * x0 * x0 - 4 * x0*x3 + 2 * x3 * x3;

	if (det >= 0) {

		t1 = time - (2 * x0 - 2 * x3 + 2 * time*v0 + sqrt(2)*sqrt(det)) / (2 * (v0 - v3));
		t2 = 0;
		t3 = (2 * x0 - 2 * x3 + 2 * time*v0 + sqrt(2) *sqrt(det)) / (2 * (v0 - v3));
		a = (2 * x0 - 2 * x3 + 2 * time*v0 + sqrt(2)*sqrt(det)) / (time *time) - (4 * x0 - 4 * x3 + 3 * time*v0 + time*v3) / (time *time);


		v_mid = abs(a * t1 + v0);

		if (t1 >= 0 && t2 >= 0 && t3 >= 0 && v_mid <= v_max) {
			if (t1 + t2 + t3 < best_time) {
				best_time = t1 + t2 + t3;
				best_a0 = a;
				best_t1 = t1;
				best_t2 = t2;
				best_t3 = t3;
				best_vmax = v_mid;
				found_best = true;
			}
		}

		t1 = time - (2 * x0 - 2 * x3 + 2 * time*v0 - sqrt(2)*sqrt(det)) / (2 * (v0 - v3));
		t2 = 0;
		t3 = (2 * x0 - 2 * x3 + 2 * time*v0 - sqrt(2) *sqrt(det)) / (2 * (v0 - v3));
		a = (2 * x0 - 2 * x3 + 2 * time*v0 - sqrt(2)*sqrt(det)) / (time *time) - (4 * x0 - 4 * x3 + 3 * time*v0 + time*v3) / (time *time);

		v_mid = abs(a * t1 + v0);

		if (t1 >= 0 && t2 >= 0 && t3 >= 0 && v_mid <= v_max) {
			if (t1 + t2 + t3 < best_time) {
				best_time = t1 + t2 + t3;
				best_a0 = a;
				best_t1 = t1;
				best_t2 = t2;
				best_t3 = t3;
				best_vmax = v_mid;
				found_best = true;
			}
		}


		p.v0 = v0;
		p.p0 = x0;
		p.a0 = best_a0;
		p.t1 = best_t1;
		p.t2 = best_t2;
		p.t3 = best_t3;
		p.v1 = p.a0 * p.t1 + v0;
		p.p1 = p.a0 * p.t1 * p.t1 / 2 + v0 * p.t1 + x0;
		p.v2 = p.v1;
		p.p2 = p.v1 * p.t2 + p.p1;
		p.v3 = -p.a0 * p.t3 + p.v2;
		p.p3 = -p.a0 * p.t3 * p.t3 / 2 + p.v2 * p.t3 + p.p2;
	}

	///////////

	det = (-t2 *t2 * v0 *v0 + 2 * t2 *t2 * v0*v3 - t2 *t2 * v3 *v3 + 2 * time *time * v0 *v0 + 2 * time *time * v3 *v3 + 4 * time*v0*x0 - 4 * time*v0*x3 + 4 * time*v3*x0 - 4 * time*v3*x3 + 4 * x0 *x0 - 8 * x0*x3 + 4 * x3 *x3);

	if (det >= 0) {

		t1 = time - t2 - (2 * x0 - 2 * x3 - t2*v0 + t2*v3 + 2 * time*v0 + sqrt(det)) / (2 * (v0 - v3));
		t2 = oldt2;
		t3 = (2 * x0 - 2 * x3 - t2*v0 + t2*v3 + 2 * time*v0 + sqrt(det)) / (2 * (v0 - v3));
		a = (4 * x0 - 4 * x3 - t2*v0 + t2*v3 + 3 * time*v0 + time*v3) / (t2 * t2 - time *time) - (2 * x0 - 2 * x3 - t2*v0 + t2*v3 + 2 * time*v0 + sqrt(det)) / (t2 *t2 - time *time);

		v_mid = abs(a * t1 + v0);

		if (t1 >= 0 && t2 >= 0 && t3 >= 0 && v_mid <= v_max) {
			if (t1 + t2 + t3 < best_time) {
				best_time = t1 + t2 + t3;
				best_a0 = a;
				best_t1 = t1;
				best_t2 = t2;
				best_t3 = t3;
				best_vmax = v_mid;
				found_best = true;
			}
		}

		t1 = time - t2 - (2 * x0 - 2 * x3 - t2*v0 + t2*v3 + 2 * time*v0 - sqrt(det)) / (2 * (v0 - v3));
		t2 = oldt2;
		t3 = (2 * x0 - 2 * x3 - t2*v0 + t2*v3 + 2 * time*v0 - sqrt(det)) / (2 * (v0 - v3));
		a = (4 * x0 - 4 * x3 - t2*v0 + t2*v3 + 3 * time*v0 + time*v3) / (t2 * t2 - time *time) - (2 * x0 - 2 * x3 - t2*v0 + t2*v3 + 2 * time*v0 - sqrt(det)) / (t2 *t2 - time *time);
		v_mid = abs(a * t1 + v0);

		if (t1 >= 0 && t2 >= 0 && t3 >= 0 && v_mid <= v_max) {
			if (t1 + t2 + t3 < best_time) {
				best_time = t1 + t2 + t3;
				best_a0 = a;
				best_t1 = t1;
				best_t2 = t2;
				best_t3 = t3;
				best_vmax = v_mid;
				found_best = true;
			}
		}

		p.v0 = v0;
		p.p0 = x0;
		p.a0 = best_a0;
		p.t1 = best_t1;
		p.t2 = best_t2;
		p.t3 = best_t3;
		p.v1 = p.a0 * p.t1 + v0;
		p.p1 = p.a0 * p.t1 * p.t1 / 2 + v0 * p.t1 + x0;
		p.v2 = p.v1;
		p.p2 = p.v1 * p.t2 + p.p1;
		p.v3 = -p.a0 * p.t3 + p.v2;
		p.p3 = -p.a0 * p.t3 * p.t3 / 2 + p.v2 * p.t3 + p.p2;
	}
	//////

	t1 = -(2 * (x0 - x3 + time*v3)) / (v0 - v3);
	t2 = (2 * x0 - 2 * x3 + time*v0 + time*v3) / (v0 - v3);
	t3 = 0;
	a = (v0 - v3) * (v0 - v3) / (2 * (x0 - x3 + time*v3));

	v_mid = abs(a * t1 + v0);

	if (t1 >= 0 && t2 >= 0 && t3 >= 0 && v_mid <= v_max) {
		if (t1 + t2 + t3 < best_time) {
			best_time = t1 + t2 + t3;
			best_a0 = a;
			best_t1 = t1;
			best_t2 = t2;
			best_t3 = t3;
			best_vmax = v_mid;
			found_best = true;
		}
	}

	p.v0 = v0;
	p.p0 = x0;
	p.a0 = best_a0;
	p.t1 = best_t1;
	p.t2 = best_t2;
	p.t3 = best_t3;
	p.v1 = p.a0 * p.t1 + v0;
	p.p1 = p.a0 * p.t1 * p.t1 / 2 + v0 * p.t1 + x0;
	p.v2 = p.v1;
	p.p2 = p.v1 * p.t2 + p.p1;
	p.v3 = -p.a0 * p.t3 + p.v2;
	p.p3 = -p.a0 * p.t3 * p.t3 / 2 + p.v2 * p.t3 + p.p2;


	return p;
}



void DynamicPath::max_vel_acc(float & max_vel, float & max_acc) {
	max_vel = 0;
	max_acc = 0;


	State s;
	float vel = 0;
	float acc = 0;

	float xtimes[] = { path[0].t1, path[0].t2, path[0].t3 };
	float ytimes[] = { path[1].t1, path[1].t2, path[1].t3 };
	for (int i = 1; i < 3; ++i) {
		xtimes[i] += xtimes[i - 1];
		ytimes[i] += ytimes[i - 1];
	}

	for (int i = 0; i < 3; ++i) {
		s = state_at(xtimes[i]);
		max_vel = std::max(s.vel.SizeSquared(), max_vel);
		max_acc = std::max(s.acc.SizeSquared(), max_acc);

		s = state_at(ytimes[i]);

		max_vel = std::max(s.vel.SizeSquared(), max_vel);
		max_acc = std::max(s.acc.SizeSquared(), max_acc);
	}

}













