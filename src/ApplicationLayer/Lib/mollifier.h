#pragma once

double mollifier_d(double x, double a, double b, double c);
float  mollifier_f(float x, float a, float b, float c);

double intMollifier_d(double c, double dt=0.001);
float  intMollifier_f(float c, float dt=0.0005);
float  intMollifier_f_tbl(float c);
