#pragma once

//System Includes
#include <stdlib.h>
#include <stdio.h>
#include <float.h>
#include <math.h>

//External Includes
#include "opencv2/highgui.hpp"

#define CMV_MAX_BUF 1024
#define MAX_POL_LENGTH 64

struct ocam_model
{
	double pol[MAX_POL_LENGTH];    // the polynomial coefficients: pol[0] + x"pol[1] + x^2*pol[2] + ... + x^(N-1)*pol[N-1]
	int length_pol;                // length of polynomial
	double invpol[MAX_POL_LENGTH]; // the coefficients of the inverse polynomial
	int length_invpol;             // length of inverse polynomial
	double xc;         // row coordinate of the center
	double yc;         // column coordinate of the center
	double c;          // affine parameter
	double d;          // affine parameter
	double e;          // affine parameter
	int width;         // image width
	int height;        // image height
};

inline int get_ocam_model(struct ocam_model* myocam_model, const char* filename)
{
	double* pol = myocam_model->pol;
	double* invpol = myocam_model->invpol;
	double* xc = &(myocam_model->xc);
	double* yc = &(myocam_model->yc);
	double* c = &(myocam_model->c);
	double* d = &(myocam_model->d);
	double* e = &(myocam_model->e);
	int* width = &(myocam_model->width);
	int* height = &(myocam_model->height);
	int* length_pol = &(myocam_model->length_pol);
	int* length_invpol = &(myocam_model->length_invpol);
	FILE* f;
	char buf[CMV_MAX_BUF];
	int i;
	int result;
	//Open file
	if (!(f = fopen(filename, "r")))
	{
		printf("File %s cannot be opened\n", filename);
		return -1;
	}

	//Read polynomial coefficients
	fgets(buf, CMV_MAX_BUF, f);
	result = fscanf(f, "\n");
	result = fscanf(f, "%d", length_pol);
	for (i = 0; i < *length_pol; i++)
	{
		fscanf(f, " %lf", &pol[i]);
	}

	//Read inverse polynomial coefficients
	result = fscanf(f, "\n");
	fgets(buf, CMV_MAX_BUF, f);
	result = fscanf(f, "\n");
	result = fscanf(f, "%d", length_invpol);
	for (i = 0; i < *length_invpol; i++)
	{
		fscanf(f, " %lf", &invpol[i]);
	}

	//Read center coordinates
	result = fscanf(f, "\n");
	fgets(buf, CMV_MAX_BUF, f);
	result = fscanf(f, "\n");
	result = fscanf(f, "%lf %lf\n", xc, yc);

	//Read affine coefficients
	fgets(buf, CMV_MAX_BUF, f);
	result = fscanf(f, "\n");
	result = fscanf(f, "%lf %lf %lf\n", c, d, e);

	//Read image size
	fgets(buf, CMV_MAX_BUF, f);
	result = fscanf(f, "\n");
	result = fscanf(f, "%d %d", height, width);

	fclose(f);
	return 0;
}

inline void cam2world(double point3D[3], double point2D[2], struct ocam_model* myocam_model)
{
	double* pol = myocam_model->pol;
	double xc = (myocam_model->xc);
	double yc = (myocam_model->yc);
	double c = (myocam_model->c);
	double d = (myocam_model->d);
	double e = (myocam_model->e);
	int length_pol = (myocam_model->length_pol);
	double invdet = 1 / (c - d * e); // 1/det(A), where A = [c,d;e,1] as in the Matlab file

	double xp = invdet * ((point2D[0] - xc) - d * (point2D[1] - yc));
	double yp = invdet * (-e * (point2D[0] - xc) + c * (point2D[1] - yc));

	double r = sqrt(xp * xp + yp * yp); //distance [pixels] of  the point from the image center
	double zp = pol[0];
	double r_i = 1;
	int i;

	for (i = 1; i < length_pol; i++)
	{
		r_i *= r;
		zp += r_i * pol[i];
	}

	//normalize to unit norm
	double invnorm = 1 / sqrt(xp * xp + yp * yp + zp * zp);

	point3D[0] = invnorm * xp;
	point3D[1] = invnorm * yp;
	point3D[2] = invnorm * zp;
}

inline void world2cam(double point2D[2], double point3D[3], struct ocam_model* myocam_model)
{
	double* invpol = myocam_model->invpol;
	double xc = (myocam_model->xc);
	double yc = (myocam_model->yc);
	double c = (myocam_model->c);
	double d = (myocam_model->d);
	double e = (myocam_model->e);
	int    width = (myocam_model->width);
	int    height = (myocam_model->height);
	int length_invpol = (myocam_model->length_invpol);
	double norm = sqrt(point3D[0] * point3D[0] + point3D[1] * point3D[1]);
	double theta = atan(point3D[2] / norm);
	double t, t_i;
	double rho, x, y;
	double invnorm;
	int i;

	if (norm != 0)
	{
		invnorm = 1 / norm;
		t = theta;
		rho = invpol[0];
		t_i = 1;

		for (i = 1; i < length_invpol; i++)
		{
			t_i *= t;
			rho += t_i * invpol[i];
		}

		x = point3D[0] * invnorm * rho;
		y = point3D[1] * invnorm * rho;

		point2D[0] = x * c + y * d + xc;
		point2D[1] = x * e + y + yc;
	}
	else
	{
		point2D[0] = xc;
		point2D[1] = yc;
	}
}

inline void create_perspective_undistortion_LUT(cv::Mat& mapx, cv::Mat& mapy, struct ocam_model* ocam_model, double sf)
{
	int i, j;
	int width = mapx.cols;
	int height = mapx.rows;    

	double Nxc = height / 2.0;
	double Nyc = width / 2.0;
	double Nz = -width / sf;
	double M[3];
	double m[2];

	for (i = 0; i < height; i++)
		for (j = 0; j < width; j++)
		{
			M[0] = (i - Nxc);
			M[1] = (j - Nyc);
			M[2] = Nz;
			world2cam(m, M, ocam_model);
			mapx.at<double>(i, j) = (double)m[1];
			mapy.at<double>(i, j) = (double)m[0];
		}
}


