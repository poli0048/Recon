#pragma once

//External Includes
#include "../../../../eigen/Eigen/SVD"
#include "../../../../eigen/Eigen/Geometry"

namespace {
	//Refine a valid solution with a Gauss-Newton Solver. The paper notes it rarely improves after two iterations.
	//The original implementation use 5 iterations. Parameters:
	// - L Valid lambdas (updated in place)
	// - a12 is the squared distance between X1 and X2
	// - a13 is the squared distance between X1 and X3
	// - a23 is the squared distance between X2 and X3
	// - b12 is the cosine of the angle between bearing vector 1 and bearing vector 2
	// - b13 is the cosine of the angle between bearing vector 1 and bearing vector 3
	// - b23 is the cosine of the angle between bearing vector 2 and bearing vector 3 
	void gauss_newton_refineL(Eigen::Vector3d& L, double a12, double a13, double a23, double b12, double b13, double b23) {
		for (int i = 0; i < 5; ++i) {
			double l1 = L(0);
			double l2 = L(1);
			double l3 = L(2);
			double r1 = l1 * l1 + l2 * l2 + b12 * l1 * l2 - a12;
			double r2 = l1 * l1 + l3 * l3 + b13 * l1 * l3 - a13;
			double r3 = l2 * l2 + l3 * l3 + b23 * l2 * l3 - a23;

			if (std::abs(r1) + std::abs(r2) + std::abs(r3) < 1e-10)
				break;

			double dr1dl1 = 2.0 * l1 + b12 * l2;
			double dr1dl2 = 2.0 * l2 + b12 * l1;

			double dr2dl1 = 2.0 * l1 + b13 * l3;
			double dr2dl3 = 2.0 * l3 + b13 * l1;

			double dr3dl2 = 2.0 * l2 + b23 * l3;
			double dr3dl3 = 2.0 * l3 + b23 * l2;

			Eigen::Vector3d r(r1, r2, r3);

			// or skip the inverse and make it explicit...
			{
				double v0 = dr1dl1;
				double v1 = dr1dl2;
				double v3 = dr2dl1;
				double v5 = dr2dl3;
				double v7 = dr3dl2;
				double v8 = dr3dl3;
				double det = 1.0 / (-v0 * v5 * v7 - v1 * v3 * v8);

				Eigen::Matrix3d Ji;
				Ji << -v5 * v7, -v1 * v8, v1* v5,
					-v3 * v8, v0* v8, -v0 * v5,
					v3* v7, -v0 * v7, -v1 * v3;
				Eigen::Vector3d L1 = Eigen::Vector3d(L) - det * (Ji * r); //Why is there an explicit copy of L here?
				//%l=l - g*H\G;%inv(H)*G
				//L=L - g*J\r;
				//% works because the size is ok!
				{
					double l1 = L1(0);
					double l2 = L1(1);
					double l3 = L1(2);
					double r11 = l1 * l1 + l2 * l2 + b12 * l1 * l2 - a12;
					double r12 = l1 * l1 + l3 * l3 + b13 * l1 * l3 - a13;
					double r13 = l2 * l2 + l3 * l3 + b23 * l2 * l3 - a23;
					if (std::abs(r11) + std::abs(r12) + std::abs(r13) > std::abs(r1) + std::abs(r2) + std::abs(r3))
						break;
					else
						L = L1;
				}
			}
		}
	}

	bool root2real(double b, double c, double& r1, double& r2) {
		double v = b * b - 4.0 * c;
		if (v < 0.0) {
			r1 = r2 = 0.5 * b;
			return false;
		}
		double y = std::sqrt(v);
		if (b < 0.0) {
			r1 = 0.5 * (-b + y);
			r2 = 0.5 * (-b - y);
		}
		else {
			r1 = 2.0 * c / (-b + y);
			r2 = 2.0 * c / (-b - y);
		}
		return true;
	}

	// This function finds a single root of a cubic polynomial. Parameters:
	// - b Coefficient of quadratic parameter
	// - c Coefficient of linear parameter
	// - d Coefficient of scalar parameter
	//
	// Returns: A single root of h(r) = r^3 + b*r^2 + c*r + d
	//
	// The returned root is as stable as possible in the sense that it has as high a
	// derivative as possible.  The solution is found by simple Newton-Raphson
	// iterations, and the trick is to choose the intial solution r0 in a clever
	// way.
	//
	// The intial solution is found by considering 5 cases:
	//
	// Cases I and II: h has no stationary points. In this case its derivative
	// is positive.  The inital solution to the NR-iteration is r0 here h has
	// minimal derivative.
	//
	// Case III, IV, and V: has two stationary points, t1 < t2.  In this case,
	// h has negative derivative between t1 and t2.  In these cases, we can make
	// a second order approximation of h around each of t1 and t2, and choose r0
	// as the leftmost or rightmost root of these approximations, depending on
	// whether two, one, or both of h(t1) and h(t2) are > 0.
	double cubick(double b, double c, double d) {
		// Choose an initial solution
		double r0;
		//Non-monotonic Case
		if (b * b >= 3.0 * c) {
			// h has two stationary points, compute them
			double v = std::sqrt(b * b - 3.0 * c);
			double t1 = (-b - v) / (3.0);

			// Check if h(t1) > 0, in this case make a 2-order approx of h around t1
			double k = ((t1 + b) * t1 + c) * t1 + d;

			if (k > 0.0) {
				// Find leftmost root of 0.5*(r0 -t1)^2*(6*t1+2*b) +  k = 0
				r0 = t1 - std::sqrt(-k / (3.0 * t1 + b));
			}
			else {
				double t2 = (-b + v) / 3.0;
				k = ((t2 + b) * t2 + c) * t2 + d;

				// Find rightmost root of 0.5 * (r0 - t2)^2 * (6 * t2 +2 * b) + k1 = 0
				r0 = t2 + std::sqrt(-k / (3.0 * t2 + b));
			}
		}
		else {
			r0 = -b / 3.0;
			if (std::abs(((3.0 * r0 + 2.0 * b) * r0 + c)) < 1e-4)
				r0 += 1;
		}

		// Do Newton-Raphson iterations. Note: as in the original implementation, the number of iterations is hard-coded.
		// According to the author, increasing the number of iterations may lead to a better solution (more robust)
		for (unsigned int cnt = 0; cnt < 50; ++cnt) {
			double fx = (((r0 + b) * r0 + c) * r0 + d);
			if ((cnt < 7 || std::abs(fx) > 1e-13)) {
				double fpx = ((3.0 * r0 + 2.0 * b) * r0 + c);
				r0 -= fx / fpx;
			}
			else
				break;
		}
		return r0;
	}

	//eigwithknown0: eigen decomposition of a matrix which has a 0 eigen value. Parameters:
	// - x the input matrix
	// - E eigenvectors (output)
	// - L eigenvalues (output)
	void eigwithknown0(Eigen::Matrix3d const& x, Eigen::Matrix3d& E, Eigen::Vector3d& L) {
		//One eigenvalue is known to be 0. Populate it now
		L(2) = 0.0;

		Eigen::Vector3d v3(x(3) * x(7) - x(6) * x(4),
			x(6) * x(1) - x(7) * x(0),
			x(4) * x(0) - x(3) * x(1));
		v3.normalize();

		//Compute the other two eigenvalues
		double x01_squared = x(0, 1) * x(0, 1);
		double b = -x(0, 0) - x(1, 1) - x(2, 2);
		double c = -x01_squared - x(0, 2) * x(0, 2) - x(1, 2) * x(1, 2) + x(0, 0) * (x(1, 1) + x(2, 2)) + x(1, 1) * x(2, 2);
		double e1, e2;
		root2real(b, c, e1, e2);
		if (std::abs(e1) < std::abs(e2))
			std::swap(e1, e2);
		L(0) = e1;
		L(1) = e2;

		double mx0011 = -x(0, 0) * x(1, 1);
		double prec_0 = x(0, 1) * x(1, 2) - x(0, 2) * x(1, 1);
		double prec_1 = x(0, 1) * x(0, 2) - x(0, 0) * x(1, 2);

		double e = e1;
		double tmp = 1.0 / (e * (x(0, 0) + x(1, 1)) + mx0011 - e * e + x01_squared);
		double a1 = -(e * x(0, 2) + prec_0) * tmp;
		double a2 = -(e * x(1, 2) + prec_1) * tmp;
		double rnorm = 1.0 / std::sqrt(a1 * a1 + a2 * a2 + 1.0);
		a1 *= rnorm;
		a2 *= rnorm;
		Eigen::Vector3d v1(a1, a2, rnorm);

		double tmp2 = 1.0 / (e2 * (x(0, 0) + x(1, 1)) + mx0011 - e2 * e2 + x01_squared);
		double a21 = -(e2 * x(0, 2) + prec_0) * tmp2;
		double a22 = -(e2 * x(1, 2) + prec_1) * tmp2;
		double rnorm2 = 1.0 / std::sqrt(a21 * a21 + a22 * a22 + 1.0);
		a21 *= rnorm2;
		a22 *= rnorm2;
		Eigen::Vector3d v2(a21, a22, rnorm2);

		// optionally remove axb from v1,v2
		// costly and makes a very small difference!
		// v1=(v1-v1.dot(v3)*v3);v1.normalize();
		// v2=(v2-v2.dot(v3)*v3);v2.normalize();
		// v2=(v2-v1.dot(v2)*v2);v2.normalize();
		E << v1(0), v2(0), v3(0),
			v1(1), v2(1), v3(1),
			v1(2), v2(2), v3(2);
	}

	void Orthogonalize_3By3(Eigen::Matrix3d& R) {
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3d U = svd.matrixU();
		Eigen::Matrix3d V = svd.matrixV();
		R = U * V.transpose();
	}

	void Orthogonalize_3By3_QuickAndDirty(Eigen::Matrix3d& R) {
		R.col(0).normalize();
		R.col(1).normalize();
		R.col(2) = R.col(0).cross(R.col(1));
		R.col(2).normalize();
		R.col(1) = R.col(2).cross(R.col(0));
	}
}

// ******************************************************************************************************************************
// **********************************************   Public Function Definitions   ***********************************************
// ******************************************************************************************************************************

//BearingVectors: A matrix where column i is the bearing vector to world point i in the frame of the camera
//WorldPoints: A matrix where column i is the location of a world point i (in world coordinates)
//PossiblePoses: [Output] A vector of up to 4 possible camera poses. Each pose is a tuple <t,R> where t represents
//               the position of the camera in world coordinates and R is R_Cam_World (rotates vector from camera frame to world frame)
//BearingsAreNormalized: If true, skip normalization of columns of BearingVectors because they are known to be normalized already
inline void LambdaTwistSolve(Eigen::Matrix3d const& BearingVectors,
	Eigen::Matrix3d const& WorldPoints,
	std::Evector<std::tuple<Eigen::Vector3d, Eigen::Matrix3d>>& PossiblePoses,
	bool BearingsAreNormalized)
{
	PossiblePoses.clear();
	PossiblePoses.reserve(4);

	// Extraction of 3D points vectors
	Eigen::Vector3d P1 = WorldPoints.col(0);
	Eigen::Vector3d P2 = WorldPoints.col(1);
	Eigen::Vector3d P3 = WorldPoints.col(2);

	// Extraction of feature vectors
	Eigen::Vector3d f1 = BearingVectors.col(0);
	Eigen::Vector3d f2 = BearingVectors.col(1);
	Eigen::Vector3d f3 = BearingVectors.col(2);

	if (!BearingsAreNormalized) {
		f1.normalize();
		f2.normalize();
		f3.normalize();
	}

	double b12 = -2.0 * (f1.dot(f2));
	double b13 = -2.0 * (f1.dot(f3));
	double b23 = -2.0 * (f2.dot(f3));

	Eigen::Vector3d d12 = P1 - P2;
	Eigen::Vector3d d13 = P1 - P3;
	Eigen::Vector3d d23 = P2 - P3;
	Eigen::Vector3d d12xd13(d12.cross(d13));

	double a12 = d12.squaredNorm();
	double a13 = d13.squaredNorm();
	double a23 = d23.squaredNorm();

	//a*g^3 + b*g^2 + c*g + d = 0
	double c31 = -0.5 * b13;
	double c23 = -0.5 * b23;
	double c12 = -0.5 * b12;
	double blob = (c12 * c23 * c31 - 1.0);

	double s31_squared = 1.0 - c31 * c31;
	double s23_squared = 1.0 - c23 * c23;
	double s12_squared = 1.0 - c12 * c12;

	double p3 = a13 * (a23 * s31_squared - a13 * s23_squared);
	double p2 = 2.0 * blob * a23 * a13 + a13 * (2.0 * a12 + a13) * s23_squared + a23 * (a23 - a12) * s31_squared;
	double p1 = a23 * (a13 - a23) * s12_squared - a12 * a12 * s23_squared - 2.0 * a12 * (blob * a23 + a13 * s23_squared);
	double p0 = a12 * (a12 * s23_squared - a23 * s12_squared);

	p3 = 1.0 / p3;
	p2 *= p3;
	p1 *= p3;
	p0 *= p3;

	// get sharpest real root of above...
	double g = cubick(p2, p1, p0);

	// gain 13 ns...
	double A00 = a23 * (1.0 - g);
	double A01 = (a23 * b12) * 0.5;
	double A02 = (a23 * b13 * g) * (-0.5);
	double A11 = a23 - a12 + a13 * g;
	double A12 = b23 * (a13 * g - a12) * 0.5;
	double A22 = g * (a13 - a23) - a12;

	Eigen::Matrix3d A;
	A << A00, A01, A02,
		A01, A11, A12,
		A02, A12, A22;

	// Get sorted eigenvalues and eigenvectors given that one should be zero...
	Eigen::Matrix3d V;
	Eigen::Vector3d L;
	eigwithknown0(A, V, L);

	double v = std::sqrt(std::max(0.0, -L(1) / L(0)));
	std::Evector<Eigen::Vector3d> Ls;
	Ls.reserve(4);

	// use the t=Vl with t2,st2,t3 and solve for t3 in t2
	{ //+v
		double s = v;

		double w2 = 1.0 / (s * V(0, 1) - V(0, 0));
		double w0 = (V(1, 0) - s * V(1, 1)) * w2;
		double w1 = (V(2, 0) - s * V(2, 1)) * w2;

		double a = 1.0 / ((a13 - a12) * w1 * w1 - a12 * b13 * w1 - a12);
		double b = (a13 * b12 * w1 - a12 * b13 * w0 - 2.0 * w0 * w1 * (a12 - a13)) * a;
		double c = ((a13 - a12) * w0 * w0 + a13 * b12 * w0 + a13) * a;

		if (b * b - 4.0 * c >= 0.0) {
			double tau1, tau2;
			root2real(b, c, tau1, tau2);
			if (tau1 > 0.0) {
				double tau = tau1;
				double d = a23 / (tau * (b23 + tau) + 1.0);
				if (d > 0.0) {
					double l2 = std::sqrt(d);
					double l3 = tau * l2;

					double l1 = w0 * l2 + w1 * l3;
					if (l1 >= 0.0)
						Ls.emplace_back(l1, l2, l3);
				}
			}
			if (tau2 > 0.0) {
				double tau = tau2;
				double d = a23 / (tau * (b23 + tau) + 1.0);
				if (d > 0.0) {
					double l2 = std::sqrt(d);
					double l3 = tau * l2;
					double l1 = w0 * l2 + w1 * l3;
					if (l1 >= 0.0)
						Ls.emplace_back(l1, l2, l3);
				}
			}
		}
	}

	{ //-v
		double s = -v;
		double w2 = 1.0 / (s * V(0, 1) - V(0, 0));
		double w0 = (V(1, 0) - s * V(1, 1)) * w2;
		double w1 = (V(2, 0) - s * V(2, 1)) * w2;

		double a = 1.0 / ((a13 - a12) * w1 * w1 - a12 * b13 * w1 - a12);
		double b = (a13 * b12 * w1 - a12 * b13 * w0 - 2.0 * w0 * w1 * (a12 - a13)) * a;
		double c = ((a13 - a12) * w0 * w0 + a13 * b12 * w0 + a13) * a;

		if (b * b - 4.0 * c >= 0) {
			double tau1, tau2;

			root2real(b, c, tau1, tau2);
			if (tau1 > 0) {
				double tau = tau1;
				double d = a23 / (tau * (b23 + tau) + 1.0);
				if (d > 0.0) {
					double l2 = std::sqrt(d);
					double l3 = tau * l2;

					double l1 = w0 * l2 + w1 * l3;
					if (l1 >= 0)
						Ls.emplace_back(l1, l2, l3);
				}
			}
			if (tau2 > 0) {
				double tau = tau2;
				double d = a23 / (tau * (b23 + tau) + 1.0);
				if (d > 0.0) {
					double l2 = std::sqrt(d);
					double l3 = tau * l2;

					double l1 = w0 * l2 + w1 * l3;
					if (l1 >= 0)
						Ls.emplace_back(l1, l2, l3);
				}
			}
		}
	}

	// Use Gauss-Newton refinement if desired (hard-coded to always be used)
	for (size_t i = 0U; i < Ls.size(); i++)
		gauss_newton_refineL(Ls[i], a12, a13, a23, b12, b13, b23);

	Eigen::Vector3d ry1, ry2, ry3;
	Eigen::Vector3d yd1;
	Eigen::Vector3d yd2;
	Eigen::Vector3d yd1xd2;
	Eigen::Matrix3d Xmat;
	Xmat << d12(0), d13(0), d12xd13(0),
		d12(1), d13(1), d12xd13(1),
		d12(2), d13(2), d12xd13(2);

	Xmat = Xmat.inverse().eval();

	//int viablePoses = 0;
	//int nonviablePoses = 0;
	for (size_t i = 0U; i < Ls.size(); i++) {
		// Compute the pose
		ry1 = f1 * Ls[i](0);
		ry2 = f2 * Ls[i](1);
		ry3 = f3 * Ls[i](2);

		yd1 = ry1 - ry2;
		yd2 = ry1 - ry3;
		yd1xd2 = yd1.cross(yd2);

		Eigen::Matrix3d Ymat;
		Ymat << yd1(0), yd2(0), yd1xd2(0),
			yd1(1), yd2(1), yd1xd2(1),
			yd1(2), yd2(2), yd1xd2(2);

		Eigen::Matrix3d Rs = Ymat * Xmat;   //R from paper (R_World_Cam)
		Eigen::Vector3d ts = ry1 - Rs * P1; //t from paper

		Eigen::Matrix3d R_World_Cam = Rs;
		Eigen::Matrix3d R_Cam_World = Rs.transpose();

		//Project R_Cam_World to manifold of orthogonal matrices. We have found that this isn't usually needed, but sometimes the
		//recovered rotation matrix has significant error in it. This reduces that error and ensures the returned matrix
		//is at least a rotation matrix (although technically it may still be possible to get a matrix with determinant -1 depending on the function used).
		//Orthogonalize_3By3(R_Cam_World);
		Orthogonalize_3By3_QuickAndDirty(R_Cam_World);

		Eigen::Vector3d CamCenter_World = -1.0 * R_Cam_World * ts;

		//It seems that all candidate pose solutions produced by LambdaTwist result in world points in front of the camera... that is, the vector
		//from the camera center to the scene point points in the same direction as the camera bearing vector, not the opposite direction. There is
		//no need for additional checking of this.
		PossiblePoses.push_back(std::make_tuple(CamCenter_World, R_Cam_World));

		//Note: It also seems that LambdaTwist gives solutions that respect the observations almost exactly, even in the presense of noise.
		//This seems to suggest that even if scene points and/or bearings are in error or are noisy, there tend to be exact solutions out there.
	}
	//std::cerr << "\r\n";
}
