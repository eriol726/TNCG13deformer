#include "ParticleSystem.h"


ParticleSystem::ParticleSystem(std::vector<MPoint> initP) {


	for (auto i = initP.begin(); i != initP.end(); i++) {
		
		positions.push_back(*i);
		x.push_back(*i);
		x_0.push_back(*i);

	}
	// initialCenterOfMass
	x_com_0 = computeCOM();
	
}


ParticleSystem::~ParticleSystem(){}

// computing t and t0
MPoint ParticleSystem::computeCOM() {
	MPoint com = MPoint(0, 0, 0);
	for (auto i = x.begin(); i != x.end(); i++) {
		com += *i;
	}
	
	float totalMass = x.size();
	float w = 1.0f;

	return (w*com / totalMass)  ;
}


/**
MPoint ParticleSystem::getPositions(int idx) {
	return positions;
}*/

std::vector<MPoint> ParticleSystem::shapeMatch() {
	
	/*
	for (auto i = positions.begin(); i != positions.end(); i++) {
		if (i->y < 0) {
			i->y = 0.0;
		}
		
	} */

	// Allocate
	arma::fmat q = arma::fmat(3, x.size());
	arma::fmat p = arma::fmat(3, x.size());
	arma::fmat Apq;
	arma::fmat Aqq;
	arma::fmat A;
	arma::fmat S;
	arma::fmat R;
	arma::fmat U;
	arma::fmat V;
	arma::fvec s;

	MPoint x_com = computeCOM(); // centerOfMass;

								 // sida 18, mitten
	
	for (int i = 0; i < x.size(); ++i)
	{

		p(0, i) = x[i].x - x_com.x;
		p(1, i) = x[i].y - x_com.y;
		p(2, i) = x[i].z - x_com.z;

		q(0, i) = x_0[i].x - x_com_0.x;
		q(1, i) = x_0[i].y - x_com_0.y;
		q(2, i) = x_0[i].z - x_com_0.z;
	}
	float m = 1.0f;
	Apq = p * q.t();
	Aqq = (m*q*q.t()).i();

	A = Apq*Aqq;
	//S = A.t()*A;
	//S = sqrt(S);

	// Singular value decomposition should be polar
	
	arma::svd(U, s, V, Apq);
	R = V * U.t();


	float beta = 0.5f;

	
	float Rmat[3][3];
	float Amat[3][3];

	for (int r = 0; r<3; r++)
	{
		for (int c = 0; c<3; c++)
		{
			Rmat[r][c] = R(r,c);
			Amat[r][c] = A(r, c);
		}
	}

	// Convert to glm matrix
	glm::mat3 R_glm = armaToGlmMat(R, 3);
	glm::mat3 A_glm = armaToGlmMat(A, 3);
	
	for (int i = 0; i < x.size(); i++) {
		//goal[i] = (beta*A_glm + (1.0f - beta) * R_glm) * (x_0[i] - x_com_0) + x_com;

	}

	//calculate finla rotation R
	/*
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			for (int i = 0; i < x.size(); i++) {
				goal[i].x = (beta*Amat[r][c] + (1.0f - beta) * Rmat[r][c]) * (x_0[i].x - x_com_0.x) + x_com.x;
				goal[i].y = (beta*Amat[r][c] + (1.0f - beta) * Rmat[r][c]) * (x_0[i].y - x_com_0.y) + x_com.y;
				goal[i].z = (beta*Amat[r][c] + (1.0f - beta) * Rmat[r][c]) * (x_0[i].z - x_com_0.z) + x_com.z;
			}
		}
		
	}*/
		
	
		

	return positions;
}

glm::vec3 armaToGlmVec(arma::fvec v, int size)
{
	glm::vec3 tempVec = glm::vec3(0.0f);

	tempVec.x = v(0);
	tempVec.y = v(1);
	tempVec.z = v(2);
	
	return tempVec;
}

glm::mat3 armaToGlmMat(arma::fmat M, int size)
{
	glm::mat3 glmMatrix = glm::mat3(1.0);

	for (int r = 0; r < size; r++) {
		for (int c = 0; c < size; c++) {
			glmMatrix[r][c] = M(r, c);
		}
	}
		
	return glmMatrix;
}