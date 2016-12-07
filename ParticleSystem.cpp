#include "ParticleSystem.h"


ParticleSystem::ParticleSystem(std::vector<MPoint> initP, MVector velocity) {
	
	arma::fvec3 temp_v;

	temp_v[0] = velocity.x;
	temp_v[1] = velocity.y;
	temp_v[2] = velocity.z;

	for (auto i = initP.begin(); i != initP.end(); i++) {
		
		arma::fvec3 temp_x;
		
		temp_x[0] = (float)i->x;
		temp_x[1] = (float)i->y;
		temp_x[2] = (float)i->z;

	
		positions.push_back(*i);
		x.push_back(temp_x);
		x_0.push_back(temp_x);
		goal.push_back(temp_x);
		v.push_back(temp_v);

	}

	
	// initialCenterOfMass
	x_com_0 = computeCOM();
	
}


ParticleSystem::~ParticleSystem(){}

// computing t and t0
arma::fvec3 ParticleSystem::computeCOM() {
	// same value for all dimensions
	arma::fvec3 com;

	for (auto i = x.begin(); i != x.end(); i++) {
		com += *i;
	}
	
	float totalMass = x.size();
	float w = 1.0f;

	return (w*com / totalMass)  ;
}



MPoint ParticleSystem::getPositions(int idx) {

	MPoint pt;
	pt.x = x[idx][0];
	pt.y = x[idx][1];
	pt.z = x[idx][2];

	return pt;
}

std::vector<MPoint> ParticleSystem::shapeMatch(float dt) {
	
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

	arma::fvec3 x_com = computeCOM(); // centerOfMass;

								 // sida 18, mitten
	
	for (int i = 0; i < x.size(); ++i)
	{

		p(0, i) = x[i](0) - x_com(0);
		p(1, i) = x[i](1)- x_com(1);
		p(2, i) = x[i](2) - x_com(2);

		q(0, i) = x_0[i](0) - x_com_0(0);
		q(1, i) = x_0[i](1) - x_com_0(1);
		q(2, i) = x_0[i](2) - x_com_0(2);
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
	
	
	//ensuring that det(A) = 1
	A = A / pow(arma::det(A), 1 / 3);

	// Convert to glm 
	//glm::mat3 R_glm = armaToGlmMat(R, 3);
	//glm::mat3 A_glm = armaToGlmMat(A, 3);
	//mpointToGlmVec(x_0);
	
	

	for (int i = 0; i < x.size(); i++) {
		goal[i] = (beta*A + (1.0f - beta) * R) * (x_0[i] - x_com_0) + x_com;
	}


	float alpha;
	for (int i = 0; i < x.size(); i++) {
		v[i] = (goal[i] - x[i]) / dt;
		x[i] = goal[i] - x[i];

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
		
	}
		
	
	*/	

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

/*
std::vector<glm::vec3> mpointToGlmVec(std::vector<MPoint> p)
{
	glm::vec3 glmVec= glm::vec3(1.0);

	for (auto i = p.begin(); i != p.end(); i++) {
		*i->x;
		*i->x;
		*i->x;
	}
	glmVec[0] = (float)p.x;
	glmVec[1] = (float)p.y;
	glmVec[2] = (float)p.z;

	return glmVec;
}*/