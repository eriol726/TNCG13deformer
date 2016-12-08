#include "ParticleSystem.h"


ParticleSystem::ParticleSystem(std::vector<MPoint> initP, MVector velocity) {

	/*
	x.resize(initP.size());
	x_0.resize(initP.size());
	v.resize(initP.size());
	force.resize(initP.size());
	goal.resize(initP.size());
	positions.resize(initP.size());*/

	
	arma::fvec3 temp_v;

	temp_v[0] = velocity.x;
	temp_v[1] = velocity.y;
	temp_v[2] = velocity.z;

	arma::fvec3 temp_x;

	//temp_x(0) = 0.f;
	//temp_x(1) = 0.f;
	//temp_x(2) = 0.f;

	arma::fvec3 temp_force;
	temp_force.zeros();

	for (auto i = initP.begin(); i != initP.end(); i++) {
		
		
		
		temp_x(0) = (float)i->x;
		temp_x(1) = (float)i->y;
		temp_x(2) = (float)i->z;

		positions.push_back(*i);
		x.push_back(temp_x);
		x_0.push_back(temp_x);
		goal.push_back(temp_x);
		v.push_back(temp_v);
		force.push_back(temp_force);
		

	}

	
	// initialCenterOfMass
	x_com_0 = computeCOM();
	
}


ParticleSystem::~ParticleSystem(){}

// computing t and t0
arma::fvec3 ParticleSystem::computeCOM() {
	// same value for all dimensions
	arma::fvec3 com;

	com(0) = 0.f;
	com(1) = 0.f;
	com(2) = 0.f;

	int inx = 0;
	for (auto i = x.begin(); i != x.end(); i++) {
		com += *i;
		
		
	}
	
	float totalMass = x.size();
	float w = 1.0f;

	cout.rdbuf(cerr.rdbuf()); //hack to get error messages out in Maya 2016.5


	cout << "com_0: " << (1.0f / static_cast<float>(x.size())) * com << endl;


	fflush(stdout);
	fflush(stderr);

	//return (w*com / totalMass)  ;
	return (1.0f / static_cast<float>(x.size())) * com;
}



MPoint ParticleSystem::getPositions(int idx) {

	MPoint pt = MPoint(0,0,0);
	pt.x = x[idx](0);
	pt.y = x[idx](1);
	pt.z = x[idx](2);



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

	cout.rdbuf(cerr.rdbuf()); //hack to get error messages out in Maya 2016.5

	cout << "-------------------------------------------------------------------- "  << endl;



	fflush(stdout);
	fflush(stderr);

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
	
	
	//ensuring that det(A) = 1
	A = A/ pow(arma::det(A), 1 / 3);


	
	//calculate finla rotation R
	R = (beta*A + (1.0f - beta) * R);

	for (int i = 0; i < x.size(); i++) {
		goal[i] = R * (x_0[i] - x_com_0) + x_com;

		
	}


	float alpha = 0.5f;
	for (int i = 0; i < x.size(); i++) {
		v[i] += (goal[i] - x[i]) / dt;
		x[i] += goal[i] - x[i];

	}

	


	return positions;
}


void ParticleSystem::applyGravity(float dt) {
	arma::fvec3 zeroVec;

	zeroVec.zeros();


	arma::fvec3 dir;

	dir(0) = gravityDirection.x;
	dir(1) = gravityDirection.y;
	dir(2) =  gravityDirection.z;


	for (int i = 0; i < force.size(); i++) {
		cout.rdbuf(cerr.rdbuf()); //hack to get error messages out in Maya 2016.5

		force[i] = dir * gravityMagnitude * mass;

		float elasticity = 0.2f;
		float dynamicFriction = 0.2f;
		// adding force from floor
		if (x[i](1) <= 0) {
			arma::fvec3 normal;
			normal(0) = 0.f;
			normal(1) = 1.f;
			normal(2) = 0.f;

			arma::fvec3 vDiff = (v[i]) - zeroVec; // Floor is static

			arma::fvec3 vDiff1 = normal * arma::dot(normal, vDiff); // vDiff composant in normal direction
			arma::fvec3 vDiff2 = vDiff - vDiff1; // vDiff composant orthogonal to normal direction

			arma::fvec3 collisionImpulse = -(elasticity + 1) * vDiff1 * mass;
			arma::fvec3 frictionImpulse = -dynamicFriction * vDiff2 * mass;

			force[i] += (collisionImpulse + frictionImpulse) / dt;
			x[i](1) = 0.01;
		}
		// Add collision impulse and friction
		
	}
}

void ParticleSystem::updatePositions(float dt) {
	for (int i = 0; i < x.size(); i++)
	{

		x[i] += v[i] * dt;

	}

}

void ParticleSystem::updateVelocities(float dt)
{
	// Euler integration
	arma::fvec3 zeroVec;
	zeroVec.zeros();


	for (int i = 0; i < v.size(); ++i)
	{
		
		v[i] += force[i] / mass * dt;
		force[i] = zeroVec; // Reset all forces
	}
}
