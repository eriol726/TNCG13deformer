#include "ParticleSystem.h"


ParticleSystem::ParticleSystem(std::vector<MPoint> initP, MVector velocity) {


	// initialize dynamic arrays
	arma::fvec3 temp_v;
	temp_v.zeros();

	temp_v[0] = velocity.x;
	temp_v[1] = velocity.y;
	temp_v[2] = velocity.z;

	arma::fvec3 temp_x;
	temp_x.zeros();

	arma::fvec3 temp_force;
	temp_force.zeros();

	for (auto i = initP.begin(); i != initP.end(); i++) {
		
		temp_x(0) = (float)i->x;
		temp_x(1) = (float)i->y;
		temp_x(2) = (float)i->z;

		x.push_back(temp_x);
		x_0.push_back(temp_x);
		goal.push_back(temp_x);
		v.push_back(temp_v);
		force.push_back(temp_force);
		mg.push_back(temp_force);
	}

	// center of mass of initial shape t_0
	x_com_0 = computeCOM();
}

ParticleSystem::~ParticleSystem(){}

// computing t and t0
arma::fvec3 ParticleSystem::computeCOM() {
	// same value for all dimensions
	arma::fvec3 com;
	com.zeros();

	for (int i = 0; i < x.size(); i++)
	{
		com += particleMass*x[i];
	}
		
	float totalMass = x.size();
	
	return (1.0f / totalMass) * com;
}


MPoint ParticleSystem::getPositions(int idx) {

	MPoint pt = MPoint(0,0,0);
	pt.x = x[idx](0);
	pt.y = x[idx](1);
	pt.z = x[idx](2);

	return pt;
}

arma::fmat ParticleSystem::computeR(float dt) {
	// Allocate
	arma::fmat q = arma::fmat(3, x.size());
	arma::fmat p = arma::fmat(3, x.size());
	arma::fmat Apq;
	arma::fmat Aqq;
	arma::fmat S;
	arma::fmat R;
	arma::fmat U;
	arma::fmat V;
	arma::fvec s;

	// center of mass of actual shape, t
	arma::fvec3 x_com = computeCOM();

	//define the relative locations,  sida 18, mitten
	for (int i = 0; i < x.size(); ++i)
	{
		p(0, i) = x[i](0) - x_com(0);
		p(1, i) = x[i](1) - x_com(1);
		p(2, i) = x[i](2) - x_com(2);

		q(0, i) = x_0[i](0) - x_com_0(0);
		q(1, i) = x_0[i](1) - x_com_0(1);
		q(2, i) = x_0[i](2) - x_com_0(2);
	}

	// rotation part
	Apq = particleMass * p * q.t();

	// scaling part
	Aqq = (particleMass*q*q.t()).i();

	// linear transformation matrix
	A = Apq*Aqq;

	//ensuring that det(A) = 1, To make sure that volume is conserved
	A = A / pow(arma::det(A), 1 / 3.0f);

	// Find rotation matrix by Singular value decomposition, should be polar decomposition
	arma::svd(U, s, V, Apq);
	R = V * U.t();

	return R;
}

void ParticleSystem::shapeMatchLinear(float dt) {

	arma::fmat R = computeR( dt);
	arma::fvec3 x_com = computeCOM();
	arma::fmat T_linear;
	
	//Aply linera matrix to the rotation matrix
	T_linear = (beta*A + (1.0f - beta) * R);

	
	// computing the goal positions
	for (int i = 0; i < x.size(); i++) {
		goal[i] = T_linear * (x_0[i] - x_com_0) + x_com;
	}
	
	// updating the final positions an velocity
	for (int i = 0; i < x.size(); i++) {
		v[i] += stiffnes*bounciness*(goal[i] - x[i]) / dt;
		x[i] += stiffnes*(goal[i] - x[i]);
	}
}
 

void ParticleSystem::shapeMatchQuadratic(float dt) {

	// Allocate
	arma::fmat q_tilde = arma::fmat(9, x.size());
	arma::fmat p_tiled = arma::fmat(3, x.size());
	arma::fmat A_tiled = arma::fmat(3, 3);
	arma::fmat A = arma::fmat(3, 3);
	arma::fmat Q = arma::fmat(3, 3);
	arma::fmat M = arma::fmat(3, 3);
	arma::fmat Apq_tiled;
	arma::fmat Aqq_tiled;
	arma::fmat R_tiled ;
	arma::fmat zeros3x3 = arma::fmat(3, 3);
	arma::fmat T_quadric;
	arma::fmat AQM;
	arma::fmat R = computeR(dt);

	arma::fvec3 x_com = computeCOM();
	
	for (int i = 0; i < x.size(); ++i)
	{
		p_tiled(0, i) = x[i](0) - x_com(0);
		p_tiled(1, i) = x[i](1) - x_com(1);
		p_tiled(2, i) = x[i](2) - x_com(2);

		q_tilde(0, i) = x_0[i](0) - x_com_0(0);
		q_tilde(1, i) = x_0[i](1) - x_com_0(1);
		q_tilde(2, i) = x_0[i](2) - x_com_0(2);
		q_tilde(3, i) = q_tilde(0, i)*q_tilde(0, i);
		q_tilde(4, i) = q_tilde(1, i)*q_tilde(1, i);
		q_tilde(5, i) = q_tilde(2, i)*q_tilde(2, i);
		q_tilde(6, i) = q_tilde(0, i)*q_tilde(1, i);
		q_tilde(7, i) = q_tilde(1, i)*q_tilde(2, i);
		q_tilde(8, i) = q_tilde(2, i)*q_tilde(0, i);
	}
	
	Apq_tiled = particleMass*p_tiled*q_tilde.t();

	Aqq_tiled = (particleMass*q_tilde*q_tilde.t()).i();

	A_tiled = Apq_tiled*Aqq_tiled;

	// creating R_tiled [R 0 0]
	zeros3x3.zeros();

	R_tiled.insert_cols(0, R);
	R_tiled.insert_cols(3, zeros3x3);
	R_tiled.insert_cols(6, zeros3x3);
	
	T_quadric = (beta*A_tiled + (1.0f - beta) * R_tiled);
	
	// splitting A_tiled into sub matrices A Q M 
	A = T_quadric.submat(0, 0, 2, 2);
	Q = T_quadric.submat(0, 3, 2, 5);
	M = T_quadric.submat(0, 6, 2, 8);

	// volume preservation and rescale
	A = A / pow(arma::det(A), 1/3.0f);
	
	// copy the sub matrx back
	AQM.insert_cols(0, A);
	AQM.insert_cols(3, Q);
	AQM.insert_cols(6, M);

	R_tiled = AQM*q_tilde;

	for (int i = 0; i < x.size(); i++) {
		goal[i](0) = R_tiled.row(0)(i)+ x_com(0);
		goal[i](1) = R_tiled.row(1)(i)+ x_com(1);
		goal[i](2) = R_tiled.row(2)(i)+ x_com(2);
	}
 
	// updating the final positions an velocity
	for (int i = 0; i < x.size(); i++) {

		v[i] += stiffnes*bounciness*(goal[i] - x[i]) / dt;
		x[i] += stiffnes*(goal[i] - x[i]);
	}
}

void ParticleSystem::applyGravity(float dt) {
	
	arma::fvec3 planeVelocity;
	planeVelocity.zeros();
	//just setting plane mass to sommething 
	float planeMass = mass*3;

	arma::fvec3 dir;

	dir(0) = gravityDirection.x;
	dir(1) = gravityDirection.y;
	dir(2) =  gravityDirection.z;


	for (int i = 0; i < force.size(); i++) {

		mg[i] = dir * gravityMagnitude * mass;
		force[i] = mg[i];
		
		// adding forces from floor 
		if (x[i](1) <= 0) {
			arma::fvec3 normal;
			normal(0) = 0.f;
			normal(1) = 1.f;
			normal(2) = 0.f;

			arma::fvec3 relativeVelocity = v[i] - planeVelocity; 

			// composant in normal direction
			arma::fvec3 contactNormal = normal * arma::dot(relativeVelocity, normal); 

			// Collision response http://gafferongames.com/virtual-go/collision-response-and-coulomb-friction/
			arma::fvec3 linearMomentum = relativeVelocity*mass;
			arma::fvec3 impulseMagnitude = -(elasticity + 1) * contactNormal / (1/mass /*+1/planeMass*/);

			// Coulomb Friction
			arma::fvec3 frictionImpulse = -dynamicFriction * contactNormal * mass;

			// derivative of impluse gives a force
			force[i] = mg[i]+(impulseMagnitude + frictionImpulse) / dt;
			x[i](1) = 0.001f;
		}
	}
}

void ParticleSystem::updatePositions(float dt) {

	// integrating velocity gives distance
	for (int i = 0; i < x.size(); i++)
	{
		x[i] += v[i] * dt;
	}

}

void ParticleSystem::updateVelocities(float dt)
{
	// Euler integration
	arma::fvec3 zeroVec;
	arma::fvec3 acceleration;
	zeroVec.zeros();
	acceleration.zeros();
	//  integrating acceleration gives velocity
	for (int i = 0; i < v.size(); ++i)
	{
		//F=ma
		acceleration = force[i] / mass;
		v[i] += acceleration * dt;
		force[i] = zeroVec;
	}
}
