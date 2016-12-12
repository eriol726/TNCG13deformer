#include "glm/glm.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtx/transform.hpp"
#include <string>
#include <vector>
#include <maya/MPointArray.h>
#include <armadillo>


class ParticleSystem {
public:
	ParticleSystem(std::vector<MPoint> initPositions, MVector velocity);
	~ParticleSystem() ;


	const double gravityMagnitude = 9.82;
	double mass;
	double elasticity;
	double dynamicFriction;
	double beta;
	double damping;
	double stiffnes;
	// linear transformation matrix
	arma::fmat A;
	

	glm::vec3 gravityDirection = glm::vec3(0.0, -1.0, 0.0);

	MPoint getPositions(int idx);
	arma::fmat computeR(float dt);
	void shapeMatchLinear(float dt);
	void shapeMatchQuadratic(float dt);

	arma::fvec3 computeCOM();
	void applyGravity(float dt);
	void updatePositions(float dt);
	void updateVelocities(float dt);

private:

	std::vector<MPoint> positions;
	std::vector<arma::fvec3> x;
	std::vector<arma::fvec3> x_0;
	std::vector<arma::fvec3> v;
	std::vector<arma::fvec3> goal;
	std::vector<arma::fvec3> force;
	std::vector<arma::fvec3> mg;
	arma::fvec3 x_com_0;


};