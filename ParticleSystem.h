#include "glm/glm.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtx/transform.hpp"
#include <string>
#include <vector>
#include <maya/MPointArray.h>
#include <armadillo>

// The Ocean Spray in Your Face
struct tParticle
{
	tParticle *prev, *next; // LINK
	glm::vec3 pos; // CURRENT POSITION
	glm::vec3 prevPos; // PREVIOUS POSITION
	glm::vec3 dir; // CURRENT DIRECTION WITH SPEED
	int life; // HOW LONG IT WILL LAST

};

class ParticleSystem {
public:
	ParticleSystem(std::vector<MPoint> initPositions, MVector velocity);
	~ParticleSystem() ;


	double gravityMagnitude;
	double mass;
	arma::fvec3 gravityDirection;

	MPoint getPositions(int idx);
	std::vector<MPoint> shapeMatch(float dt);
	glm::mat3 armaToGlmMat(arma::fmat M, int size);
	glm::vec3 armaToGlmMat(arma::fvec v, int size);
	std::vector<glm::vec3> mpointToGlmVec(std::vector<MPoint> p);
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
	arma::fvec3 x_com_0;


};