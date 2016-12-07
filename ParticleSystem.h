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
	ParticleSystem(std::vector<MPoint> initPositions);
	~ParticleSystem() ;

	std::vector<MPoint> getPositions();
	std::vector<MPoint> shapeMatch();
	glm::mat3 armaToGlmMat(arma::fmat M, int size);
	glm::vec3 armaToGlmMat(arma::fvec v, int size);
	MPoint computeCOM();
private:
	glm::vec3 velocity;
	glm::vec3 gravity;
	std::vector<MPoint> positions;
	std::vector<MPoint> x;
	std::vector<MPoint> x_0;
	std::vector<MPoint> goal;
	MPoint x_com_0;


};