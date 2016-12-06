#include "glm/glm.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtx/transform.hpp"
#include <string>
#include <vector>
#include <maya/MPointArray.h>


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
private:
	glm::vec3 velocity;
	glm::vec3 gravity;
	std::vector<MPoint> positions;
};