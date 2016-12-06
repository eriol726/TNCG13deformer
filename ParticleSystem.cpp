#include "ParticleSystem.h"


ParticleSystem::ParticleSystem(std::vector<MPoint> initP) {


	for (auto i = initP.begin(); i != initP.end(); i++) {
		
		positions.push_back(*i);
		
	}


	
}



ParticleSystem::~ParticleSystem(){}




/**
MPoint ParticleSystem::getPositions(int idx) {
	return positions;
}*/

std::vector<MPoint> ParticleSystem::shapeMatch() {
	
	for (auto i = positions.begin(); i != positions.end(); i++) {
		if (i->y < 0) {
			i->y = 0.0;
		}
		
	} 

	return positions;
}