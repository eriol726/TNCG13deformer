#ifndef BLENDNODE_H
#define BLENDNODE_H

#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MGlobal.h>
#include <maya/MItGeometry.h>
#include <maya/MMatrix.h>
#include <maya/MPointArray.h>
#include <maya/MStatus.h>
#include <maya/MTime.h>
#include <maya/MFloatVectorArray.h>
#include <maya/MFnMesh.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MPxDeformerNode.h>
#include <maya/MAnimControl.h>
#include <maya/MFnEnumAttribute.h>
#include "ParticleSystem.h"


class DeformerNode : public MPxDeformerNode {
public:
	DeformerNode() {};

	virtual MStatus deform(MDataBlock& data, MItGeometry& itGeo,
		const MMatrix &localToWorldMatrix, unsigned int mIndex);
	static void* creator();
	static MStatus initialize();

	// Unique id
	static MTypeId id;

	// Object attributes from rigid body
	static MObject aCurrentTime;
	static MObject aInitVelocity;
	static MObject aMass;
	static MObject aDynamicFriction;
	static MObject aElasticity;
	static MObject aDeformation;
	static MObject aDamping;
	static MObject aStiffnes;
	static MObject aDeformMethod;

private:
	static MTime tPrevious;
	static ParticleSystem* particleSystem;
};
#endif