#include "DeformerNode.h"
#include <maya/MFnPlugin.h>
#include <iostream>
#include <sstream>
#include <maya/MIOStream.h>


MTypeId DeformerNode::id(0x00000002);
MObject DeformerNode::aCurrentTime;
MObject DeformerNode::aInitVelocity;
MObject DeformerNode::aMass;
MObject DeformerNode::aDynamicFriction;
MObject DeformerNode::aElasticity;
MObject DeformerNode::aDeformation;
MObject DeformerNode::aBounciness;
MObject DeformerNode::aStiffnes;
MObject DeformerNode::aDeformMethod;

MTime DeformerNode::tPrevious;
ParticleSystem* DeformerNode::particleSystem;



void* DeformerNode::creator() { return new DeformerNode; }

MStatus DeformerNode::deform(MDataBlock& data, MItGeometry& itGeo,
	const MMatrix &localToWorldMatrix, unsigned int mIndex) {
	
	MStatus status;
	MMatrix localToWorldMatrixInv = localToWorldMatrix.inverse();

	MTime currentTime = MAnimControl::currentTime();
	int currentFrame = (int)currentTime.value();

	MTime tNow = data.inputValue(aCurrentTime).asTime();
	
	float env = data.inputValue(envelope).asFloat();


	if (currentFrame == 1 )
	{
		cout.rdbuf(cerr.rdbuf()); //hack to get error messages out in Maya 2016.5

		cout << "load" << endl;

		fflush(stdout);
		fflush(stderr);
		
		tPrevious = data.inputValue(aCurrentTime).asTime();
		std::vector<MPoint> initPositions;
		
		// load the mesh vertices into particles 
		for (; !itGeo.isDone(); itGeo.next()) {

			MPoint pt = itGeo.position()*localToWorldMatrix;
			initPositions.push_back(pt); 
		}
		
		MVector initVelocity = data.inputValue(aInitVelocity).asVector();
		particleSystem = new ParticleSystem(initPositions, initVelocity);

	}
	else {
		// User inputs
		particleSystem->mass = data.inputValue(aMass).asDouble();
		particleSystem->elasticity = data.inputValue(aElasticity).asDouble();
		particleSystem->dynamicFriction = data.inputValue(aDynamicFriction).asDouble();
		particleSystem->beta = data.inputValue(aDeformation).asDouble();
		particleSystem->bounciness = data.inputValue(aBounciness).asDouble();
		particleSystem->stiffnes = data.inputValue(aStiffnes).asDouble();
		bool deformMethod  = data.inputValue(aDeformMethod).asBool();

		
		tNow = data.inputValue(aCurrentTime).asTime();
		MTime timeDiff = tNow - tPrevious;
		int timeDiffInt = (int)timeDiff.value();
		tPrevious = tNow;

		// Since we want to update with t+h, we upade with timeDiff
		int timeStep = 2;
		float fps = 24.0f;
		float dt = 1 / (fps  * timeStep * abs(timeDiffInt))  ;

		
		for (int i = 0; i < timeStep * abs(timeDiffInt); ++i)
		{
				
			particleSystem->applyGravity(dt);
			particleSystem->updateVelocities(dt);
			particleSystem->updatePositions(dt);

			if (deformMethod) {

				particleSystem->shapeMatchQuadratic(dt);
			}
			else {

				particleSystem->shapeMatchLinear(dt);
			}
		}
	
		for (; !itGeo.isDone(); itGeo.next()) {

			int idx = itGeo.index();

			MPoint new_pos = particleSystem->getPositions(idx)*localToWorldMatrixInv;

			// Set the new output point
			itGeo.setPosition(new_pos);
		}
	}

	return MS::kSuccess;
}

MStatus DeformerNode::initialize() {

	//Setup attributes
	MFnTypedAttribute tAttr;
	MFnNumericAttribute nAttr;
	MFnUnitAttribute uAttr;
	MFnEnumAttribute eAttr;

	// Time
	aCurrentTime = uAttr.create("currentTime", "tm", MFnUnitAttribute::kTime, 0.0);
	MTime currentFrame = MAnimControl::currentTime();
	//24 frames per second
	uAttr.setDefault(currentFrame.as(MTime::kFilm));
	uAttr.setChannelBox(true);
	uAttr.setDefault(1);


	aMass = nAttr.create("Mass", "ms", MFnNumericData::kDouble, 0.0);
	nAttr.setDefault(1.0);
	nAttr.setMin(0.0);
	nAttr.setMax(10.0);
	nAttr.setChannelBox(true);


	aInitVelocity = nAttr.create("initVelocity", "iv", MFnNumericData::k3Double, 0.0);
	nAttr.setDefault(1.0);
	nAttr.setMin(0.0);
	nAttr.setMax(10.0);
	nAttr.setChannelBox(true);

	aElasticity = nAttr.create("elasticity", "el", MFnNumericData::kDouble, 0.0);
	nAttr.setDefault(0.5);
	nAttr.setMin(0.0);
	nAttr.setMax(1.0);
	nAttr.setChannelBox(true);

	aDynamicFriction = nAttr.create("dynamicFriction", "df", MFnNumericData::kDouble, 0.0);
	nAttr.setDefault(0.5);
	nAttr.setMin(0.0);
	nAttr.setMax(1.0);
	nAttr.setChannelBox(true);

	aDeformation = nAttr.create("deformation", "de", MFnNumericData::kDouble, 0.0);
	nAttr.setDefault(0.5);
	nAttr.setMin(0.0);
	nAttr.setMax(1.0);
	nAttr.setChannelBox(true);

	aBounciness = nAttr.create("Waving", "da", MFnNumericData::kDouble, 0.0);
	nAttr.setDefault(0.5);
	nAttr.setMin(0.0);
	nAttr.setMax(1.0);
	nAttr.setChannelBox(true);

	aStiffnes = nAttr.create("stiffnes", "st", MFnNumericData::kDouble, 0.0);
	nAttr.setDefault(0.5);
	nAttr.setMin(0.0);
	nAttr.setMax(1.0);
	nAttr.setChannelBox(true);

	//aDeformMethod = eAttr.create("method", "me", 0);
	aDeformMethod = nAttr.create("method", "me", MFnNumericData::kBoolean, 0.0);
	nAttr.setStorable(true);
	nAttr.setDefault(true);
	nAttr.setChannelBox(true);



	// Add the attribute
	addAttribute(aCurrentTime);
	addAttribute(aMass);
	addAttribute(aInitVelocity);
	addAttribute(aElasticity);
	addAttribute(aDynamicFriction);
	addAttribute(aDeformation);
	addAttribute(aStiffnes);
	addAttribute(aBounciness);
	addAttribute(aDeformMethod);

	// Link inputs that change the output of the mesh
	attributeAffects(aCurrentTime, outputGeom);
	attributeAffects(aMass, outputGeom);
	attributeAffects(aInitVelocity, outputGeom);
	attributeAffects(aElasticity, outputGeom);
	attributeAffects(aDynamicFriction, outputGeom);
	attributeAffects(aDeformation, outputGeom);
	attributeAffects(aStiffnes, outputGeom);
	attributeAffects(aBounciness, outputGeom);
	attributeAffects(aDeformMethod, outputGeom);

	// Make the deformer weights paintable
	//MGlobal::executeCommand("makePaintable -attrType multiFloat -sm deformer blendNode weights;");

	return MS::kSuccess;
}

MStatus initializePlugin(MObject obj) {
	MStatus status;
	MFnPlugin plugin(obj, "Erik Olsson", "1.0", "Any");

	// Specify we are making a deformer node
	status = plugin.registerNode("deformerNode", DeformerNode::id, DeformerNode::creator,
		DeformerNode::initialize, MPxNode::kDeformerNode);

	CHECK_MSTATUS_AND_RETURN_IT(status);

	cout.rdbuf(cerr.rdbuf()); //hack to get error messages out in Maya 2016.5

	cout << "-> Plugin Initialized" << endl;
	
	fflush(stdout);
	fflush(stderr);

	MGlobal::displayInfo("Hello World!");

	return status;
}

MStatus uninitializePlugin(MObject obj) {
	MStatus     status;
	MFnPlugin plugin(obj);

	status = plugin.deregisterNode(DeformerNode::id);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MGlobal::displayInfo("Goodbye World!");

	return status;
}