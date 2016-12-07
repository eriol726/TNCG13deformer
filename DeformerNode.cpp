#include "DeformerNode.h"
#include <maya/MFnPlugin.h>
#include <iostream>
#include <sstream>
#include <maya/MIOStream.h>

MTypeId DeformerNode::id(0x00000002);
MObject DeformerNode::aGravityMagnitude;
MObject DeformerNode::aGravityDirection;
MObject DeformerNode::aCurrentTime;
MObject DeformerNode::aInflation;

MTime DeformerNode::tPrevious;




void* DeformerNode::creator() { return new DeformerNode; }

MStatus DeformerNode::deform(MDataBlock& data, MItGeometry& itGeo,
	const MMatrix &localToWorldMatrix, unsigned int mIndex) {
	
	MStatus status;
	MMatrix worldToLocalMatrixInv = localToWorldMatrix.inverse();

	MTime currentTime = MAnimControl::currentTime();
	int currentFrame = (int)currentTime.value();
	
	
	float env = data.inputValue(envelope).asFloat();
	float inflation = data.inputValue(aInflation).asDouble();


	MTime currTime = data.inputValue(aCurrentTime).asTime();
	double gravity = data.inputValue(aGravityMagnitude).asDouble();
	MVector direction = data.inputValue(aGravityDirection).asVector();
	

	//Get the input mesh (fnInputMesh) http://www.chadvernon.com/
	MArrayDataHandle hInput = data.outputArrayValue(input, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status)
		status = hInput.jumpToElement(mIndex);
	CHECK_MSTATUS_AND_RETURN_IT(status)
		MObject oInputGeom = hInput.outputValue().child(inputGeom).asMesh();
	MFnMesh fnInputMesh(oInputGeom);

	


	if (currentFrame == 1)
	{
		
		std::vector<MPoint> initPositions;
		
		for (; !itGeo.isDone(); itGeo.next()) {

			MPoint pt = itGeo.position()*localToWorldMatrix;
			initPositions.push_back(pt); 

		}
		
		particleSystem = new ParticleSystem(initPositions);

		//http://www.ngreen.org/

		// Get the normal array from the input mesh
		MFloatVectorArray normals = MFloatVectorArray();
		fnInputMesh.getVertexNormals(true, normals, MSpace::kTransform);


		MPoint pt;
		MVector nrm;
		
	

		float w = 0.0f;
		// Loop through the geometry and set vertex positions
		// Get the current frame

		MTime tNow = data.inputValue(aCurrentTime).asTime();
		MTime timeDiff = tNow - tPrevious;
		int timeDiffInt = (int)timeDiff.value();
		tPrevious = tNow;
	}
	else {
		std::vector<MPoint> newPositions = particleSystem->shapeMatch();

		//MPointArray newPositions = particleSystem->getPositions();
	
		for (; !itGeo.isDone(); itGeo.next()) {

			int idx = itGeo.index();
			//nrm = MVector(normals[idx]);

			// Get the input point
			//pt = itGeo.position();

			//cout.rdbuf(cerr.rdbuf()); //hack to get error messages out in Maya 2016.5

									  // << "tNow: " << timeDiffInt << endl;
									  //cout << "timeDiff: " << currentFrame << endl;
			//cout << "newShape: " << newPositions[idx].y << endl;

			//fflush(stdout);
			//fflush(stderr);

			MPoint new_pos = newPositions.at(idx)*worldToLocalMatrixInv;

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

	// Time
	aCurrentTime = uAttr.create("currentTime", "tm", MFnUnitAttribute::kTime, 0.0);
	MTime currentFrame = MAnimControl::currentTime();
	//24 frames per second
	uAttr.setDefault(currentFrame.as(MTime::kFilm));
	uAttr.setChannelBox(true);

	// Gravity magnitude
	aGravityMagnitude = nAttr.create("gravityMagnitude", "gm", MFnNumericData::kDouble, 0.0);
	nAttr.setDefault(0.0);
	nAttr.setMin(0.0);
	nAttr.setMax(10.0);
	nAttr.setChannelBox(true);

	aGravityDirection = nAttr.create("gravityDirection", "gd", MFnNumericData::k3Double, 0.0);
	nAttr.setDefault(0.0);
	nAttr.setMin(-1.0);
	nAttr.setMax(1.0);
	nAttr.setChannelBox(true);

	aInflation = nAttr.create("inflation", "in", MFnNumericData::kDouble, 0.0);
	nAttr.setMin(0.0);
	nAttr.setMax(10.0);
	nAttr.setChannelBox(true);


	// Add the attribute
	addAttribute(aCurrentTime);
	addAttribute(aGravityMagnitude);
	addAttribute(aGravityDirection);
	addAttribute(aInflation);

	// Link inputs that change the output of the mesh
	attributeAffects(aCurrentTime, outputGeom);
	attributeAffects(aGravityMagnitude, outputGeom);
	attributeAffects(aGravityDirection, outputGeom);
	attributeAffects(aInflation, outputGeom);

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