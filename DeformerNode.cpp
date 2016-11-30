#include "DeformerNode.h"
#include <maya/MFnPlugin.h>


MTypeId DeformerNode::id(0x00000002);
MObject DeformerNode::aGravityMagnitude;
MObject DeformerNode::aGravityDirection;
MObject DeformerNode::aCurrentTime;

void* DeformerNode::creator() { return new DeformerNode; }

MStatus DeformerNode::deform(MDataBlock& data, MItGeometry& itGeo,
	const MMatrix &localToWorldMatrix, unsigned int mIndex) {
	
	MStatus status;

	MTime currTime = data.inputValue(aCurrentTime).asTime();
	float env = data.inputValue(envelope).asFloat();
	double gravity = data.inputValue(aGravityMagnitude).asDouble();
	MVector direction = data.inputValue(aGravityDirection).asVector();
	

	//http://www.chadvernon.com/
	MArrayDataHandle hInput = data.outputArrayValue(input, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status)
		status = hInput.jumpToElement(mIndex);
	CHECK_MSTATUS_AND_RETURN_IT(status)
		MObject oInputGeom = hInput.outputValue().child(inputGeom).asMesh();
	MFnMesh fnInputMesh(oInputGeom);

	// Get the normal array from the input mesh
	MFloatVectorArray normals = MFloatVectorArray();
	fnInputMesh.getVertexNormals(true, normals, MSpace::kTransform);


	MPoint pt;
	float w = 0.0f;
	for (; !itGeo.isDone(); itGeo.next()) {

		int idx = itGeo.index();
		// Get the input point
		pt = itGeo.position();

		MPoint new_pos = pt * localToWorldMatrix;

		if (new_pos.y < 0.0)
			 new_pos.y = 0.0;

		// Set the new output point
		itGeo.setPosition(new_pos * localToWorldMatrix.inverse());
	}

	return MS::kSuccess;
}

MStatus DeformerNode::initialize() {
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

	// Add the attribute
	addAttribute(aCurrentTime);
	addAttribute(aGravityMagnitude);
	addAttribute(aGravityDirection);

	// Affect
	attributeAffects(aCurrentTime, outputGeom);
	attributeAffects(aGravityMagnitude, outputGeom);
	attributeAffects(aGravityDirection, outputGeom);

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

	return status;
}

MStatus uninitializePlugin(MObject obj) {
	MStatus     status;
	MFnPlugin plugin(obj);

	status = plugin.deregisterNode(DeformerNode::id);
	CHECK_MSTATUS_AND_RETURN_IT(status);


	return status;
}