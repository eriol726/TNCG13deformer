#ifndef DOUBLERNODE_H
#define DOUBLERNODE_H

#include <maya/mdatablock.h>
#include <maya/mdatahandle.h>
#include <maya/mstatus.h>
#include <maya/MGlobal.h>

#include <maya/mfnnumericattribute.h>

#include <maya/mpxnode.h>

class DoublerNode : public MPxNode {
public:
	DoublerNode() {};
	virtual   ~DoublerNode() {};
	virtual MStatus compute(const MPlug& plug, MDataBlock& data);
	MStatus deform(MDataBlock& data, MItGeometry& itGeo, const MMatrix &localToWorldMatrix, unsigned int geomIndex);

	static void* creator();
	static MStatus initialize();

	static MTypeId id;
	static MObject aInput;
	static MObject aOutput;
};
#endif