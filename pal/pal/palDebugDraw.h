//(c) Adrian Boeing 2009, see liscence.txt (BSD liscence)
/** \file palDebugDraw.h
	\brief
		PAL - Physics Abstraction Layer.
		Debug Drawing utility class.
	\author
		David
	\version
	<pre>
	Revision History:
		Version 0.0.1: 21/12/09 - Initial Version
*/

#ifndef PALDEBUGDRAW_H
#define PALDEBUGDRAW_H

#include <framework/common.h>
#include <pal/palMath.h>

enum DebugDrawPrimitiveType {
	POINTS,
	LINES,
	TRIANGLES,
};

struct palDebugGeometry {
	palDebugGeometry()
	: m_eType(POINTS) {}
	DebugDrawPrimitiveType m_eType;
	PAL_VECTOR<palVector4> m_vColors;
	PAL_VECTOR<palVector3> m_vVertices;
	PAL_VECTOR<int> m_vIndices;
	void Clear() {
		m_vColors.clear();
		m_vVertices.clear();
		m_vIndices.clear();
	}
};

struct palDebugText {
	palVector3 m_vPos;
	PAL_STRING text;
};

class palDebugDraw {
public:
	palDebugDraw() {}
	virtual ~palDebugDraw() {}

	void Clear()
	{
		m_Lines.Clear();
		m_Points.Clear();
		m_Triangles.Clear();
		m_vTextItems.clear();
	}

	palDebugGeometry m_Lines;
	palDebugGeometry m_Points;
	palDebugGeometry m_Triangles;
	PAL_VECTOR<palDebugText> m_vTextItems;
};

#endif /* PALDEBUGDRAW_H */