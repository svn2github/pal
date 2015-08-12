#ifndef BULLET_PAL_LINKS_H
#define BULLET_PAL_LINKS_H

#include "bullet_pal.h"

class palBulletSphericalLink : public palSphericalLink {
public:
	palBulletSphericalLink();
	virtual ~palBulletSphericalLink();
	virtual void Init(palBodyBase *parent, palBodyBase *child,
			const palMatrix4x4& parentFrame, const palMatrix4x4& childFrame, bool disableCollisionsBetweenLinkedBodies);
	virtual void Init(palBodyBase *parent, palBodyBase *child, const palVector3& pos, const palVector3& axis, bool disableCollisionsBetweenLinkedBodies);

	virtual void ComputeFrameParent(palMatrix4x4& frameOut) const { convertBtTransformToPalMat(frameOut, m_btConeTwist->getFrameOffsetA()); }
	virtual void ComputeFrameChild(palMatrix4x4& frameOut) const { convertBtTransformToPalMat(frameOut, m_btConeTwist->getFrameOffsetB()); }

	/*override*/ bool SetParam(int parameterCode, Float value, int axis);
	/*override*/ Float GetParam(int parameterCode, int axis);
	/*override*/ bool SupportsParameters() const;
	/*override*/ bool SupportsParametersPerAxis() const;

	/*override*/ bool GetEnabled() const { return m_btConeTwist->isEnabled(); }
	/*override*/ bool SetEnabled(bool enable)
	{
		m_btConeTwist->setEnabled(enable);
		return true;
	}


	btConeTwistConstraint *m_btConeTwist;
protected:
	FACTORY_CLASS(palBulletSphericalLink,palSphericalLink,Bullet,1)
};

class bulletRevoluteLinkFeedback : public palLinkFeedback {
public:
	bulletRevoluteLinkFeedback(btHingeConstraint *hinge);
	virtual bool IsEnabled() const;
	virtual bool SetEnabled(bool enable);
	virtual Float GetValue() const;
protected:
	btHingeConstraint *m_btHinge;
};

class palBulletRevoluteLink: public palRevoluteLink {
public:
	palBulletRevoluteLink();
	virtual ~palBulletRevoluteLink();
	virtual void Init(palBodyBase *parent, palBodyBase *child,
			const palMatrix4x4& parentFrame, const palMatrix4x4& childFrame, bool disableCollisionsBetweenLinkedBodies);
	virtual void Init(palBodyBase *parent, palBodyBase *child, const palVector3& pos, const palVector3& axis, bool disableCollisionsBetweenLinkedBodies);

	virtual Float GetAngle() const;
	virtual void GetPosition(palVector3& pos) const;

	virtual void ComputeFrameParent(palMatrix4x4& frameOut) const { convertBtTransformToPalMat(frameOut, m_btHinge->getFrameOffsetA()); }
	virtual void ComputeFrameChild(palMatrix4x4& frameOut) const { convertBtTransformToPalMat(frameOut, m_btHinge->getFrameOffsetB()); }

	/*override*/ bool SetParam(int parameterCode, Float value, int axis);
	/*override*/ Float GetParam(int parameterCode, int axis);
	/*override*/ bool SupportsParameters() const;
	/*override*/ bool SupportsParametersPerAxis() const;

	/*override*/ bool GetEnabled() const { return m_btHinge->isEnabled(); }
	/*override*/ bool SetEnabled(bool enable)
	{
		m_btHinge->setEnabled(enable);
		return true;
	}

	virtual palLinkFeedback* GetFeedback() const throw(palIllegalStateException);
	btHingeConstraint *m_btHinge;
protected:
	bulletRevoluteLinkFeedback* m_feedback;
	FACTORY_CLASS(palBulletRevoluteLink,palRevoluteLink,Bullet,1)
};

#if BT_BULLET_VERSION < 283
/// Needed to create a subclass because the actual bullet link didn't allow setting the equilibrium point directly
class SubbtGeneric6DofSpringConstraint : public btGeneric6DofSpringConstraint {
public:
	SubbtGeneric6DofSpringConstraint(btRigidBody& rbA, btRigidBody& rbB, const btTransform& frameInA, const btTransform& frameInB ,bool useLinearReferenceFrameA)
: btGeneric6DofSpringConstraint(rbA, rbB, frameInA, frameInB, useLinearReferenceFrameA)
{
}
	virtual ~SubbtGeneric6DofSpringConstraint() {}

	void setEquilibriumPoint(int index, btScalar point) {
		btAssert((index >= 0) && (index < 6));
		if (index < 3) {
			m_equilibriumPoint[index] = point;
		} else {
			m_equilibriumPoint[index] = btNormalizeAngle(point);
		}
	}

	void getSpringDesc(int index, palSpringDesc& desc) const {
		desc.m_fDamper = m_springDamping[index];
		desc.m_fSpringCoef = m_springStiffness[index];
		desc.m_fTarget = m_equilibriumPoint[index];
	}
};
#endif

class palBulletRevoluteSpringLink: public palRevoluteSpringLink {
public:
	palBulletRevoluteSpringLink();
	virtual ~palBulletRevoluteSpringLink();
	virtual void Init(palBodyBase *parent, palBodyBase *child,
			const palMatrix4x4& parentFrame, const palMatrix4x4& childFrame, bool disableCollisionsBetweenLinkedBodies);
	virtual void Init(palBodyBase *parent, palBodyBase *child, const palVector3& pos, const palVector3& axis, bool disableCollisionsBetweenLinkedBodies);

	virtual void ComputeFrameParent(palMatrix4x4& frameOut) const { convertBtTransformToPalMat(frameOut, m_bt6Dof->getFrameOffsetA()); }
	virtual void ComputeFrameChild(palMatrix4x4& frameOut) const { convertBtTransformToPalMat(frameOut, m_bt6Dof->getFrameOffsetB()); }

	virtual void SetSpring(const palSpringDesc& springDesc);
	virtual void GetSpring(palSpringDesc& springDescOut) const;

	/*override*/ bool SetParam(int parameterCode, Float value, int axis);
	/*override*/ Float GetParam(int parameterCode, int axis);
	/*override*/ bool SupportsParameters() const;
	/*override*/ bool SupportsParametersPerAxis() const;

	/*override*/ bool GetEnabled() const { return m_bt6Dof->isEnabled(); }
	/*override*/ bool SetEnabled(bool enable)
	{
		m_bt6Dof->setEnabled(enable);
		return true;
	}

protected:
#if BT_BULLET_VERSION < 283
	SubbtGeneric6DofSpringConstraint *m_bt6Dof;
#else
	btGeneric6DofSpring2Constraint *m_bt6Dof;
#endif
	FACTORY_CLASS(palBulletRevoluteSpringLink,palRevoluteSpringLink,Bullet,1)
};

class palBulletPrismaticLink:  public palPrismaticLink {
public:
	palBulletPrismaticLink();
	virtual ~palBulletPrismaticLink();
	virtual void Init(palBodyBase *parent, palBodyBase *child,
			const palMatrix4x4& parentFrame, const palMatrix4x4& childFrame, bool disableCollisionsBetweenLinkedBodies);
	virtual void Init(palBodyBase *parent, palBodyBase *child, const palVector3& pos, const palVector3& axis, bool disableCollisionsBetweenLinkedBodies);

	virtual void ComputeFrameParent(palMatrix4x4& frameOut) const { convertBtTransformToPalMat(frameOut, m_btSlider->getFrameOffsetA()); }
	virtual void ComputeFrameChild(palMatrix4x4& frameOut) const { convertBtTransformToPalMat(frameOut, m_btSlider->getFrameOffsetB()); }

	/*override*/ bool SetParam(int parameterCode, Float value, int axis);
	/*override*/ Float GetParam(int parameterCode, int axis);
	/*override*/ bool SupportsParameters() const;
	/*override*/ bool SupportsParametersPerAxis() const;

	/*override*/ bool GetEnabled() const { return m_btSlider->isEnabled(); }
	/*override*/ bool SetEnabled(bool enable)
	{
		m_btSlider->setEnabled(enable);
		return true;
	}

	btSliderConstraint* m_btSlider;
protected:
	FACTORY_CLASS(palBulletPrismaticLink,palPrismaticLink,Bullet,1)
};

class palBulletGenericLink : public palGenericLink {
public:
	palBulletGenericLink();
	virtual ~palBulletGenericLink();
	virtual void Init(palBodyBase *parent, palBodyBase *child,
			const palMatrix4x4& parentFrame, const palMatrix4x4& childFrame, bool disableCollisionsBetweenLinkedBodies);
	virtual void Init(palBodyBase *parent, palBodyBase *child, const palVector3& pos, const palVector3& axis, bool disableCollisionsBetweenLinkedBodies);

	virtual void ComputeFrameParent(palMatrix4x4& frameOut) const { convertBtTransformToPalMat(frameOut, genericConstraint->getFrameOffsetA()); }
	virtual void ComputeFrameChild(palMatrix4x4& frameOut) const { convertBtTransformToPalMat(frameOut, genericConstraint->getFrameOffsetB()); }

	/*override*/ bool SetParam(int parameterCode, Float value, int axis);
	/*override*/ Float GetParam(int parameterCode, int axis);
	/*override*/ bool SupportsParameters() const;
	/*override*/ bool SupportsParametersPerAxis() const;

	/*override*/ bool GetEnabled() const { return genericConstraint->isEnabled(); }
	/*override*/ bool SetEnabled(bool enable)
	{
		genericConstraint->setEnabled(enable);
		return true;
	}

#if BT_BULLET_VERSION < 283
	SubbtGeneric6DofSpringConstraint* BulletGetGenericConstraint() { return genericConstraint; }
protected:
	SubbtGeneric6DofSpringConstraint* genericConstraint;
#else
	btGeneric6DofSpring2Constraint* BulletGetGenericConstraint() { return genericConstraint; }
protected:
	btGeneric6DofSpring2Constraint* genericConstraint;
#endif
	FACTORY_CLASS(palBulletGenericLink,palGenericLink,Bullet,1)
};

class palBulletRigidLink : public palRigidLink {
public:
	palBulletRigidLink();
	virtual ~palBulletRigidLink();
	virtual void Init(palBodyBase *parent, palBodyBase *child,
			const palMatrix4x4& parentFrame, const palMatrix4x4& childFrame, bool disableCollisionsBetweenLinkedBodies);
	virtual void Init(palBodyBase *parent, palBodyBase *child, const palVector3& pos, const palVector3& axis, bool disableCollisionsBetweenLinkedBodies);

	virtual void ComputeFrameParent(palMatrix4x4& frameOut) const { convertBtTransformToPalMat(frameOut, m_btFixed->getFrameOffsetA()); }
	virtual void ComputeFrameChild(palMatrix4x4& frameOut) const { convertBtTransformToPalMat(frameOut, m_btFixed->getFrameOffsetB()); }

	/*override*/ bool SetParam(int parameterCode, Float value, int axis);
	/*override*/ Float GetParam(int parameterCode, int axis);
	/*override*/ bool SupportsParameters() const;
	/*override*/ bool SupportsParametersPerAxis() const;

	/*override*/ bool GetEnabled() const { return m_btFixed->isEnabled(); }
	/*override*/ bool SetEnabled(bool enable)
	{
		m_btFixed->setEnabled(enable);
		return true;
	}

protected:
	FACTORY_CLASS(palBulletRigidLink,palRigidLink,Bullet,1)
private:
// there are a few issues with the fixed constraint such as unsupported parameters and no access to the frames.
#if BT_BULLET_VERSION > 281
	btFixedConstraint *m_btFixed;
#else
	btHingeConstraint *m_btFixed;
#endif
};

class palBulletGenericLinkSpring : public palGenericLinkSpring {
public:
	typedef palGenericLinkSpring BaseClass;

	palBulletGenericLinkSpring();

	virtual void Init(palGenericLink* link);

	virtual void SetLinearSpring(palAxis axis, const palSpringDesc& spring);

	virtual void GetLinearSpring(palAxis axis, palSpringDesc& out) const;

	virtual void SetAngularSpring(palAxis axis, const palSpringDesc& spring);

	virtual void GetAngularSpring(palAxis axis, palSpringDesc& out) const;

	virtual void Apply(float dt);

	palBulletGenericLink* BulletGetLink() { return m_pBulletLink; }
private:
	palBulletGenericLink* m_pBulletLink;
	FACTORY_CLASS(palBulletGenericLinkSpring,palGenericLinkSpring,Bullet,1);
};


#endif // BULLET_PAL_LINKS_H
