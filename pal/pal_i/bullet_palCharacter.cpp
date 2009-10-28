//(c) Alion Science and Technology Inc. 2009, see liscence.txt (BSD liscence)
/** \file palCharacter.h
   \brief
      PAL - Physics Abstraction Layer.
      Character motion model
   \author
      David Guthrie
   \version
   <pre>
      Version 0.1   : 10/12/09 - Original
   </pre>
   \todo
*/
#include "bullet_palCharacter.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"

palBulletCharacterController::palBulletCharacterController()
: m_pKinematicCharacterController(NULL)
{


}

palBulletCharacterController::~palBulletCharacterController() {
	palPhysics* physics = palFactory::GetInstance()->GetActivePhysics();

	if (m_pKinematicCharacterController != NULL)
	{
		btPairCachingGhostObject* pairCachingGhost = m_pKinematicCharacterController->getGhostObject();

		if (pairCachingGhost != NULL)
		{
			if (physics != NULL)
			{
				btDynamicsWorld* world = dynamic_cast<palBulletPhysics*>(physics)->BulletGetDynamicsWorld();
				world->removeCollisionObject(pairCachingGhost);
				world->removeCharacter(m_pKinematicCharacterController);
			}

			delete pairCachingGhost->getBroadphaseHandle();
			delete pairCachingGhost->getCollisionShape();
			delete pairCachingGhost;
		}

		delete m_pKinematicCharacterController;
		m_pKinematicCharacterController = NULL;
	}
}

bool palBulletCharacterController::Init(palCharacterControllerDesc& desc) {
	palBulletGeometry* geom = dynamic_cast<palBulletGeometry*>(desc.m_pShape);
	m_pShape = geom;
	bool validData = false;
	if (geom != NULL)
	{
		btCollisionShape* shape = geom->BulletGetCollisionShape();
		if (shape != NULL && shape->isConvex())
		{
			btPairCachingGhostObject* pairCachingGhost = new btPairCachingGhostObject;
			btDynamicsWorld* world = dynamic_cast<palBulletPhysics*>(palFactory::GetInstance()->GetActivePhysics())->BulletGetDynamicsWorld();
			pairCachingGhost->setCollisionShape(shape);
			pairCachingGhost->setCollisionFlags(btCollisionObject::CF_CHARACTER_OBJECT);
			world->addCollisionObject(pairCachingGhost, convert_group(desc.m_Group), ~0);

			btConvexShape* convexShape = static_cast<btConvexShape*>(shape);

			unsigned int upAxis = palFactory::GetInstance()->GetActivePhysics()->GetUpAxis();

			m_pKinematicCharacterController = new btKinematicCharacterController(
						pairCachingGhost, convexShape, desc.m_fStepHeight, upAxis);
			// TODO: For some reason this doesn't work unless I set it.
			//m_pKinematicCharacterController->setUseGhostSweepTest(false);
			world->addCharacter(m_pKinematicCharacterController);
			validData = true;
		}
	}
	return validData;
}

void palBulletCharacterController::SetGroup(palGroup group) {
	m_pKinematicCharacterController->getGhostObject()->getBroadphaseHandle()->m_collisionFilterGroup = convert_group(group);
}

palGroup palBulletCharacterController::GetGroup() {
	return convert_to_pal_group(m_pKinematicCharacterController->getGhostObject()->getBroadphaseHandle()->m_collisionFilterGroup);
}

void palBulletCharacterController::Move(const palVector3& displacement) {
	m_pKinematicCharacterController->setWalkDirection(
				btVector3(displacement.x, displacement.y, displacement.z));
}

void palBulletCharacterController::Walk(const palVector3& walkVelocity, Float timeInterval) {

	m_pKinematicCharacterController->setVelocityForTimeInterval(
				btVector3(walkVelocity.x, walkVelocity.y, walkVelocity.z), timeInterval);
}

void palBulletCharacterController::WalkClear() {
   m_pKinematicCharacterController->reset();
}

void palBulletCharacterController::Warp(const palVector3& worldPos) {
	m_pKinematicCharacterController->warp(btVector3(worldPos.x, worldPos.y, worldPos.z));
}

palVector3& palBulletCharacterController::GetPosition() {
	btTransform btTrans = m_pKinematicCharacterController->getGhostObject()->getWorldTransform();
	for (unsigned i = 0; i < 3; ++i)
	{
		m_vPos._vec[i] = btTrans.getOrigin().m_floats[i];
	}
	return m_vPos;
}