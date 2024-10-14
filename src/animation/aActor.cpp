#include "aActor.h"

#pragma warning(disable : 4018)



/****************************************************************
*
*    	    Actor functions
*
****************************************************************/

AActor::AActor() 
{
	m_pInternalSkeleton = new ASkeleton();
	m_pSkeleton = m_pInternalSkeleton;

	m_BVHController = new BVHController();
	m_BVHController->setActor(this);

	m_IKController = new IKController();
	m_IKController->setActor(this);

	m_BehaviorController = new BehaviorController();
	m_BehaviorController->setActor(this);

	// code to update additional Actor data goes here
	resetGuide();

}

AActor::AActor(const AActor* actor)
{
	*this = *actor;
}

AActor& AActor::operator = (const AActor& actor)
{
	// Performs a deep copy
	if (&actor == this)
	{
		return *this;
	}
	m_pSkeleton = actor.m_pSkeleton;

	// code to update additional Actor data goes here


	return *this;
}

AActor::~AActor()
{
	 delete m_BehaviorController;
	 delete m_IKController;
	 delete m_BVHController;
	 delete m_pInternalSkeleton;

}

void AActor::clear()
{
	// looks like it is clearing more times than the number of actors.  as a result, m_pSkeleton is not defined for last case.
	m_pSkeleton->clear();  

	// code to update additional Actor data goes here
}

void AActor::update()
{
	if (!m_pSkeleton->getRootNode() )
		 return; // Nothing loaded
	else m_pSkeleton->update();

	// code to update additional Actor data goes here

}

ASkeleton* AActor::getSkeleton()
{
	return m_pSkeleton;
}

void AActor::setSkeleton(ASkeleton* pExternalSkeleton)
{
	m_pSkeleton = pExternalSkeleton;
}

void AActor::resetSkeleton()
{
	m_pSkeleton = m_pInternalSkeleton;
}

BVHController* AActor::getBVHController()
{
	return m_BVHController;
}

IKController* AActor::getIKController()
{
	return m_IKController;
}

BehaviorController* AActor::getBehaviorController()
{
	return m_BehaviorController;
}

void AActor::updateGuideJoint(vec3 guideTargetPos)
{
	if (!m_pSkeleton->getRootNode()) { return; }

	// TODO: 
	// 1.	Set the global position of the guide joint to the global position of the root joint
	// 2.	Set the y component of the guide position to 0
	// 3.	Set the global rotation of the guide joint towards the guideTarget
	// Hint: Return of getGlobal***() here is in target space but not in world/global space
	vec3 rootPos = m_pSkeleton->getRootNode()->getGlobalTranslation();
	rootPos = m_Guide.getGlobalRotation() * rootPos + m_Guide.getGlobalTranslation();
	rootPos[1] = 0.f;
	m_Guide.setGlobalTranslation(rootPos);
	
	guideTargetPos[1] = 0.f;
	vec3 d1 = vec3(0.f, 0.f, 1.f);
	vec3 d2 = guideTargetPos - rootPos;
	vec3 axis = d1.Cross(d2);
	axis.Normalize();
	double theta = acos(Dot(d1, d2) / d2.Length());
	quat a = quat();
	a.FromAxisAngle(axis, theta);
	mat3 rot = a.ToRotation();
	m_Guide.setGlobalRotation(rot);
}

void AActor::solveFootIK(float leftHeight, float rightHeight, bool rotateLeft, bool rotateRight, vec3 leftNormal, vec3 rightNormal)
{
	if (!m_pSkeleton->getRootNode()) { return; }
	AJoint* leftFoot = m_pSkeleton->getJointByID(m_IKController->mLfootID);
	AJoint* rightFoot = m_pSkeleton->getJointByID(m_IKController->mRfootID);

	// TODO: 
	// The normal and the height given are in the world space
	// Hint: Return of getGlobal***() here is in target space but not in world/global space

	// 1.	Update the local translation of the root based on the left height and the right height
	
	// store left foot and right foot original world pos
	vec3 leftFootW = leftFoot->getGlobalTranslation();
	vec3 rightFootW = rightFoot->getGlobalTranslation();
	
	// update root
	vec3 rootPosW = m_pSkeleton->getRootNode()->getLocalTranslation();
	rootPosW[1] += min(leftHeight, rightHeight);
	m_pSkeleton->getRootNode()->setLocalTranslation(rootPosW);
	m_pSkeleton->update();

	// 2.	Update the character with Limb-based IK 
	
	// update left foot and right foot world location using leftheight and rightheight
	leftFootW[1] += leftHeight;
	rightFootW[1] += rightHeight;

	ATarget leftTarget, rightTarget;
	leftTarget.setGlobalTranslation(leftFootW);
	rightTarget.setGlobalTranslation(rightFootW);
	m_IKController->IKSolver_Limb(leftFoot->getID(), leftTarget);
	m_IKController->IKSolver_Limb(rightFoot->getID(), rightTarget);

	// Rotate Foot
	if (rotateLeft)
	{
		// Update the local orientation of the left foot based on the left normal
		leftNormal = leftNormal.Normalize();
		vec3 leftFootZW = leftFoot->getGlobalRotation().GetCol(2);
		vec3 leftFootXN = leftNormal.Cross(leftFootZW).Normalize();
		vec3 leftFootZN = leftFootXN.Cross(leftNormal).Normalize();
		mat3 l2W = mat3(leftFootXN, leftNormal, leftFootZN).Transpose();
		mat3 l2P = leftFoot->getParent()->getGlobalRotation().Inverse() * l2W;
		leftFoot->setLocalRotation(l2P);
	}
	if (rotateRight)
	{
		// Update the local orientation of the right foot based on the right normal
		rightNormal = rightNormal / rightNormal.Length();
		vec3 rightFootZW = rightFoot->getGlobalRotation().GetCol(2);
		vec3 rightFootXN = rightNormal.Cross(rightFootZW).Normalize();
		vec3 rightFootZN = rightFootXN.Cross(rightNormal).Normalize();
		mat3 l2W = mat3(rightFootXN, rightNormal, rightFootZN).Transpose();
		mat3 l2P = rightFoot->getParent()->getGlobalRotation().Inverse() * l2W;
		rightFoot->setLocalRotation(l2P);
	}
	m_pSkeleton->update();
}
