#include "AnimatedObject.h"
#include "GameWorld.h"
#include "PhysicsObject.h"
#include "RenderObject.h"
#include "GameTechRenderer.h"
#include "AnimationController.h"
#include "MeshAnimation.h"

using namespace NCL;
using namespace CSC8503;

AnimatedObject::AnimatedObject(const Vector3& position, GameTechRenderer* renderer) {
	std::unordered_map<std::string, MeshAnimation*> animations;

	animations.insert(std::make_pair("idle", new MeshAnimation("AJIdle.anm")));
	animations.insert(std::make_pair("run", new MeshAnimation("AJRun.anm")));
	
	// Right leg end joints
	effectorJointChain.emplace(28, 0);
	effectorJointChain.emplace(19, 0); // On the same level as the joint above
	
	// Left leg end joints
	effectorJointChain.emplace(29, 0);
	effectorJointChain.emplace(21, 0); // On the same level as the joint above

	animCon = new AnimationController(this, animations);

	float radius = 2.0f;
	float inverseMass = 0.9f;

	SphereVolume* volume = new SphereVolume(radius);
	SetBoundingVolume((CollisionVolume*)volume);

	GetTransform()
		.SetScale(Vector3(radius, radius, radius))
		.SetPosition(position);

	SetRenderObject(new RenderObject(&GetTransform(), renderer->LoadMesh("Aj_Tpose.msh"), nullptr, renderer->LoadShader("skinning.vert", "character.frag")));
	GetRenderObject()->SetRigged(true);
	GetRenderObject()->SetAnimationController(animCon);
	GetRenderObject()->SetColour({ 1, 0, 0, 1 });

	SetPhysicsObject(new PhysicsObject(&GetTransform(), GetBoundingVolume()));
	GetPhysicsObject()->SetInverseMass(inverseMass);
	GetPhysicsObject()->InitSphereInertia();
	GetPhysicsObject()->SetElasticity(0);

	isMoving = false;
}

AnimatedObject::~AnimatedObject() {
	delete animCon;
}

void AnimatedObject::SolveIK(const Vector3& snapPoint, unsigned int currentJoint, const unsigned int& endJoint) {
	vector<Matrix4> bindPose = renderObject->GetMesh()->GetBindPose();
	vector<int> parents = renderObject->GetMesh()->GetJointParents();
	unsigned int curFrame = animCon->GetCurrentFrame();
	unsigned int previousJoint = 999;
	
	while (currentJoint != endJoint) {
		Matrix4 position = Matrix4::Translation(snapPoint - GetTransform().GetPosition());

		if (previousJoint != 999) {
			Matrix4 jointOffset = ((MeshAnimation*)animCon->GetCurrentAnimation())->GetJointOffset(curFrame, previousJoint, currentJoint);
			position = (position - jointOffset) * bindPose.at(parents.at(currentJoint));
		} else {
			position = position * bindPose.at(parents.at(currentJoint));
		}

		((MeshAnimation*)animCon->GetCurrentAnimation())->SetJointValue(curFrame, currentJoint, position);
		previousJoint = currentJoint;
		currentJoint = parents.at(currentJoint);
	}


}

void AnimatedObject::ResetIK(unsigned int currentJoint, const unsigned int& endJoint) {
	vector<int> parents = renderObject->GetMesh()->GetJointParents();
	unsigned int curFrame = animCon->GetCurrentFrame();

	unsigned int previousJoint = 999;
	while (currentJoint != endJoint) {
		((MeshAnimation*)animCon->GetCurrentAnimation())->ResetJointValue(curFrame, currentJoint);
		previousJoint = currentJoint;
		currentJoint = parents.at(currentJoint);
	}
}

void AnimatedObject::Update(float dt) {
	animCon->Update(dt);

	if (!isMoving && GetPhysicsObject()->GetLinearVelocity().y >= -.1f && world != nullptr) {
		for (auto& jointChain : effectorJointChain) {
			Vector3 jointWorldSpace = (GetTransform().GetMatrix() *
				(renderObject->GetMesh()->GetBindPose().at(jointChain.first) * 
					renderObject->GetMesh()->GetInverseBindPose().at(renderObject->GetMesh()->GetJointParents().at(jointChain.first))))
				.GetPositionVector();

			Ray ray = Ray(jointWorldSpace, Vector3(0, -1, 0));
			RayCollision closestCollision;
			world->Raycast(ray, closestCollision, true, this);

			if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::E))
				Debug::DrawLine(jointWorldSpace, jointWorldSpace + Vector3(0, -1, 0), { 0, 0, 1, 1 }, 3);

			if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::Q))
				DrawSkeleton();

			if (closestCollision.rayDistance > 3.0f) {
				SolveIK( closestCollision.collidedAt, jointChain.first, jointChain.second);
			} else {
				ResetIK(jointChain.first, jointChain.second);
			}
		}
	}
}

void AnimatedObject::DrawSkeleton() {
	for (int i = 0; i < 82; i++) {
		vector<int> parents = renderObject->GetMesh()->GetJointParents();
		int jointID = i;
		while (parents.at(jointID) != -1) {
			Vector3 jointWorldSpace = (GetTransform().GetMatrix() *
				(renderObject->GetMesh()->GetBindPose().at(jointID) *
					renderObject->GetMesh()->GetInverseBindPose().at(renderObject->GetMesh()->GetJointParents().at(jointID))))
				.GetPositionVector();

			Vector3 parentWorldSpace = (GetTransform().GetMatrix() *
				(renderObject->GetMesh()->GetBindPose().at(parents.at(jointID)) *
					renderObject->GetMesh()->GetInverseBindPose().at(renderObject->GetMesh()->GetJointParents().at(parents.at(jointID)))))
				.GetPositionVector();
			
			Debug::DrawLine(jointWorldSpace, parentWorldSpace, { 0, 0, 1, 1 }, 10);
			jointID = parents.at(jointID);
		}
	}
}