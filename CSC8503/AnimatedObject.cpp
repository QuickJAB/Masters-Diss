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

void AnimatedObject::SolveIK(const float& rayDist, const Vector3& snapPoint, unsigned int currentJoint, const unsigned int& endJoint) {
	vector<Matrix4> bindPose = renderObject->GetMesh()->GetBindPose();
	vector<int> parents = renderObject->GetMesh()->GetJointParents();
	unsigned int curFrame = animCon->GetCurrentFrame();
	unsigned int previousJoint = 999;

	if (rayDist > 3.0f) {
		do {
			Matrix4 position = Matrix4::Translation(snapPoint - GetTransform().GetPosition());

			if (previousJoint != 999) {
				Matrix4 jointOffset = ((MeshAnimation*)animCon->GetCurrentAnimation())->GetJointOffset(curFrame, previousJoint, currentJoint);
				position = (position - jointOffset) * bindPose.at(parents.at(currentJoint));
			}
			else {
				position = position * bindPose.at(parents.at(currentJoint));
			}

			((MeshAnimation*)animCon->GetCurrentAnimation())->SetJointValue(curFrame, currentJoint, position);
			previousJoint = currentJoint;
			currentJoint = parents.at(currentJoint);
		} while (currentJoint != endJoint);
	}
	else {
		do {
			((MeshAnimation*)animCon->GetCurrentAnimation())->ResetJointValue(curFrame, currentJoint);
			previousJoint = currentJoint;
			currentJoint = parents.at(currentJoint);
		} while (currentJoint != endJoint);
	}
}

void AnimatedObject::Update(float dt) {
	animCon->Update(dt);

	if (!isMoving && GetPhysicsObject()->GetLinearVelocity().y >= -.1f && world != nullptr) {
		vector<Matrix4> bindPose = renderObject->GetMesh()->GetBindPose();
		vector<Matrix4> invBindPose = renderObject->GetMesh()->GetInverseBindPose();
		vector<int> parents = renderObject->GetMesh()->GetJointParents();

		for (auto& jointChain : effectorJointChain) {
			unsigned int currentJoint = jointChain.first;

			Vector3 jointWorldSpace = GetTransform().GetPosition() + (bindPose.at(currentJoint) * invBindPose.at(parents.at(currentJoint))).GetPositionVector();

			Ray ray = Ray(jointWorldSpace, Vector3(0, -1, 0));
			RayCollision closestCollision;
			world->Raycast(ray, closestCollision, true, this);

			if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::E)) Debug::DrawLine(jointWorldSpace, jointWorldSpace + Vector3(0, -1, 0), { 0, 0, 1, 1 }, 3);

			SolveIK(closestCollision.rayDistance, closestCollision.collidedAt, currentJoint, jointChain.second);
		}
	}
}