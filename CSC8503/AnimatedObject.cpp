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
	effectorJointChain.emplace(28, 2);
	effectorJointChain.emplace(19, 2); // On the same level as the joint above
	
	// Left leg end joints
	effectorJointChain.emplace(29, 3);
	effectorJointChain.emplace(21, 3); // On the same level as the joint above

	animCon = new AnimationController(this, animations);

	float radius = 1.0f;
	float height = 3.0f;
	float inverseMass = 0.9f;

	//CapsuleVolume* volume = new CapsuleVolume(height, radius);
	SphereVolume* volume = new SphereVolume(height);
	SetBoundingVolume((CollisionVolume*)volume);

	GetTransform()
		.SetScale(Vector3(radius * 2, height, radius * 2))
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

void AnimatedObject::Update(float dt) {
	if (world == nullptr) return;
	
	animCon->Update(dt);

	if (!isMoving) {
		vector<Matrix4> bindPose = renderObject->GetMesh()->GetBindPose();
		vector<Matrix4> invBindPose = renderObject->GetMesh()->GetInverseBindPose();
		vector<int> parents = renderObject->GetMesh()->GetJointParents();
		unsigned int curFrame = animCon->GetCurrentFrame();

		for (auto& jointChain : effectorJointChain) {
			unsigned int currentJoint = jointChain.first;

			Vector3 jointWorldSpace = GetTransform().GetPosition() + (bindPose.at(currentJoint) * invBindPose.at(parents.at(currentJoint))).GetPositionVector();

			Ray ray = Ray(jointWorldSpace, Vector3(0, -1, 0));
			RayCollision closestCollision;
			world->Raycast(ray, closestCollision, true, this);

			if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::E)) Debug::DrawLine(jointWorldSpace, jointWorldSpace + Vector3(0, -1, 0) * 2, { 0, 0, 1, 1 }, 3);

			if (closestCollision.rayDistance > 3.0f) {
				do {
					// Calculate new position
					Matrix4 position = Matrix4();
					//Matrix4 position = closestCollision.collidedAt; need to figure this outs

					((MeshAnimation*)animCon->GetCurrentAnimation())->SetJointValue(curFrame, currentJoint, position);
					currentJoint = parents.at(currentJoint);
				} while (currentJoint != jointChain.second);
			}
			else {
				do {
					((MeshAnimation*)animCon->GetCurrentAnimation())->ResetJointValue(curFrame, currentJoint);
					currentJoint = parents.at(currentJoint);
				} while (currentJoint != jointChain.second);
			}
		}
	}
}