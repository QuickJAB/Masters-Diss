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

	CapsuleVolume* volume = new CapsuleVolume(height, radius);
	SetBoundingVolume((CollisionVolume*)volume);

	GetTransform()
		.SetScale(Vector3(radius * 2, height * 0.8, radius * 2))
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
	animCon->Update(dt);

	if (!isMoving) {
		vector<Matrix4> bindPose = renderObject->GetMesh()->GetBindPose();
		vector<int> parents = renderObject->GetMesh()->GetJointParents();
		unsigned int curFrame = animCon->GetCurrentFrame();

		for (auto& jointChain : effectorJointChain) {
			unsigned int currentJoint = jointChain.first;
			do {
				// Calculate new position
				Matrix4 position = Matrix4();

				((MeshAnimation*)animCon->GetCurrentAnimation())->SetJointValue(curFrame, currentJoint, position);
				currentJoint = parents.at(currentJoint);
			} while (currentJoint != jointChain.second);
		}
	}
}