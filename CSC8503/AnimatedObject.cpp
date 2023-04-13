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
	effectorJoints.push_back(28);
	effectorJoints.push_back(19); // On the same level as the joint above
	
	// Left leg end joints
	effectorJoints.push_back(29);
	effectorJoints.push_back(21); // On the same level as the joint above

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

		for (size_t i = 0; i < effectorJoints.size(); i++) {
			unsigned int currentJoint = effectorJoints.at(i);
			do {
				((MeshAnimation*)animCon->GetCurrentAnimation())->SetJointValue(curFrame, currentJoint, Matrix4());
				currentJoint = parents.at(currentJoint);
			} while (currentJoint != -1);
		}
	}
}