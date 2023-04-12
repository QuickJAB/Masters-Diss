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
	
	// TEMPORARY HARD CODE TEST
	//for (int i = 0; i < 15; i++) animations.at("idle")->AddEffectorJoint(i);

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
		//vector<Matrix4> bindPose = renderObject->GetMesh()->GetBindPose();
		//for (int i = 0; i < 82; i++) {
		//	std::cout << i << ", ";
		//	vector<int> parents = renderObject->GetMesh()->GetJointParents();
		//	int jointID = i;
		//	while (parents.at(jointID) != -1) {
		//		std::cout << parents.at(jointID) << ", ";
		//		jointID = parents.at(jointID);
		//	}
		//	std::cout << std::endl;
		//}


		// Right leg
		int jointID = 28;
		vector<Matrix4> bindPose = renderObject->GetMesh()->GetBindPose();
		vector<int> parents = renderObject->GetMesh()->GetJointParents();
		unsigned int curFrame = animCon->GetCurrentFrame();
		do {
			((MeshAnimation*)animCon->GetCurrentAnimation())->SetJointValue(curFrame, jointID, Matrix4());
			jointID = parents.at(jointID);
		} while (jointID != -1);
		((MeshAnimation*)animCon->GetCurrentAnimation())->SetJointValue(curFrame, 19, Matrix4()); // Confused what this is

		// Left leg
		jointID = 29;
		do {
			((MeshAnimation*)animCon->GetCurrentAnimation())->SetJointValue(curFrame, jointID, Matrix4());
			jointID = parents.at(jointID);
		} while (jointID != -1);
		((MeshAnimation*)animCon->GetCurrentAnimation())->SetJointValue(curFrame, 21, Matrix4());  // Confused what this is
	}
}