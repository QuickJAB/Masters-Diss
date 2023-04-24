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

	animations.insert(std::make_pair("idle", new MeshAnimation("Idle.anm")));
	animations.insert(std::make_pair("run", new MeshAnimation("Walk.anm")));
	
	// Right leg end joints
	//effectorJointChain.emplace(28, 0);
	//effectorJointChain.emplace(19, 0); // On the same level as the joint above
	
	// Left leg end joints
	//effectorJointChain.emplace(29, 0);
	//effectorJointChain.emplace(21, 0); // On the same level as the joint above

	animCon = new AnimationController(this, animations);

	float radius = 2.0f;
	float inverseMass = 0.9f;

	SphereVolume* volume = new SphereVolume(radius);
	SetBoundingVolume((CollisionVolume*)volume);

	GetTransform()
		.SetScale(Vector3(radius, radius, radius))
		.SetPosition(position);

	SetRenderObject(new RenderObject(&GetTransform(), renderer->LoadMesh("Idle.msh"), nullptr, renderer->LoadShader("skinning.vert", "character.frag")));
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

	vector<int> parents = renderObject->GetMesh()->GetJointParents();
	const vector<Matrix4> bindPose = renderObject->GetMesh()->GetBindPose();
	const vector<Matrix4> invBindPose = renderObject->GetMesh()->GetInverseBindPose();
	const Matrix4 modelMat = GetTransform().GetMatrix();

	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::Q))
		DrawSkeleton();

	((MeshAnimation*)animCon->GetCurrentAnimation())->FixRootPosition(animCon->GetCurrentFrame());

	if (!isMoving && GetPhysicsObject()->GetLinearVelocity().y >= -.1f && world != nullptr) {
		for (auto& jointChain : effectorJointChain) {
			Vector3 jointWorldSpace = (modelMat * bindPose.at(jointChain.first) * invBindPose.at(parents.at(jointChain.first))).GetPositionVector();

			Ray ray = Ray(jointWorldSpace, Vector3(0, -1, 0));
			RayCollision closestCollision;
			world->Raycast(ray, closestCollision, true, this);

			if (closestCollision.rayDistance > 3.0f) {
				//SolveIK( closestCollision.collidedAt, jointChain.first, jointChain.second);
			} else {
				//ResetIK(jointChain.first, jointChain.second);
			}
		}
	}
}

void AnimatedObject::DrawSkeleton() {
	vector<int> parents = renderObject->GetMesh()->GetJointParents();
	const unsigned int frame = animCon->GetCurrentFrame();
	const MeshAnimation* currentAnim = animCon->GetCurrentAnimation();
	const vector<Matrix4> bindPose = renderObject->GetMesh()->GetBindPose();
	const vector<Matrix4> invBindPose = renderObject->GetMesh()->GetInverseBindPose();
	const Matrix4 modelMat = GetTransform().GetMatrix();

	for (int i = 0; i < 54; i++) {
		unsigned int joint = i;

		while (parents.at(joint) != -1) {
			unsigned int parent = parents.at(joint);
			
			Vector3 jointWorldSpace = (modelMat * bindPose.at(joint) * invBindPose.at(parent)).GetPositionVector();

			Matrix4 parentWorldMatrix = modelMat * bindPose.at(parent);
			//if (parent != 0) parentWorldMatrix = parentWorldMatrix * invBindPose.at(parents.at(parent));
			Vector3 parentWorldSpace = parentWorldMatrix.GetPositionVector();

			Debug::DrawLine(jointWorldSpace, parentWorldSpace, { 0, 0, 1, 1 }, 5);

			joint = parent;
		}	
	}
}