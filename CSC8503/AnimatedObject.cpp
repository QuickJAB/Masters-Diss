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
	effectorJointChain.emplace(51, 0);
	
	// Left leg end joints
	effectorJointChain.emplace(47, 0);

	animCon = new AnimationController(this, animations);

	float radius = 2.0f;
	float inverseMass = 0.9f;

	SphereVolume* volume = new SphereVolume(radius);
	SetBoundingVolume((CollisionVolume*)volume);

	GetTransform()
		.SetScale(Vector3(radius, radius, radius))
		.SetPosition(position)
		.SetColOffset(Vector3(0, radius, 0));

	SetRenderObject(new RenderObject(&GetTransform(), renderer->LoadMesh("DummyMesh.msh"), nullptr, renderer->LoadShader("skinning.vert", "character.frag")));
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

void AnimatedObject::SolveIK(const Vector3& snapPoint, int currentJoint, const unsigned int& endJoint) {
	MeshAnimation* curAnim = (MeshAnimation*)animCon->GetCurrentAnimation();
	unsigned int frame = animCon->GetCurrentFrame();
	const Matrix4 modelMat = GetTransform().GetMatrix();
	vector<int> parents = renderObject->GetMesh()->GetJointParents();

	Vector3 offset = modelMat.Inverse() * snapPoint;

	vector<bool> adjustedJoints;
	for (int i = 0; i < (int)parents.size(); i++) {
		adjustedJoints.push_back(false);
	}

	while (currentJoint != parents.at(endJoint)) {
		Matrix4 joint = curAnim->GetJoint(frame, currentJoint);

		joint.SetPositionVector(offset);

		curAnim->SetJointValue(frame, currentJoint, joint);

		if (parents.at(currentJoint) != -1) 
			offset += curAnim->GetJointOffset(frame, currentJoint, parents.at(currentJoint));

		adjustedJoints.at(currentJoint) = true;

		currentJoint = parents.at(currentJoint);
	}

	

	/*
	Vector3 rootPos = curAnim->GetJoint(frame, 0).GetPositionVector();
	
	for (int i = 0; i < (int)parents.size(); i++) {
		if (adjustedJoints.at(i)) continue;

		Vector3 offset = rootPos;
		int jointId = i;
		while (parents.at(jointId) != -1) {
			offset += curAnim->GetJointOffset(frame, jointId, parents.at(jointId));
			jointId = parents.at(jointId);
		}

		Matrix4 joint = curAnim->GetJoint(frame, i);
		joint.SetPositionVector(offset);
		curAnim->SetJointValue(frame, i, joint);
		adjustedJoints.at(i) = true;
	}
	*/
}

void AnimatedObject::ResetIK() {
	vector<int> parents = renderObject->GetMesh()->GetJointParents();
	unsigned int curFrame = animCon->GetCurrentFrame();

	for (int i = 0; i < parents.size(); i++) {
		((MeshAnimation*)animCon->GetCurrentAnimation())->ResetJointValue(curFrame, i);
	}

}

void AnimatedObject::Update(float dt) {
	animCon->Update(dt);

	vector<int> parents = renderObject->GetMesh()->GetJointParents();
	const Matrix4 modelMat = GetTransform().GetMatrix();
	MeshAnimation* curAnim = (MeshAnimation*)animCon->GetCurrentAnimation();
	unsigned int frame = animCon->GetCurrentFrame();

	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::Q))
		DrawSkeleton();
			
	if (!isMoving && GetPhysicsObject()->GetLinearVelocity().y >= -.1f && world != nullptr) {
		bool reset = true;
		for (auto& jointChain : effectorJointChain) {
			Vector3 jointWorldSpace = (modelMat * curAnim->GetJoint(frame, jointChain.first, true)).GetPositionVector();

			Ray ray = Ray(jointWorldSpace, Vector3(0, -1, 0));
			RayCollision closestCollision;
			world->Raycast(ray, closestCollision, true, this);

			Debug::DrawLine(jointWorldSpace, closestCollision.collidedAt, { 0, 0, 1, 1 }, 0.1f);

			if (closestCollision.rayDistance > 0.3f && closestCollision.rayDistance < 1.0f) {
				SolveIK( closestCollision.collidedAt, jointChain.first, jointChain.second);
				reset = false;
			}
		}
		if (reset) {
			ResetIK();
		}
	}
	else {
		((MeshAnimation*)animCon->GetCurrentAnimation())->FixRootPosition(animCon->GetCurrentFrame(), parents);
	}
}

void AnimatedObject::DrawSkeleton() {
	const vector<int> parents = renderObject->GetMesh()->GetJointParents();
	const Matrix4 modelMat = GetTransform().GetMatrix();
	unsigned int frame = animCon->GetCurrentFrame();
	MeshAnimation* curAnim = (MeshAnimation*)animCon->GetCurrentAnimation();

	for (int i = 0; i < parents.size(); i++) {
		DisplayJointData(i);
		if (parents.at(i) == -1) continue;
		
		Vector3 jointPos = (modelMat * curAnim->GetJoint(frame, i)).GetPositionVector();
		Vector3 parentPos = (modelMat * curAnim->GetJoint(frame, parents.at(i))).GetPositionVector();
		
		Debug::DrawLine(jointPos, parentPos, {0, 0, 1, 1}, 5);
	}
}

void AnimatedObject::DisplayJointData(unsigned int joint) {
	const vector<int> parents = renderObject->GetMesh()->GetJointParents();
	const MeshGeometry* mesh = renderObject->GetMesh();
	
	std::cout << joint << ": " << mesh->GetJointName(joint) << std::endl;
	std::cout << "Parents: ";
	while (parents.at(joint) != -1) {
		std::cout << parents.at(joint) << ", ";
		joint = parents.at(joint);
	}
	std::cout << std::endl;
}