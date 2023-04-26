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
	effectorJointChain.emplace(52, 49);
	
	// Left leg end joints
	effectorJointChain.emplace(48, 45);

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

void AnimatedObject::SolveIK(const Vector3& snapPoint, unsigned int currentJoint, const unsigned int& endJoint) {
	vector<int> parents = renderObject->GetMesh()->GetJointParents();
	unsigned int curFrame = animCon->GetCurrentFrame();
	unsigned int previousJoint = 999;
	
	while (currentJoint != endJoint) {
		Matrix4 jointOffset = ((MeshAnimation*)animCon->GetCurrentAnimation())->GetJointOffset(curFrame, previousJoint, currentJoint);
		Matrix4 position = Matrix4::Translation(snapPoint - GetTransform().GetPosition()) - jointOffset;

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

	if (!isMoving && GetPhysicsObject()->GetLinearVelocity().y >= -.1f && world != nullptr) {
		for (auto& jointChain : effectorJointChain) {
			Vector3 jointWorldSpace = (modelMat * bindPose.at(jointChain.first)).GetPositionVector();

			Ray ray = Ray(jointWorldSpace, Vector3(0, -1, 0));
			RayCollision closestCollision;
			world->Raycast(ray, closestCollision, true, this);

			if (closestCollision.rayDistance > 1.0f) {
				SolveIK( closestCollision.collidedAt, jointChain.first, jointChain.second);
			} else {
				ResetIK(jointChain.first, jointChain.second);
			}
		}
	}
	else {
		((MeshAnimation*)animCon->GetCurrentAnimation())->FixRootPosition(animCon->GetCurrentFrame(), parents);
	}
}

void AnimatedObject::DrawSkeleton() {
	const vector<int> parents = renderObject->GetMesh()->GetJointParents();
	const vector<Matrix4> bindPose = renderObject->GetMesh()->GetBindPose();
	const Matrix4 modelMat = GetTransform().GetMatrix();
	const MeshGeometry* mesh = renderObject->GetMesh();

	for (int i = 0; i < 54; i++) {
		if (parents.at(i) == -1) continue;
		Vector3 jointPos = (modelMat * bindPose.at(i)).GetPositionVector();
		Matrix4 parentMat = bindPose.at(parents.at(i));
		Debug::DrawLine(jointPos, (modelMat * parentMat).GetPositionVector(), {0, 0, 1, 1}, 5);

		std::cout << i << ":\t" << mesh->GetJointName(i) << std::endl;
	}
}