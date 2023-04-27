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

void AnimatedObject::SolveIK(const Vector3& snapPoint, unsigned int currentJoint, const unsigned int& endJoint) {
	const vector<Matrix4> bindPose = renderObject->GetMesh()->GetBindPose();
	MeshAnimation* curAnim = (MeshAnimation*)animCon->GetCurrentAnimation();
	unsigned int frame = animCon->GetCurrentFrame();
	const Matrix4 modelMat = GetTransform().GetMatrix() * Matrix4::Rotation(180, Vector3(0, 1, 0));
	vector<int> parents = renderObject->GetMesh()->GetJointParents();

	Vector3 adjustment = bindPose.at(currentJoint).GetPositionVector() - snapPoint;

	while (currentJoint != endJoint) {
		//Matrix4 jointWorldSpace = modelMat * bindPose.at(currentJoint);

		//curAnim->SetJointValue(frame, currentJoint, Matrix4::Translation(adjustment) * jointWorldSpace);
		//curAnim->SetJointValue(frame, currentJoint, modelMat.Inverse() * jointWorldSpace);

		Matrix4 jointWorldSpace = curAnim->GetJoint(frame, currentJoint);

		std::cout << jointWorldSpace;
		std::cout << (Matrix4::Translation(snapPoint) * modelMat.Inverse()) << std::endl;

		//jointWorldSpace.SetPositionVector(snapPoint);

		curAnim->SetJointValue(frame, currentJoint, jointWorldSpace);

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

			Debug::DrawLine(jointWorldSpace, closestCollision.collidedAt, { 0, 0, 1, 1 }, 0.1f);

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
	const Matrix4 modelMat = GetTransform().GetMatrix() * Matrix4::Rotation(180, Vector3(0, 1, 0));

	for (int i = 0; i < 54; i++) {
		DisplayJointData(i);
		if (parents.at(i) == -1) continue;
		Vector3 jointPos = (modelMat * bindPose.at(i)).GetPositionVector();
		Matrix4 parentMat = bindPose.at(parents.at(i));
		Debug::DrawLine(jointPos, (modelMat * parentMat).GetPositionVector(), {0, 0, 1, 1}, 5);
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