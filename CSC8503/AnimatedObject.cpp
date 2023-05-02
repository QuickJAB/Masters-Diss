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
	curAnim = animations.at("idle");
	
	effectorJoints.push_back(52);	// Right toe
	effectorJoints.push_back(48);	// Left toe
	effectorJoints.push_back(6);	// Jaw
	effectorJoints.push_back(10);	// Left hand
	effectorJoints.push_back(29);	// Right hand
	effectorJoints.push_back(52);	// Right toe
	effectorJoints.push_back(48);	// Left toe

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

void AnimatedObject::SolveIK(const Vector3& snapPoint, int currentJoint, const unsigned int& chainId) {
	vector<int> parents = renderObject->GetMesh()->GetJointParents();
	
	for (int i = 0; i < (int)parents.size(); i++) {
		adjusted.push_back(false);
	}

	unsigned int frame = animCon->GetCurrentFrame();
	const Matrix4 modelMat = GetTransform().GetMatrix();

	Vector3 offset = modelMat.Inverse() * snapPoint;

	while (currentJoint != -1) {
		Matrix4 joint = curAnim->GetJoint(frame, currentJoint);

		joint.SetPositionVector(offset);

		curAnim->SetJointValue(frame, currentJoint, joint);

		if (parents.at(currentJoint) != -1)
			offset += curAnim->GetJointOffset(frame, currentJoint, parents.at(currentJoint));

		adjusted.at(currentJoint) = true;

		currentJoint = parents.at(currentJoint);
	}

	// IK on other leg
	unsigned int altFoot = chainId == 0 ? effectorJoints.at(1) : effectorJoints.at(0);
	currentJoint = parents.at(parents.at(parents.at(altFoot)));
	Matrix4 rotation = Matrix4::Rotation(70, Vector3(1, 0, 0));
	Matrix4 joint = rotation * curAnim->GetJoint(frame, currentJoint, true);
	offset = curAnim->GetJoint(frame, parents.at(currentJoint)).GetPositionVector() + curAnim->GetJointOffset(frame, parents.at(currentJoint), currentJoint);
	joint.SetPositionVector(offset);
	curAnim->SetJointValue(frame, currentJoint, joint);
	adjusted.at(currentJoint) = true;

	currentJoint = parents.at(parents.at(altFoot));
	offset += rotation * curAnim->GetJointOffset(frame, parents.at(currentJoint), currentJoint);
	joint.SetPositionVector(offset);
	joint = rotation.Inverse() * joint;
	curAnim->SetJointValue(frame, currentJoint, joint);
	adjusted.at(currentJoint) = true;

	currentJoint = parents.at(altFoot);
	offset += curAnim->GetJointOffset(frame, parents.at(currentJoint), currentJoint);
	joint.SetPositionVector(offset);
	curAnim->SetJointValue(frame, currentJoint, joint);
	adjusted.at(currentJoint) = true;

	currentJoint = altFoot;
	offset += curAnim->GetJointOffset(frame, parents.at(currentJoint), currentJoint);
	joint.SetPositionVector(offset);
	curAnim->SetJointValue(frame, currentJoint, joint);
	adjusted.at(currentJoint) = true;

	// IK on rest of body
	//for (int i = 2; i < effectorJoints.size(); i++) {
	//	currentJoint = effectorJoints.at(i);
	//	vector<int> jointChain;
	//	while (!adjusted.at(currentJoint)) {
	//		jointChain.insert(jointChain.begin(), currentJoint);
	//		currentJoint = parents.at(currentJoint);
	//	}

	//	AdjustJointChain(jointChain, currentJoint, frame, modelMat);
	//}
}

void AnimatedObject::AdjustJointChain(vector<int> jointChain, const int& endJoint, const unsigned int& frame, const Matrix4& modelMat) {
	vector<int> parents = renderObject->GetMesh()->GetJointParents();

	Vector3 offset = curAnim->GetJoint(frame, endJoint).GetPositionVector();

	for (int i = 0; i < jointChain.size(); i++) {
		unsigned int currentJoint = jointChain.at(i);

		Matrix4 joint = curAnim->GetJoint(frame, currentJoint);

		offset += curAnim->GetJointOffset(frame, currentJoint, parents.at(currentJoint));
		
		joint.SetPositionVector(offset);
		
		curAnim->SetJointValue(frame, currentJoint, joint);

		adjusted.at(currentJoint) = true;
	}
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
	unsigned int frame = animCon->GetCurrentFrame();

	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::Q))
		DrawSkeleton();
			
	if (!isMoving && GetPhysicsObject()->GetLinearVelocity().y >= -.1f && world != nullptr) {
		bool reset = true;
		for (unsigned int i = 0; i < 2; i++) {
			unsigned int effector = effectorJoints.at(i);

			Vector3 jointWorldSpace = (modelMat * curAnim->GetJoint(frame, effector, true)).GetPositionVector();

			Ray ray = Ray(jointWorldSpace, Vector3(0, -1, 0));
			RayCollision closestCollision;
			world->Raycast(ray, closestCollision, true, this);

			Debug::DrawLine(jointWorldSpace, closestCollision.collidedAt, { 0, 0, 1, 1 }, 0.1f);

			if (closestCollision.rayDistance > 0.3f && closestCollision.rayDistance < 1.0f) {
				SolveIK(closestCollision.collidedAt, effector, i);
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
	vector<int> parents = renderObject->GetMesh()->GetJointParents();

	const Matrix4 modelMat = GetTransform().GetMatrix();
	unsigned int frame = animCon->GetCurrentFrame();

	for (int i = 0; i < parents.size(); i++) {
		DisplayJointData(i);
		if (parents.at(i) == -1) continue;
		
		Vector3 jointPos = (modelMat * curAnim->GetJoint(frame, i)).GetPositionVector();
		Vector3 parentPos = (modelMat * curAnim->GetJoint(frame, parents.at(i))).GetPositionVector();
		
		Debug::DrawLine(jointPos, parentPos, {0, 0, 1, 1}, 5);
	}
}

void AnimatedObject::DisplayJointData(unsigned int joint) {
	const MeshGeometry* mesh = renderObject->GetMesh();
	vector<int> parents = mesh->GetJointParents();
	
	std::cout << joint << ": " << mesh->GetJointName(joint) << std::endl;
	std::cout << "Parents: ";
	while (parents.at(joint) != -1) {
		std::cout << parents.at(joint) << ", ";
		joint = parents.at(joint);
	}
	std::cout << std::endl;
}