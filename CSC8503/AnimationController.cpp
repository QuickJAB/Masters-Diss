#include "AnimationController.h"
#include "AnimatedObject.h"
#include "../NCLCoreClasses/MeshAnimation.h"
#include "PhysicsObject.h"
#include "State.h"
#include "StateTransition.h"

using namespace NCL;
using namespace CSC8503;

AnimationController::AnimationController(AnimatedObject* object, const std::unordered_map<std::string, MeshAnimation*>& animMap) {
	animObj = object;
	animations = animMap;
	curFrame = 0;
	frameTime = 0.0f;

	State* idle = new State([&](float dt)->void {
		curAnim = animations.at("idle");
	});
	State* run = new State([&](float dt)->void {
		curAnim = animations.at("run");
	});

	StateTransition* idleToRun = new StateTransition(idle, run, [&](void)->bool {
		if (this->animObj->GetPhysicsObject()->GetLinearVelocity().Length() > .1f) {
			curFrame = 0;
			this->animObj->SetMoving(true);
			return true;
		}
		return false;
	});

	StateTransition* runToIdle = new StateTransition(run, idle, [&](void)->bool {
		if (this->animObj->GetPhysicsObject()->GetLinearVelocity().Length() < .1f) {
			curFrame = 0;
			animObj->SetMoving(false);
			return true;
		}
		return false;
	});

	AddState(idle);
	AddState(run);
	AddTransition(idleToRun);
	AddTransition(runToIdle);
}

AnimationController::~AnimationController() {

}

void AnimationController::Update(float dt) {
	StateMachine::Update(dt);

	frameTime -= dt;
	while (frameTime < 0.0f) {
		curFrame = (curFrame + 1) % curAnim->GetFrameCount();
		frameTime += 1.0f / curAnim->GetFrameRate();
	}
}