#pragma once

#include "StateMachine.h"

namespace NCL {
	class MeshAnimation;

	namespace CSC8503 {
		class AnimatedObject;

		class AnimationController : public StateMachine {
		public:
			AnimationController(AnimatedObject* object, const std::unordered_map<std::string, MeshAnimation*>& animMap);
			~AnimationController();

			void Update(float dt);

			const MeshAnimation* GetCurrentAnimation() const {
				return curAnim;
			}

			const int GetCurrentFrame() const {
				return curFrame;
			}

		private:
			MeshAnimation* curAnim;
			AnimatedObject* animObj;
			std::unordered_map<std::string, MeshAnimation*> animations;
			int curFrame;
			float frameTime;
		};
	}
}