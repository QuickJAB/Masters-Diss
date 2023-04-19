#pragma once
#include "GameObject.h"

namespace NCL {
	namespace CSC8503 {
		class GameTechRenderer;
		class AnimationController;

		class AnimatedObject : public GameObject {
		public:
			AnimatedObject(const Vector3& position, GameTechRenderer* renderer);
			~AnimatedObject();

			virtual void Update(float dt);

			void SetMoving(bool value) {
				isMoving = value;
			}

			void SolveIK(const float& rayDist, const Vector3& snapPoint, unsigned int currentJoint, const unsigned int& endJoint);

		private:
			AnimationController* animCon;
			bool isMoving = false;

			std::unordered_map<unsigned int, unsigned int> effectorJointChain;
		};
	}
}