#pragma once
#include "GameObject.h"

namespace NCL {
	class MeshAnimation;

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

		private:
			AnimationController* animCon;
			bool isMoving = false;

			std::vector<unsigned int> effectorJoints;

			std::vector<bool> adjusted;
			MeshAnimation* curAnim;

			void SolveIK(const Vector3& snapPoint, int currentJoint);
			void AdjustJointChain(const std::vector<int> jointChain, const int& endJoint, const unsigned int& frame, const Matrix4& modelMat);
			void ResetIK();
			void DrawSkeleton();
			void DisplayJointData(unsigned int joint);
		};
	}
}