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
			bool performedIK = false;

			std::vector<unsigned int> effectorJoints;

			MeshAnimation* curAnim;

			void SolveIK(const Vector3& snapPoint, int currentJoint, const unsigned int& chainId, const float& degrees);
			void DrawSkeleton();
			void DisplayJointData(unsigned int joint);
			void AdjustJoint(const int& joint, Vector3& offset, const bool& hasRotation = false, const Matrix4& rotation = Matrix4());
		};
	}
}