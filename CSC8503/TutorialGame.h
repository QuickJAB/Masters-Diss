#pragma once
#include "GameTechRenderer.h"
#ifdef USEVULKAN
#include "GameTechVulkanRenderer.h"
#endif
#include "PhysicsSystem.h"

#include "StateGameObject.h"

namespace NCL {
	namespace CSC8503 {
		class TutorialGame {
		public:
			TutorialGame();
			~TutorialGame();

			virtual void UpdateGame(float dt);

		protected:
			void InitialiseAssets();

			void InitCamera();
			void UpdateKeys();

			void InitWorld();

			void AddFloorToWorld(const Vector3& position);
			GameObject* AddCubeToWorld(const Vector3& position, const Vector3& dimensions);
			void AddRampToWorld(const Vector3& position, const Vector2& dimensions, const Vector2& orientation);
			void AddStairsToWorld(const Vector3& position, const Vector3& dimensions, const int& numSteps);

			GameObject* AddPlayerToWorld(const Vector3& position);

#ifdef USEVULKAN
			GameTechVulkanRenderer*	renderer;
#else
			GameTechRenderer* renderer;
#endif
			PhysicsSystem*		physics;
			GameWorld*			world;

			MeshGeometry*	cubeMesh	= nullptr;

			TextureBase*	basicTex	= nullptr;
			ShaderBase*		basicShader = nullptr;

			GameObject* lockedObject	= nullptr;
			Vector3 lockedOffset		= Vector3(0, 7, 20);

			bool freeRotation;
		};
	}
}

