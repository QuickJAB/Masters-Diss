#include "TutorialGame.h"
#include "GameWorld.h"
#include "PhysicsObject.h"
#include "RenderObject.h"
#include "TextureLoader.h"

#include "PositionConstraint.h"
#include "OrientationConstraint.h"
#include "StateGameObject.h"
#include "AnimatedObject.h"

using namespace NCL;
using namespace CSC8503;

TutorialGame::TutorialGame()	{
	world		= new GameWorld();
#ifdef USEVULKAN
	renderer	= new GameTechVulkanRenderer(*world);
#else 
	renderer = new GameTechRenderer(*world);
#endif

	physics		= new PhysicsSystem(*world);

	freeRotation = false;

	InitialiseAssets();
}

void TutorialGame::InitialiseAssets() {
	cubeMesh	= renderer->LoadMesh("cube.msh");

	basicTex	= renderer->LoadTexture("checkerboard.png");
	basicShader = renderer->LoadShader("scene.vert", "scene.frag");

	InitCamera();
	InitWorld();
}

TutorialGame::~TutorialGame()	{
	delete cubeMesh;

	delete basicTex;
	delete basicShader;

	delete physics;
	delete renderer;
	delete world;
}

void TutorialGame::UpdateGame(float dt) {
	if (lockedObject != nullptr) {
		Vector3 objPos = lockedObject->GetTransform().GetPosition();	
		
		Vector3 camPos = Vector3();

		if (freeRotation) {
			float yaw = -(Window::GetMouse()->GetRelativePosition().x);

			if (yaw < 0) {
				yaw += 360.0f;
			}
			if (yaw > 360.0f) {
				yaw -= 360.0f;
			}
			
			camPos = (Matrix4::Rotation(yaw, Vector3(0, 1, 0)) * Matrix4::Translation(lockedOffset)).GetPositionVector();
		}
		else {
			float objYaw = lockedObject->GetTransform().GetOrientation().ToEuler().y;
			camPos = (Matrix4::Rotation(objYaw, Vector3(0, 1, 0)) * Matrix4::Translation(lockedOffset)).GetPositionVector();
		}	
		
		camPos += objPos;

		Matrix4 temp = Matrix4::BuildViewMatrix(camPos, objPos, Vector3(0, 1, 0));

		Matrix4 modelMat = temp.Inverse();

		Quaternion q(modelMat);
		Vector3 angles = q.ToEuler();

		world->GetMainCamera()->SetPosition(camPos);
		world->GetMainCamera()->SetYaw(angles.y);

		UpdateKeys();
	}

	lockedObject->Update(dt);

	world->UpdateWorld(dt);
	renderer->Update(dt);
	physics->Update(dt);

	renderer->Render();
	Debug::UpdateRenderables(dt);
}

void TutorialGame::UpdateKeys() {
	freeRotation = Window::GetKeyboard()->KeyDown(KeyboardKeys::SHIFT);

	Camera* cam = world->GetMainCamera();

	float yaw = -(Window::GetMouse()->GetRelativePosition().x);

	if (yaw < 0) {
		yaw += 360.0f;
	}
	if (yaw > 360.0f) {
		yaw -= 360.0f;
	}

	lockedObject->GetTransform().SetOrientation(lockedObject->GetTransform().GetOrientation() * Matrix4::Rotation(yaw, Vector3(0, 1, 0)));

	Matrix4 view = cam->BuildViewMatrix();
	Matrix4 camWorld = view.Inverse();

	Vector3 rightAxis = Vector3(camWorld.GetColumn(0));

	Vector3 fwdAxis = Vector3::Cross(Vector3(0, 1, 0), rightAxis);
	fwdAxis.y = 0.0f;
	fwdAxis.Normalise();

	PhysicsObject* physObj = lockedObject->GetPhysicsObject();

	const float SPEED = 30.0f;

	Vector3 direction = (fwdAxis * Window::GetKeyboard()->KeyDown(KeyboardKeys::W)) +
		(-fwdAxis * Window::GetKeyboard()->KeyDown(KeyboardKeys::S)) +
		(-rightAxis * Window::GetKeyboard()->KeyDown(KeyboardKeys::A)) +
		(rightAxis * Window::GetKeyboard()->KeyDown(KeyboardKeys::D));

	physObj->AddForce(direction * SPEED);

	if (direction == Vector3()) {
		physObj->SetLinearVelocity({ 0, physObj->GetLinearVelocity().y, 0 });
	}
}

void TutorialGame::InitCamera() {
	world->GetMainCamera()->SetNearPlane(0.1f);
	world->GetMainCamera()->SetFarPlane(500.0f);
	world->GetMainCamera()->SetPitch(-15.0f);
	world->GetMainCamera()->SetYaw(315.0f);
	world->GetMainCamera()->SetPosition(Vector3(-60, 40, 60));
	lockedObject = nullptr;
}

void TutorialGame::InitWorld() {
	world->ClearAndErase();
	physics->Clear();

	AddFloorToWorld({ 0, 0, 0 });

	//AddRampToWorld({ 10, 3, 10 }, {5, 10}, {10, 0});
	//AddRampToWorld({ 20, 3, 10 }, {5, 10}, {20, 0});
	//AddRampToWorld({ 30, 3, 10 }, {5, 10}, {30, 0});

	AddStairsToWorld({ 0, 0, 5 }, { 5, 3, 15 }, 10);

	AddStairsToWorld({ -15, 0, 5 }, { 5, 1, 15 }, 20);
	AddStairsToWorld({ -30, 0, 5 }, { 5, 3, 15 }, 20);
	AddStairsToWorld({ -45, 0, 5 }, { 5, 5, 15 }, 20);

	AddStairsToWorld({ 15, 0, 5 }, { 5, 2, 10 }, 20);
	AddStairsToWorld({ 30, 0, 5 }, { 5, 2, 20 }, 20);
	AddStairsToWorld({ 45, 0, 5 }, { 5, 2, 30 }, 20);

	lockedObject = AddPlayerToWorld({ 0, 5, 0 });
}

void TutorialGame::AddFloorToWorld(const Vector3& position) {
	GameObject* floor = new GameObject();

	Vector3 floorSize = Vector3(200, 2, 200);
	AABBVolume* volume = new AABBVolume(floorSize);
	floor->SetBoundingVolume((CollisionVolume*)volume);
	floor->GetTransform()
		.SetScale(floorSize * 2)
		.SetPosition(position);

	floor->SetRenderObject(new RenderObject(&floor->GetTransform(), cubeMesh, basicTex, basicShader));

	world->AddGameObject(floor);
}

GameObject* TutorialGame::AddCubeToWorld(const Vector3& position, const Vector3& dimensions) {
	GameObject* cube = new GameObject();

	AABBVolume* volume = new AABBVolume(dimensions);
	cube->SetBoundingVolume((CollisionVolume*)volume);

	cube->GetTransform()
		.SetPosition(position)
		.SetScale(dimensions * 2);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));

	world->AddGameObject(cube);

	return cube;
}

GameObject* TutorialGame::AddPlayerToWorld(const Vector3& position) {
	AnimatedObject* character = new AnimatedObject(position, renderer);
	character->SetGameWorld(world);

	world->AddGameObject(character);

	return character;
}

void TutorialGame::AddRampToWorld(const Vector3& position, const Vector2& dimensions, const Vector2& orientation) {
	GameObject* cube = new GameObject();

	Vector3 scale = Vector3(dimensions.x, 0.1f, dimensions.y);

	OBBVolume* volume = new OBBVolume(scale);
	cube->SetBoundingVolume((CollisionVolume*)volume);

	cube->GetTransform()
		.SetPosition(position)
		.SetScale(scale * 2)
		.SetOrientation(Quaternion::EulerAnglesToQuaternion(orientation.x, 0.0f, orientation.y));

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));

	world->AddGameObject(cube);
}

void TutorialGame::AddStairsToWorld(const Vector3& position, const Vector3& dimensions, const int& numSteps) {
	float stepHeight = dimensions.y / numSteps;
	float stepDepth = dimensions.z / numSteps;
	
	Vector3 stepDim = Vector3(dimensions.x, stepHeight, stepDepth);

	for (int i = 0; i < numSteps; i++) {
		Vector3 pos = Vector3(position.x, position.y + (i * stepHeight * 2), position.z + (i * stepDepth * 2));
		AddCubeToWorld(pos, stepDim);
	}
}