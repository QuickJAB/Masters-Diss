#include "RenderObject.h"
#include "MeshGeometry.h"
#include "MeshAnimation.h"
#include "../CSC8503/AnimationController.h"

using namespace NCL;
using namespace CSC8503;

RenderObject::RenderObject(Transform* parentTransform, MeshGeometry* mesh, TextureBase* tex, ShaderBase* shader) {
	this->transform	= parentTransform;
	this->mesh		= mesh;
	this->texture	= tex;
	this->shader	= shader;
	this->colour	= Vector4(1.0f, 1.0f, 1.0f, 1.0f);
	
	this->rigged	= false;
	this->animCon	= nullptr;
}

RenderObject::~RenderObject() {

}

void RenderObject::GetFrameMatrices(std::vector<Matrix4>& frameMatrices) const {
	if (animCon == nullptr) return;
	
	const std::vector<Matrix4> invBindPose = mesh->GetInverseBindPose();
	if (animCon->GetCurrentAnimation()) {
		const Matrix4* frameData = animCon->GetCurrentAnimation()->GetJointData(animCon->GetCurrentFrame());
		for (unsigned int i = 0; i < mesh->GetJointCount(); ++i) {
			frameMatrices.emplace_back(frameData[i] * invBindPose[i]);
		}
	}
}