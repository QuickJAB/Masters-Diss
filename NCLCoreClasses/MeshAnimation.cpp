#include "MeshAnimation.h"
#include "Matrix4.h"
#include "Vector3.h"
#include "Assets.h"

#include <fstream>
#include <string>

using namespace NCL;
using namespace NCL::Maths;

MeshAnimation::MeshAnimation() {
	jointCount	= 0;
	frameCount	= 0;
	frameRate	= 0.0f;
}

MeshAnimation::MeshAnimation(unsigned int jointCount, unsigned int frameCount, float frameRate, std::vector<Matrix4>& frames) {
	this->jointCount = jointCount;
	this->frameCount = frameCount;
	this->frameRate  = frameRate;
	this->allJoints  = frames;
	this->originalJoints  = frames;
}

MeshAnimation::MeshAnimation(const std::string& filename) : MeshAnimation() {
	std::ifstream file(Assets::MESHDIR + filename);

	std::string filetype;
	int fileVersion;

	file >> filetype;

	if (filetype != "MeshAnim") {
		std::cout << __FUNCTION__ << " File is not a MeshAnim file!\n";
		return;
	}
	file >> fileVersion;
	file >> frameCount;
	file >> jointCount;
	file >> frameRate;

	allJoints.reserve((size_t)frameCount * jointCount);

	for (unsigned int frame = 0; frame < frameCount; ++frame) {
		for (unsigned int joint = 0; joint < jointCount; ++joint) {
			Matrix4 mat;
			for (int i = 0; i < 4; ++i) {
				for (int j = 0; j < 4; ++j) {
					file >> mat.array[i][j];
				}
			}
			allJoints.emplace_back(mat);
			originalJoints.emplace_back(mat);
		}
	}
}

MeshAnimation::~MeshAnimation() {

}

const Matrix4* MeshAnimation::GetJointData(unsigned int frame) const {
	if (frame >= frameCount) {
		return nullptr;
	}
	int matStart = frame * jointCount;

	Matrix4* dataStart = (Matrix4*)allJoints.data();

	return dataStart + matStart;
}

const Matrix4 MeshAnimation::GetJoint(unsigned int frame, unsigned int id) const {
	if (frame >= frameCount) {
		return nullptr;
	}
	int matStart = frame * jointCount;

	return (allJoints.data())[matStart + id];
}

void MeshAnimation::SetJointValue(unsigned int frame, unsigned int joint, Matrix4 value) {
	if (frame >= frameCount) return;

	int poseJoint = (frame * jointCount) + joint;
	allJoints.at(poseJoint) = value;
}

void MeshAnimation::ResetJointValue(unsigned int frame, unsigned int joint) {
	if (frame >= frameCount) return;

	int poseJoint = (frame * jointCount) + joint;
	allJoints.at(poseJoint) = originalJoints.at(poseJoint);
}

const Matrix4 MeshAnimation::GetJointOffset(unsigned int frame, unsigned int jointA, unsigned int jointB) {
	if (frame >= frameCount) return Matrix4();
	int poseJointA = (frame * jointCount) + jointA;
	int poseJointB = (frame * jointCount) + jointB;

	return originalJoints.at(poseJointB) - originalJoints.at(poseJointA);
}

void MeshAnimation::FixRootPosition(unsigned int frame, std::vector<int> parents) {
	if (frame >= frameCount) return;

	int joint = (frame * jointCount);

	Vector3 posOffset = originalJoints.at(joint).GetPositionVector() - originalJoints.at(0).GetPositionVector();
	Matrix4 matOffset = Matrix4::Translation({ 0, 0, posOffset.z });

	for (int i = 0; i < jointCount; i++) {
		allJoints.at(joint + i) = matOffset.Inverse() * originalJoints.at(joint + i);
	}
	
}