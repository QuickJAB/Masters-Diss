/*
Part of Newcastle University's Game Engineering source code.

Use as you see fit!

Comments and queries to: richard-gordon.davison AT ncl.ac.uk
https://research.ncl.ac.uk/game/
*/
#pragma once
#include <vector>
#include <string>

namespace NCL {
	namespace Maths {
		class Matrix4;
	}

	class MeshAnimation	{
	public:
		MeshAnimation();

		MeshAnimation(unsigned int jointCount, unsigned int frameCount, float frameRate, std::vector<Maths::Matrix4>& frames);

		MeshAnimation(const std::string& filename);
		virtual ~MeshAnimation();

		unsigned int GetJointCount() const {
			return jointCount;
		}

		unsigned int GetFrameCount() const {
			return frameCount;
		}

		float GetFrameRate() const {
			return frameRate;
		}

		const Maths::Matrix4* GetJointData(unsigned int frame) const;
		const Maths::Matrix4 GetJoint(unsigned int frame, unsigned int id) const;

		void SetJointValue(unsigned int frame, unsigned int joint, Maths::Matrix4 value);
		void ResetJointValue(unsigned int frame, unsigned int joint);
		const Maths::Matrix4 GetJointOffset(unsigned int frame, unsigned int jointA, unsigned int jointB);

		void FixRootPosition(unsigned int frame, std::vector<int>);

	protected:
		unsigned int	jointCount;
		unsigned int	frameCount;
		float			frameRate;

		std::vector<Maths::Matrix4>		allJoints;
		std::vector<Maths::Matrix4>		originalJoints;
	};
}

