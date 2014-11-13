#ifndef OMNI_LIB_H
#define OMNI_LIB_H

#include <vector>

#pragma once
namespace WMRA{
	class Pose;
	typedef unsigned int HHD;
};

namespace WMRA{
	class omni{
	public:
		omni();
		~omni();
		WMRA::Pose getDeltaPose();
		static void omniThread(void *aArg);
		HHD hHD;

	private:
		double gain;
	};
};

#endif;