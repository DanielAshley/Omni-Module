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
		bool checkButton1();
		bool checkButton2();
		int isInitialized();
	private:
		HHD hHD;
		int initialized;
	};
};

#endif;