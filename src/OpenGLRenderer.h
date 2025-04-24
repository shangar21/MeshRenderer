#pragma once

#include "Camera.h"
#include "Hit.h"
#include "Mesh.h"
#include "Ray.h"
#include <opencv2/opencv.hpp>

class OpenGLRenderer {
	public:
		void initWindow();
		void renderMesh(Mesh &mesh);
};
