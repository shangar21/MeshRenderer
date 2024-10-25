#include "Camera.h"
#include "Mesh.h"
#include "Renderer.h"

int main() {
  // Create a Camera
  Camera camera(Eigen::Vector3f(0.0f, 0.0f, 1.0f),   // Camera position
                Eigen::Vector3f(0.0f, 0.0f, 0.0f), // Target point
                Eigen::Vector3f(0.0f, 1.0f, 0.0f),   // Up vector
                1.0f, 800, 600);
  Mesh mesh;
  bool loaded = mesh.loadFromObj("/home/shangar21/Downloads/teapot.obj");
  mesh.printMeshInfo();

  // Create R, G, B matrices to store the image colors
  Eigen::MatrixXf R(camera.imageHeight, camera.imageWidth);
  Eigen::MatrixXf G(camera.imageHeight, camera.imageWidth);
  Eigen::MatrixXf B(camera.imageHeight, camera.imageWidth);

  // Create a Renderer
  Renderer renderer;

  // Render the scene
  renderer.renderRayTrace(camera, mesh, R, G, B);
  renderer.saveAsPNG(R, G, B, "/home/shangar21/mesh_render.png");
  return 0;
}
