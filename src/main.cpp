#include "BVH.h"
#include "Camera.h"
#include "Mesh.h"
#include "Renderer.h"

int main(int argc, char *argv[]) {

  if (argc < 3) {
    std::cerr
        << "Error: Add path to obj file and output file to render something!"
        << std::endl;
    return 1;
  }

  std::string objPath = argv[1];
  std::string outPath = argv[2];
	std::string texPath = argv[3];

  Camera camera(Eigen::Vector3f(2.0f, 3.0f, 3.0f), // Camera position
                Eigen::Vector3f(0.0f, 0.0f, 0.0f), // Target point
                Eigen::Vector3f(0.0f, 1.0f, 0.0f), // Up vector
                500.0f, 800, 600);
  Mesh mesh(texPath);
  bool loaded = mesh.loadFromObj(objPath);
  mesh.printMeshInfo();

  BVH bvh;
  bvh.buildFromMesh(mesh);

  Eigen::MatrixXf R(camera.imageHeight, camera.imageWidth);
  Eigen::MatrixXf G(camera.imageHeight, camera.imageWidth);
  Eigen::MatrixXf B(camera.imageHeight, camera.imageWidth);

  Renderer renderer;

  //renderer.renderRayTrace(camera, bvh, R, G, B);
  renderer.renderRasterize(camera, mesh, R, G, B);
  renderer.saveAsPNG(R, G, B, outPath);
  return 0;
}
