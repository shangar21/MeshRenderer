#include "BVH.h"
#include "Camera.h"
#include "CudaCamera.cuh"
#include "CudaTriangle.cuh"
#include "Mesh.h"
#include "Project.cuh"
#include "Renderer.h"
#include <chrono>

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

  auto begin = std::chrono::high_resolution_clock::now();
  renderer.renderRayTrace(camera, bvh, R, G, B);
  auto stop = std::chrono::high_resolution_clock::now();
  auto rtDuration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - begin);
  std::cout << "BVH RayTrace in ms: " << rtDuration.count() << std::endl;

  auto start = std::chrono::high_resolution_clock::now();
  renderer.renderRasterize(camera, mesh, R, G, B);
  auto end = std::chrono::high_resolution_clock::now();
  auto rasterizeDuration =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  std::cout << "Rasterize in ms: " << rasterizeDuration.count() << std::endl;

  renderer.saveAsPNG(R, G, B, outPath);

  std::cout << "testing cuda triangle struct conversion..." << std::endl;
  std::vector<Triangle> triangles = mesh.meshToTriangles();
  CudaTriangle *cudaTriangles = triangleToCudaTriangle(triangles);

  std::cout << "testing cuda camera struct conversion..." << std::endl;
  CudaCamera *cudaCam = cameraToCudaCamera(camera);

  std::cout << "Running cuda projection kernel..." << std::endl;
  projectTriangles(cudaTriangles, cudaCam, triangles.size());

  std::cout << "Inspecting projections..." << std::endl;
  std::vector<CudaTriangle> hostTriangles(triangles.size());
  cudaDeviceSynchronize(); // Ensure the kernel is finished

  cudaMemcpy(hostTriangles.data(), cudaTriangles,
             sizeof(CudaTriangle) * hostTriangles.size(),
             cudaMemcpyDeviceToHost);

  for (int i = 0; i < 500; ++i) {
    const auto &tri = hostTriangles[i];
    std::cout << "Triangle " << i << " projA: " << tri.projA.x << ", "
              << tri.projA.y << ", " << tri.projA.z << std::endl;
  }

  freeCudaTriangles(cudaTriangles);
  freeCudaCamera(cudaCam);
  return 0;
}
