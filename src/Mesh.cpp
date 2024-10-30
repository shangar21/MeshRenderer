#define TINYOBJLOADER_IMPLEMENTATION
#include "Mesh.h"
#include "tiny_obj_loader.h"

bool Mesh::loadFromObj(const std::string &filename) {
  tinyobj::ObjReaderConfig readerConfig;
  tinyobj::ObjReader reader;

  if (!reader.ParseFromFile(filename, readerConfig)) {
    if (!reader.Error().empty()) {
      std::cerr << "TinyObjReader Error: " << reader.Error();
    }
    exit(1);
  }

  if (!reader.Warning().empty()) {
    std::cout << "TinyObjReader Warning: " << reader.Warning();
  }

  auto &attrib = reader.GetAttrib();
  auto &shapes = reader.GetShapes();
  // auto& materials = reader.GetMaterials(); // disregard for now

  size_t numVertices = attrib.vertices.size() / 3;
  vertices = Eigen::MatrixXf(numVertices, 3);
  for (size_t i = 0; i < numVertices; i++) {
    vertices(i, 0) = attrib.vertices[3 * i];
    vertices(i, 1) = attrib.vertices[3 * i + 1];
    vertices(i, 2) = attrib.vertices[3 * i + 2];
  }

  if (!attrib.normals.empty()) {
    size_t numNormals = attrib.normals.size() / 3;
    normals = Eigen::MatrixXf(numNormals, 3);
    for (size_t i = 0; i < numVertices; i++) {
      normals(i, 0) = attrib.normals[3 * i];
      normals(i, 1) = attrib.normals[3 * i + 1];
      normals(i, 2) = attrib.normals[3 * i + 2];
    }
  }

  if (!attrib.texcoords.empty()) {
    size_t numNormals = attrib.texcoords.size() / 3;
    texcoords = Eigen::MatrixXf(numNormals, 3);
    for (size_t i = 0; i < numVertices; i++) {
      texcoords(i, 0) = attrib.texcoords[3 * i];
      texcoords(i, 1) = attrib.texcoords[3 * i + 1];
      texcoords(i, 2) = attrib.texcoords[3 * i + 2];
    }
  }

  for (size_t s = 0; s < shapes.size(); s++) {
    size_t indexOffset = 0;

    for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
      size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);

      if (fv != 3) {
        std::cerr << "Not a triangle, skipping." << std::endl;
        indexOffset += fv;
        continue;
      }

      Eigen::Vector3i face;
      Eigen::Vector3i normal;
      Eigen::Vector3i texture;
      for (size_t v = 0; v < fv; v++) {
        tinyobj::index_t idx = shapes[s].mesh.indices[indexOffset + v];
        face[v] = idx.vertex_index;
        if (idx.normal_index >= 0)
          normal[v] = idx.normal_index;
        else
          normal[v] = -1;
        if (idx.texcoord_index >= 0)
          texture[v] = idx.texcoord_index;
        else
          texture[v] = -1;
      }
      faces.push_back(face);
      faceNormals.push_back(normal);
      faceTexture.push_back(texture);
      indexOffset += fv;
    }
  }

  return true;
}

void Mesh::printMeshInfo() const {
  std::cout << "Mesh Info: \n";
  std::cout << "Number of vertices: " << vertices.rows() << std::endl;
  std::cout << "Number of normals: " << normals.rows() << std::endl;
  std::cout << "Number of texture coordinates: " << texcoords.rows()
            << std::endl;
  std::cout << "Number of faces: " << faces.size() << std::endl;
}

Hit Mesh::intersect(const Ray &ray) const {
  Hit closestHit;
  closestHit.lambda = std::numeric_limits<float>::max();

#pragma omp parallel for
  for (int i = 0; i < faces.size(); i++) {
    const Eigen::Vector3i &face = faces[i];

    Eigen::Vector3f a = vertices.row(face[0]);
    Eigen::Vector3f b = vertices.row(face[1]);
    Eigen::Vector3f c = vertices.row(face[2]);

    Eigen::Vector3f edge1 = b - a;
    Eigen::Vector3f edge2 = c - a;

    Eigen::Vector3f h = ray.direction.cross(edge2);
    float det = edge1.dot(h);

    if (fabs(det) < FP_TOLERANCE) {
      continue;
    }

    float invDet = 1.0f / det;

    Eigen::Vector3f s = ray.origin - a;

    float u = invDet * s.dot(h);
    if (u < 0.0f || u > 1.0f) {
      continue; // The intersection point is outside the triangle
    }

    Eigen::Vector3f q = s.cross(edge1);
    float v = invDet * ray.direction.dot(q);
    if (v < 0.0f || (u + v) > 1.0f) {
      continue; // The intersection point is outside the triangle
    }

    float lambda = invDet * edge2.dot(q);
    if (lambda < FP_TOLERANCE) {
      continue; // Ignore intersections behind the ray's origin
    }

    if (lambda < closestHit.lambda) {
      closestHit.hit = true;
      closestHit.lambda = lambda;
      closestHit.point = ray.pointAt(lambda);
      closestHit.normal = edge1.cross(edge2).normalized();
      // Need to set colour to something else later
      // closestHit.colour =
      //    Eigen::Vector3f(1.0f, 1.0f, 1.0f); // White color for now
      closestHit.colour =
          (closestHit.normal + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.0f;
    }
  }

  return closestHit;
}

void Mesh::renderWithCuda(const camera &cam) {
  mesh d_mesh;
  d_mesh.num_faces = faces.size();
  d_mesh.num_vertices = vertices.rows();

  // Allocate and copy vertices and faces to device
  cudaMalloc(&d_mesh.vertices, d_mesh.num_vertices * sizeof(float3));
  std::vector<float3> h_vertices(d_mesh.num_vertices);
  for (int i = 0; i < d_mesh.num_vertices; ++i) {
    h_vertices[i] = make_float3(vertices(i, 0), vertices(i, 1), vertices(i, 2));
  }
  cudaMemcpy(d_mesh.vertices, h_vertices.data(),
             d_mesh.num_vertices * sizeof(float3), cudaMemcpyHostToDevice);

  cudaMalloc(&d_mesh.faces, d_mesh.num_faces * sizeof(int3));
  std::vector<int3> h_faces(d_mesh.num_faces);
  for (int i = 0; i < d_mesh.num_faces; ++i) {
    h_faces[i] = make_int3(faces[i][0], faces[i][1], faces[i][2]);
  }
  cudaMemcpy(d_mesh.faces, h_faces.data(), d_mesh.num_faces * sizeof(int3),
             cudaMemcpyHostToDevice);

  // Allocate device image buffers
  float *d_image_r, *d_image_g, *d_image_b;
  cudaMalloc(&d_image_r, cam.image_width * cam.image_height * sizeof(float));
  cudaMalloc(&d_image_g, cam.image_width * cam.image_height * sizeof(float));
  cudaMalloc(&d_image_b, cam.image_width * cam.image_height * sizeof(float));

  // Calculate viewport dimensions and pixel steps
  float vh = 2.0f * cam.fl;
  float vw = vh * cam.aspect_ratio;
  float3 du = make_float3(vw / cam.image_width, 0.0f, 0.0f);
  float3 dv = make_float3(0.0f, vh / cam.image_height, 0.0f);

  // Launch the CUDA renderer
  launchRayTraceKernel(cam, d_mesh, d_image_r, d_image_g, d_image_b, vh, vw, du,
                       dv);

  // Free device memory
  cudaFree(d_mesh.vertices);
  cudaFree(d_mesh.faces);
  cudaFree(d_image_r);
  cudaFree(d_image_g);
  cudaFree(d_image_b);
}
