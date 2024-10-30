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
