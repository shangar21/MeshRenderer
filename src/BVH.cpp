#include "BVH.h"

void BVH::buildFromMesh(const Mesh &mesh) {
  std::vector<Triangle> triangles = meshToTriangles(mesh);
  root = build(triangles, 0);
}

void BVH::parseUVMap(std::vector<Triangle> &triangles) const{
	cv::Mat texture = cv::imread(uvMapPath, cv::IMREAD_COLOR);
	if(texture.empty())
		throw std::runtime_error("Something wrong with texture, double check...");
	
	for(Triangle &triangle : triangles){
		if(!triangle.hasTexCoords) continue;

		Eigen::Vector3f texColours[3];
		Eigen::Vector2f texCoords[3] = {triangle.tA, triangle.tB, triangle.tC};

		for(int i = 0; i < 3; i++){
			// convert uv to pixel coords
			int x = (int)(texCoords[i].x() * (texture.cols - 1));
			int y = (int)((1 - texCoords[i].y()) * (texture.rows - 1));

			x = std::clamp(x, 0, texture.cols - 1);
			y = std::clamp(y, 0, texture.rows - 1);

			cv::Vec3b colour = texture.at<cv::Vec3b>(y, x);
			texColours[i] = Eigen::Vector3f(colour[2], colour[1], colour[0]) / 255;
		}

		triangle.colA = texColours[0];
		triangle.colB = texColours[1];
		triangle.colC = texColours[2];
		triangle.hasColours = true;
	}

}

std::vector<Triangle> BVH::meshToTriangles(const Mesh &mesh) const {
  std::vector<Triangle> triangles;

  for (size_t i = 0; i < mesh.faces.size(); ++i) {
    Eigen::Vector3i vertexIndices = mesh.faces[i];

    // Check vertex index bounds
    if (vertexIndices[0] >= mesh.vertices.rows() ||
        vertexIndices[1] >= mesh.vertices.rows() ||
        vertexIndices[2] >= mesh.vertices.rows()) {
      std::cerr << "Error: Vertex index out of bounds in face " << i << "\n";
      continue; // Skip this face if indices are invalid
    }

    Eigen::Vector3f a = mesh.vertices.row(vertexIndices[0]);
    Eigen::Vector3f b = mesh.vertices.row(vertexIndices[1]);
    Eigen::Vector3f c = mesh.vertices.row(vertexIndices[2]);

    if (mesh.normals.size() > 0 && mesh.faceNormals.size() > i) {
      Eigen::Vector3i normalIndices = mesh.faceNormals[i];

      // Check normal index bounds
      if (normalIndices[0] >= mesh.normals.rows() ||
          normalIndices[1] >= mesh.normals.rows() ||
          normalIndices[2] >= mesh.normals.rows()) {
        std::cerr << "Error: Normal index out of bounds in face " << i << "\n";
        continue; // Skip this face if indices are invalid
      }

      Eigen::Vector3f nA = mesh.normals.row(normalIndices[0]);
      Eigen::Vector3f nB = mesh.normals.row(normalIndices[1]);
      Eigen::Vector3f nC = mesh.normals.row(normalIndices[2]);

      // skip textures till I can get em to work
      if (mesh.texcoords.size() > 0 && mesh.faceTexture.size() > i) {
        Eigen::Vector3i texIndices = mesh.faceTexture[i];

        // Check texture coordinate index bounds
        if (texIndices[0] >= mesh.texcoords.rows() ||
            texIndices[1] >= mesh.texcoords.rows() ||
            texIndices[2] >= mesh.texcoords.rows()) {
          std::cerr << "Error: Texture coordinate index out of bounds in face "
                    << i << "\n";
          continue; // Skip this face if indices are invalid
        }

        Eigen::Vector2f tA = mesh.texcoords.row(texIndices[0]).head<2>();
        Eigen::Vector2f tB = mesh.texcoords.row(texIndices[1]).head<2>();
        Eigen::Vector2f tC = mesh.texcoords.row(texIndices[2]).head<2>();

        triangles.push_back(Triangle(a, b, c, nA, nB, nC, tA, tB, tC));
      } else {
        triangles.push_back(Triangle(a, b, c, nA, nB, nC));
      }
    } else {
      triangles.push_back(Triangle(a, b, c));
    }
  }

  if (!uvMapPath.empty())
    parseUVMap(triangles);

  return triangles;
}

void BVH::sortTrianglesByAxis(std::vector<Triangle> &triangles, int axis) {
  switch (axis) {
  case 0: // Sort by x-axis
    std::sort(triangles.begin(), triangles.end(),
              [](const Triangle &a, const Triangle &b) {
                return a.centroid().x() < b.centroid().x();
              });
    break;
  case 1: // Sort by y-axis
    std::sort(triangles.begin(), triangles.end(),
              [](const Triangle &a, const Triangle &b) {
                return a.centroid().y() < b.centroid().y();
              });
    break;
  case 2: // Sort by z-axis
    std::sort(triangles.begin(), triangles.end(),
              [](const Triangle &a, const Triangle &b) {
                return a.centroid().z() < b.centroid().z();
              });
    break;
  default:
    throw std::invalid_argument("Invalid axis for sorting");
  }
}

std::shared_ptr<BVHNode> BVH::build(std::vector<Triangle> &triangles,
                                    int depth) {
  if (triangles.size() <= maxInLeaf) {
    return std::make_shared<BVHNode>(triangles);
  }

  sortTrianglesByAxis(triangles, depth % 3);
  int mid = triangles.size() / 2;
  std::vector<Triangle> leftSubset(triangles.begin(), triangles.begin() + mid);
  std::vector<Triangle> rightSubset(triangles.begin() + mid, triangles.end());

  std::shared_ptr<BVHNode> left = build(leftSubset, depth + 1);
  std::shared_ptr<BVHNode> right = build(rightSubset, depth + 1);

  std::shared_ptr<BVHNode> node = std::make_shared<BVHNode>(left, right);
  return node;
}

bool BVH::intersect(const Ray &ray, Hit &hit) const {
  hit.lambda = std::numeric_limits<float>::infinity();
  return root->intersect(ray, hit, 0.0, hit.lambda);
}
