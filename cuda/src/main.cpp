#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <cuda_runtime.h>
#include "renderer.cuh"


// Helper function to parse a vertex index in the form v/vt/vn
int parseVertexIndex(const std::string& token) {
    size_t pos = token.find('/');
    if (pos != std::string::npos) {
        return std::stoi(token.substr(0, pos)) - 1;  // Only parse the vertex index, convert to 0-based
    } else {
        return std::stoi(token) - 1;  // If there are no slashes, just parse the integer
    }
}

// Function to load mesh data from a simple .obj file format
bool loadMeshFromFile(const std::string& filename, mesh& d_mesh) {
    std::vector<float3> h_vertices;
    std::vector<int3> h_faces;

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string type;
        iss >> type;
        if (type == "v") {
            float x, y, z;
            iss >> x >> y >> z;
            h_vertices.push_back(make_float3(x, y, z));
        } else if (type == "f") {
            std::string vertex1, vertex2, vertex3;
            iss >> vertex1 >> vertex2 >> vertex3;
            int v0 = parseVertexIndex(vertex1);
            int v1 = parseVertexIndex(vertex2);
            int v2 = parseVertexIndex(vertex3);

            std::cout << "Face: " << v0 << " " << v1 << " " << v2 << "\n";
            h_faces.push_back(make_int3(v0, v1, v2));  // Store 0-based indices for vertices
        }
    }

    file.close();

    d_mesh.num_vertices = h_vertices.size();
    d_mesh.num_faces = h_faces.size();

    std::cout << "Mesh has " << d_mesh.num_vertices << " vertices\n";
    std::cout << "Mesh has " << d_mesh.num_faces << " faces\n";

    // Allocate device memory and copy vertices
    cudaMalloc(&d_mesh.vertices, d_mesh.num_vertices * sizeof(float3));
    cudaMemcpy(d_mesh.vertices, h_vertices.data(), d_mesh.num_vertices * sizeof(float3), cudaMemcpyHostToDevice);

    // Allocate device memory and copy faces
    cudaMalloc(&d_mesh.faces, d_mesh.num_faces * sizeof(int3));
    cudaMemcpy(d_mesh.faces, h_faces.data(), d_mesh.num_faces * sizeof(int3), cudaMemcpyHostToDevice);

    return true;
}

// Function to save rendered RGB buffers as PNG using OpenCV
void saveImageAsPNG(const std::string& filename, float* d_r, float* d_g, float* d_b, int width, int height) {
    // Allocate host memory and copy data from device
    std::vector<float> h_r(width * height);
    std::vector<float> h_g(width * height);
    std::vector<float> h_b(width * height);

    cudaMemcpy(h_r.data(), d_r, width * height * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_g.data(), d_g, width * height * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_b.data(), d_b, width * height * sizeof(float), cudaMemcpyDeviceToHost);

    // Convert to 8-bit RGB format for OpenCV
    cv::Mat image(height, width, CV_8UC3);
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int idx = y * width + x;
            image.at<cv::Vec3b>(y, x) = cv::Vec3b(
                static_cast<unsigned char>(std::min(255.0f, h_b[idx] * 255.0f)),
                static_cast<unsigned char>(std::min(255.0f, h_g[idx] * 255.0f)),
                static_cast<unsigned char>(std::min(255.0f, h_r[idx] * 255.0f))
            );
        }
    }

    // Save as PNG
    cv::imwrite(filename, image);
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <obj_file_path>" << std::endl;
        return 1;
    }

    std::string objFilePath = argv[1];
    mesh d_mesh;

    if (!loadMeshFromFile(objFilePath, d_mesh)) {
        return 1;
    }

    // Set up the camera
    camera cam;
    cam.eye = make_float3(0.0f, 0.0f, 1.0f);
    cam.target = make_float3(0.0f, 0.0f, 0.0f);
    cam.up = make_float3(0.0f, 1.0f, 0.0f);
    cam.fl = 1.0f;
    cam.image_width = 800;
    cam.image_height = 600;
    cam.aspect_ratio = static_cast<float>(cam.image_width) / cam.image_height;

    // Calculate viewport dimensions and pixel steps
    float vh = 2.0f * cam.fl;
    float vw = vh * cam.aspect_ratio;
    float3 du = make_float3(vw / cam.image_width, 0.0f, 0.0f);
    float3 dv = make_float3(0.0f, vh / cam.image_height, 0.0f);

    // Allocate device memory for image output buffers
    float *d_image_r, *d_image_g, *d_image_b;
    cudaMalloc(&d_image_r, cam.image_width * cam.image_height * sizeof(float));
    cudaMalloc(&d_image_g, cam.image_width * cam.image_height * sizeof(float));
    cudaMalloc(&d_image_b, cam.image_width * cam.image_height * sizeof(float));

    // Launch the CUDA kernel for rendering
    launch_ray_trace_kernel(cam, d_mesh, d_image_r, d_image_g, d_image_b, vh, vw, du, dv);

    // Save the rendered image
    saveImageAsPNG("rendered_output.png", d_image_r, d_image_g, d_image_b, cam.image_width, cam.image_height);

    std::cout << "Rendering completed! Image saved as 'rendered_output.png'." << std::endl;

    // Free device memory
    cudaFree(d_mesh.vertices);
    cudaFree(d_mesh.faces);
    cudaFree(d_image_r);
    cudaFree(d_image_g);
    cudaFree(d_image_b);

    return 0;
}


