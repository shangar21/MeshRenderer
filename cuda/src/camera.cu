#include "camera.cuh"

__device__ void getC2W(const camera& cam, float* c2w){
	float3 w = normalize(cam.target - cam.eye);
	float3 u = normalize(cross(cam.up, w));
	float3 v = cross(w, u);
	
	c2w[0] = u.x; c2w[1] = u.y; c2w[2] = u.z; c2w[3] = 0.0f;
	c2w[4] = v.x; c2w[5] = v.y; c2w[6] = v.z; c2w[7] = 0.0f;
	c2w[8] = w.x; c2w[9] = w.y; c2w[10] = w.z; c2w[11] = 0.0f;
	c2w[12] = cam.eye.x; c2w[13] = cam.eye.y; c2w[14] = cam.eye.z; c2w[15] = 1.0f;
	
}

__device__ void getW2C(const camera& cam, float* w2c){
	float3 w = normalize(cam.target - cam.eye);
	float3 u = normalize(cross(cam.up, w));
	float3 v = cross(w, u);

	w2c[0] = u.x; w2c[1] = v.x; w2c[2] = w.x; w2c[3] = 0.0f;
	w2c[4] = u.y; w2c[5] = v.y; w2c[6] = w.y; w2c[7] = 0.0f;
	w2c[8] = u.z; w2c[9] = v.z; w2c[10] = w.z; w2c[11] = 0.0f;
	w2c[12] = -dot(u, cam.eye); w2c[13] = -dot(v, cam.eye); w2c[14] = -dot(w, cam.eye); w2c[15] = 1.0f;
}
