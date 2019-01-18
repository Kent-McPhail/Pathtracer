#pragma once
#ifndef RAYTRACERH
#define RAYTRACERH
#include <G3D/G3D.h>


class RayTracer
{
public:
	RayTracer();
	virtual ~RayTracer();

	void RayTracer::TraceImage(
		const shared_ptr<Camera>& cam, 
		shared_ptr<Image3>& image, 
		Array<shared_ptr<Surface>>& surfaceArray, 
		const int raysPerPixel, 
		const Array<shared_ptr<Light>>& LightArray); 

	void RayTracer::castAllPrimaryRays(
		Array<Ray>& rayBuffer);

	void RayTracer::addEmissive(
		const Array<Ray>& rayBuffer,
		const Array<shared_ptr<Surfel>>& surfelBuffer,
		Array<Radiance3>& outputBuffer,
		const Array<Color3>& modulationBuffer);

	void RayTracer::calcBiradiance(
		const Array<shared_ptr<Surfel>>& surfelBuffer,
		Array<Biradiance3>& biradianceBuffer,
		Array<Ray>& shadowRayBuffer,
		const Array<shared_ptr<Light>>& LightArray);

	void RayTracer::directLighting(
		const Array<Ray>& rayBuffer,
		const Array<shared_ptr<Surfel>>& surfelBuffer,
		const Array<shared_ptr<Light>>& LightArray,
		const Array<Biradiance3>& biradianceBuffer,
		Array<Ray>& shadowRayBuffer,
		const Array<Color3>& modulationBuffer,
		Array<Radiance3>& outputBuffer,
		const Array<bool> lightShadowBuffer);

	void RayTracer::L_o(
		Array<Ray>& rayBuffer, 
		const Array<shared_ptr<Light>>& LightArray,
		Array<shared_ptr<Surfel>>& surfelBuffer,
		Array<Radiance3>& outputBuffer,
		Array<Color3>& modulationBuffer, 
		int bounces);

	Color3 RayTracer::L_i(
		Array<Ray>& rayBuffer, 
		const Array<shared_ptr<Light>>& LightArray,
		Array<shared_ptr<Surfel>>& surfelBuffer, 
		Array<Radiance3>& outputBuffer);	

	Radiance3 RayTracer::L_indirect(
		Array<Ray>& rayBuffer, 
		Array<shared_ptr<Surfel>>& surfelBuffer,
		Array<Radiance3>& outputBuffer, 
		Array<Color3>& modulationBuffer, 
		const Array<shared_ptr<Light>>& LightArray, int bounces);

	int m_maxNumberOfScatterEvents;
	shared_ptr<TriTree> m_triTree;

private:
	shared_ptr<Camera> m_cam;
	shared_ptr<Image3> m_image;	
	int m_numberOfIndirectRays;
	int m_raysPerPixel;
	
	
};




#endif // !RAYTRACERH

