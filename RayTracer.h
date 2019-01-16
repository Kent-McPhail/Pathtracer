#pragma once
#ifndef RAYTRACERH
#define RAYTRACERH
#include <G3D/G3D.h>


class RayTracer
{
public:
	RayTracer();
	virtual ~RayTracer();
	void RayTracer::TraceImage(const shared_ptr<Camera>& cam, shared_ptr<Image3>& image, Array<shared_ptr<Surface>>& surfaceArray, const int raysPerPixel, const Array<shared_ptr<Light>>& LightArray); //iterater over all pixels 
	void RayTracer::castAllPrimaryRays(const Point2int32 pixel);
	void RayTracer::addEmissive();
	void RayTracer::calcBiradiance();
	void RayTracer::directLighting();
	Color3 RayTracer::L_i();
	void RayTracer::L_o(int bounces);
	Radiance3 RayTracer::L_indirect(int bounces);
	int m_maxNumberOfScatterEvents;
	shared_ptr<TriTree> m_triTree;

private:
	shared_ptr<Camera> m_cam;
	shared_ptr<Image3> m_image;
	Array<shared_ptr<Light>> m_LightArray;
	CPUVertexArray m_vertexArray;
	Array<Tri> m_triArray;
	Array<Sphere> m_spheres;
	Array<Color3> m_modulationBuffer;
	Array<Ray> m_rayBuffer;
	Array<shared_ptr<Surfel>> m_surfelBuffer;
	Array<Biradiance3> m_biradianceBuffer;
	Array<Ray> m_shadowRayBuffer;
	Array<bool> m_lightShadowBuffer;
	int m_numberOfIndirectRays;
	int m_raysPerPixel;
	Array<Radiance3> m_outputBuffer;
	
};




#endif // !RAYTRACERH

