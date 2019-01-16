#include "RayTracer.h"



RayTracer::RayTracer()
{
}

RayTracer::~RayTracer()
{
}
void RayTracer::TraceImage(const shared_ptr<Camera>& cam, shared_ptr<Image3>& image, Array<shared_ptr<Surface>>& surfaceArray, int raysPerPixel, const Array<shared_ptr<Light>>& LightArray) {
	
	m_image = image;
	m_cam = cam;
	const int bufferSize = m_image->width() * m_image->height();
	m_LightArray = LightArray;
	m_lightShadowBuffer.resize(bufferSize);
	m_biradianceBuffer.resize(bufferSize);
	m_modulationBuffer.resize(bufferSize);
	m_shadowRayBuffer.resize(bufferSize);
	m_rayBuffer.resize(bufferSize);
	m_surfelBuffer.resize(bufferSize);
	m_outputBuffer.resize(bufferSize);
	m_triTree = TriTree::create(true);
	m_triTree->setContents(surfaceArray, COPY_TO_CPU);
	m_raysPerPixel = 128;
	Surface::getTris(surfaceArray, m_vertexArray, m_triArray);
	Tri::setStorage(m_triArray, COPY_TO_CPU);
	Radiance3 sum = Radiance3::zero();	
	m_maxNumberOfScatterEvents = 2;
	m_image->setAll(Color3(0.0f));
	for (int currentNumberOfRays(0); currentNumberOfRays < m_raysPerPixel; ++currentNumberOfRays) {
		runConcurrently(Point2int32(0, 0), Point2int32(m_image->width(), m_image->height()), [this](Point2int32 pixel) {castAllPrimaryRays(pixel); }, false);
		L_i();	
	}	
	runConcurrently(Point2int32(0, 0), Point2int32(m_image->width(), m_image->height()), [&](Point2int32 pixel) {
		shared_ptr<Surfel> surfel(m_surfelBuffer[pixel.x + pixel.y * m_image->width()]);
		//m_image->set(pixel.x, pixel.y, Color3(surfel->position*0.3 + Vector3(0.5, 0.5, 0.5)));     //Debug hit points
		//m_image->set(pixel.x, pixel.y, Color3((surfel->geometricNormal + Vector3(1, 1, 1)) / 2));  //Debug Normals	
		m_image->set(pixel.x, pixel.y, m_outputBuffer[pixel.x + pixel.y * m_image->width()]);				
	}, false);

	image = m_image;
}
void RayTracer::castAllPrimaryRays(const Point2int32 pixel) {
	//Random& rng = Random::threadCommon();
	m_rayBuffer[pixel.x + pixel.y * m_image->width()] = m_cam->worldRay(pixel.x + Random::threadCommon().uniform(), pixel.y + Random::threadCommon().uniform(), m_image->rect2DBounds());
	return;
}
Color3 RayTracer::L_i() {
	int bounces(1);	
	
	Color3 modBufferInit(1.0f / m_raysPerPixel);	
	runConcurrently(int(0), m_modulationBuffer.size(), [&](int i) {
		m_modulationBuffer[i] = modBufferInit;
	}, false);
	m_triTree->intersectRays(m_rayBuffer, m_surfelBuffer);	 	
	L_o(bounces);			
	return Color3::black();
}
void RayTracer::addEmissive() {
	runConcurrently(int(0), m_outputBuffer.size(), [&](int i) {
		if (notNull(m_surfelBuffer[i])) {
			m_outputBuffer[i] += m_surfelBuffer[i]->emittedRadiance(-m_rayBuffer[i].direction()) * m_modulationBuffer[i];
		}
	}, false);
	return;
}
void RayTracer::calcBiradiance(){
	float epsilon = .01;
	runConcurrently(int(0), m_biradianceBuffer.size(), [&](int i) {
		if (notNull(m_surfelBuffer[i])) {
			Random& rng = Random::threadCommon();
			int chooseLight(rng.integer(0, m_LightArray.size() - 1));
			const Vector4 Y(m_LightArray[chooseLight]->position());
			Vector3 overSurface = m_surfelBuffer[i]->position + m_surfelBuffer[i]->geometricNormal * epsilon;
			m_biradianceBuffer[i] = m_LightArray[chooseLight]->biradiance(m_surfelBuffer[i]->position);
			Vector3 wi(overSurface - Y.xyz());
			float distance = wi.length();
			wi /= distance;
			distance -= epsilon;
			debugAssert(distance > 0.0f);
			m_shadowRayBuffer[i] = Ray(Y.xyz(), wi, 0.0f, distance);
		}
	}, false);
}
void RayTracer::directLighting(){
	runConcurrently(int(0), m_lightShadowBuffer.size(), [&](int i) {
		if (!m_lightShadowBuffer[i]) {
			shared_ptr<Surfel> currentSurfel(m_surfelBuffer[i]);
			if (notNull(currentSurfel)) {
				const Vector3 wi(m_shadowRayBuffer[i].direction());
				const Color3 f(currentSurfel->finiteScatteringDensity(-wi, -m_rayBuffer[i].direction()));
				m_outputBuffer[i] += m_biradianceBuffer[i] * f * m_modulationBuffer[i];
			}
		}
	}, false);
}
void RayTracer::L_o(int bounces) {
	//Add Emissive terms
	if(bounces < 2) addEmissive();
	//addEmissive();
	calcBiradiance();
	//Checks if the point is in shadow from the chosen light
	m_triTree->intersectRays(m_shadowRayBuffer, m_lightShadowBuffer, TriTree::OCCLUSION_TEST_ONLY);
	//if(bounces >1)directLighting();	
	directLighting();
	if (bounces < m_maxNumberOfScatterEvents) {
		L_indirect(bounces);
	}	
	return;
}
Radiance3 RayTracer::L_indirect(int bounces) {
	Radiance3 L(0.0);
	float epsilon(.01);
	Color3 weight;
	Vector3 newRayDirection;
	bool wasImpulse(false);
	//Scatter the rays 
	//Create degenerative rays for rays that missed the scene completely
	runConcurrently(int(0), m_outputBuffer.size(), [&](int i) {	
		if (notNull(m_surfelBuffer[i])) {
			Random& rng = Random::threadCommon();
			Vector3 geometricNormal(m_surfelBuffer[i]->geometricNormal);
			Vector3 wo = -m_rayBuffer[i].direction();
			m_surfelBuffer[i]->scatter(PathDirection::EYE_TO_SOURCE, wo, false, Random::threadCommon(), weight, newRayDirection, wasImpulse);
			newRayDirection = normalize(newRayDirection);
			//m_rayBuffer[i] = Ray(Ray(m_surfelBuffer[i]->position, newRayDirection).bumpedRay(epsilon));
			//m_rayBuffer[i] = Ray(newRay.origin(), newRayDirection);
			//m_rayBuffer[i] = Ray(m_surfelBuffer[i]->position + epsilon * geometricNormal, normalize(newRayDirection), 0.1f);
			m_rayBuffer[i] = Ray(m_surfelBuffer[i]->position + epsilon * geometricNormal, newRayDirection, 0.1f);
			m_modulationBuffer[i] *= weight;
		}

		else {
			//Degenerate Ray for rays that missed scene completely 
			m_rayBuffer[i] = Ray(Vector3(10000.0f, 10000.0f, 10000.0f), Vector3::unitX(),0.0f, 0.5f);
		}
	}, false);

	//Test all triangles for intersection with the current ray
	m_triTree->intersectRays(m_rayBuffer, m_surfelBuffer);
	bounces++;		
	L_o(bounces);		
	return L;
}