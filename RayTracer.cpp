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
	CPUVertexArray m_vertexArray;
	Array<Tri> m_triArray;
	
	Array<Ray> rayBuffer;
	Array<shared_ptr<Surfel>> surfelBuffer;	
	Array<Radiance3> outputBuffer;
	
	rayBuffer.resize(bufferSize);	
	surfelBuffer.resize(bufferSize);
	outputBuffer.resize(bufferSize);

	m_triTree = TriTree::create(true);
	m_triTree->setContents(surfaceArray, COPY_TO_CPU);
	m_raysPerPixel = 128;
	Surface::getTris(surfaceArray, m_vertexArray, m_triArray);
	Tri::setStorage(m_triArray, COPY_TO_CPU);

	Radiance3 sum = Radiance3::zero();	
	m_maxNumberOfScatterEvents = 2;
	m_image->setAll(Color3(0.0f));
	for (int currentNumberOfRays(0); currentNumberOfRays < m_raysPerPixel; ++currentNumberOfRays) {
		castAllPrimaryRays(rayBuffer);
		//runConcurrently(Point2int32(0, 0), Point2int32(m_image->width(), m_image->height()), [this](Point2int32 pixel) {castAllPrimaryRays(pixel, rayBuffer); }, false);
		L_i(rayBuffer, LightArray, surfelBuffer, outputBuffer);	
	}	
	runConcurrently(Point2int32(0, 0), Point2int32(m_image->width(), m_image->height()), [&](Point2int32 pixel) {
		shared_ptr<Surfel> surfel(surfelBuffer[pixel.x + pixel.y * m_image->width()]);
		//m_image->set(pixel.x, pixel.y, Color3(surfel->position*0.3 + Vector3(0.5, 0.5, 0.5)));     //Debug hit points
		//m_image->set(pixel.x, pixel.y, Color3((surfel->geometricNormal + Vector3(1, 1, 1)) / 2));  //Debug Normals	
		image->set(pixel.x, pixel.y, outputBuffer[pixel.x + pixel.y * m_image->width()]);				
	}, false);

	//image = m_image;
}
void RayTracer::castAllPrimaryRays(
	Array<Ray>& rayBuffer) {
	//Random& rng = Random::threadCommon();
	runConcurrently(Point2int32(0, 0), Point2int32(m_image->width(), m_image->height()), [&](Point2int32 pixel){
		rayBuffer[pixel.x + pixel.y * m_image->width()] = m_cam->worldRay(pixel.x + Random::threadCommon().uniform(), 
												pixel.y + Random::threadCommon().uniform(), m_image->rect2DBounds());
	}, false);
	return;
}
Color3 RayTracer::L_i(
		Array<Ray>& rayBuffer, 
		const Array<shared_ptr<Light>>& LightArray, 
		Array<shared_ptr<Surfel>>& surfelBuffer, 
		Array<Radiance3>& outputBuffer) {

	int bounces(1);	
	Array<Color3> modulationBuffer;
	modulationBuffer.resize(rayBuffer.size());
	Color3 modBufferInit(1.0f / m_raysPerPixel);	

	runConcurrently(int(0), modulationBuffer.size(), [&](int i) {
		modulationBuffer[i] = modBufferInit;
	}, false);
	m_triTree->intersectRays(rayBuffer, surfelBuffer);	 	
	L_o(rayBuffer, LightArray, surfelBuffer, outputBuffer, modulationBuffer, bounces);
	return Color3::black();
}
void RayTracer::addEmissive(
		const Array<Ray>& rayBuffer, 
		const Array<shared_ptr<Surfel>>& surfelBuffer, 
		Array<Radiance3>& outputBuffer, 
		const Array<Color3>& modulationBuffer) {

	runConcurrently(int(0), outputBuffer.size(), [&](int i) {
		if (notNull(surfelBuffer[i])) {
			debugAssertM(rayBuffer[i].direction().isUnit(), "Ray is not unit direction");
			outputBuffer[i] += surfelBuffer[i]->emittedRadiance(-rayBuffer[i].direction()) * modulationBuffer[i];
		}
	}, false);
	return;
}

void RayTracer::calcBiradiance(
		const Array<shared_ptr<Surfel>>& surfelBuffer,
		Array<Biradiance3>& biradianceBuffer, 
		Array<Ray>& shadowRayBuffer, 
		const Array<shared_ptr<Light>>& LightArray){
	float epsilon = .001;
	if (LightArray.size()==1) {
		runConcurrently(int(0), biradianceBuffer.size(), [&](int i) {
			if (notNull(surfelBuffer[i])) {
				Random& rng = Random::threadCommon();
				int chooseLight(rng.integer(0, LightArray.size() - 1));
				const Vector4 Y(LightArray[chooseLight]->position());
				Vector3 overSurface = surfelBuffer[i]->position + surfelBuffer[i]->geometricNormal * epsilon;
				biradianceBuffer[i] = LightArray[chooseLight]->biradiance(surfelBuffer[i]->position);
				Vector3 wi(overSurface - Y.xyz());
				float distance = wi.length();
				wi /= distance;
				distance -= epsilon * 2.0f;
				debugAssert(distance > 0.0f);
				shadowRayBuffer[i] = Ray(Y.xyz(), wi, 0.01f, distance);
			}
		}, false);
	}

}

void RayTracer::directLighting(
		const Array<Ray>& rayBuffer, 
		const Array<shared_ptr<Surfel>>& surfelBuffer, 
		const Array<shared_ptr<Light>>& LightArray,	
		const Array<Biradiance3>& biradianceBuffer, 
		Array<Ray>& shadowRayBuffer,
		const Array<Color3>& modulationBuffer, 
		Array<Radiance3>& outputBuffer,
		const Array<bool> lightShadowBuffer){

	runConcurrently(int(0), lightShadowBuffer.size(), [&](int i) {
		if (!lightShadowBuffer[i]) {
			shared_ptr<Surfel> currentSurfel(surfelBuffer[i]);
			if (notNull(currentSurfel)) {				
				debugAssertM(rayBuffer[i].direction().isUnit(), "Ray is not unit direction");
				const Vector3 wi(shadowRayBuffer[i].direction());
				const Color3 f(currentSurfel->finiteScatteringDensity(-wi, -rayBuffer[i].direction()));
				debugAssertM(f.min() >= 0.0f, "Negative finiteScatteringDensity");
				debugAssertM(f.isFinite(), "Infinite/NaN finiteScatteringDensity");
				debugAssertM(modulationBuffer[i].isFinite(), "Non-finite modulation");				
				outputBuffer[i] += biradianceBuffer[i] * f * modulationBuffer[i];
			}
		}
	}, false);
}

void RayTracer::L_o(
		Array<Ray>& rayBuffer, 
		const Array<shared_ptr<Light>>& LightArray, 
		Array<shared_ptr<Surfel>>& surfelBuffer,
		Array<Radiance3>& outputBuffer, 
		Array<Color3>& modulationBuffer, 
		int bounces) {

	Array<Biradiance3> biradianceBuffer;
	Array<Ray> shadowRayBuffer;
	Array<bool> lightShadowBuffer;

	biradianceBuffer.resize(rayBuffer.size());
	shadowRayBuffer.resize(rayBuffer.size());
	lightShadowBuffer.resize(rayBuffer.size());
	//Add Emissive terms
	//if(bounces < 2) addEmissive(rayBuffer, surfelBuffer, outputBuffer, modulationBuffer);
	addEmissive(rayBuffer, surfelBuffer, outputBuffer, modulationBuffer);
	calcBiradiance(surfelBuffer, biradianceBuffer, shadowRayBuffer, LightArray);
	//Checks if the point is in shadow from the chosen light
	m_triTree->intersectRays(shadowRayBuffer, lightShadowBuffer, TriTree::OCCLUSION_TEST_ONLY);
	//if(bounces >1)directLighting();	
	directLighting(rayBuffer, surfelBuffer, LightArray, biradianceBuffer, shadowRayBuffer, modulationBuffer, outputBuffer, lightShadowBuffer);
	if (bounces < m_maxNumberOfScatterEvents) {
		L_indirect(rayBuffer, surfelBuffer, outputBuffer, modulationBuffer, LightArray, bounces);
	}	
	return;
}
Radiance3 RayTracer::L_indirect(
		Array<Ray>& rayBuffer, 
		Array<shared_ptr<Surfel>>& surfelBuffer,
		Array<Radiance3>& outputBuffer, 
		Array<Color3>& modulationBuffer, 
		const Array<shared_ptr<Light>>& LightArray, int bounces) {
	Radiance3 L(0.0);	
	Array<Color3> weight;
	Array<Vector3> newRayDirection;
	weight.resize(rayBuffer.size());
	newRayDirection.resize(rayBuffer.size());
	bool wasImpulse(false);
	float epsilon(.001);
	//Scatter the rays 
	//Create degenerative rays for rays that missed the scene completely
	runConcurrently(int(0), outputBuffer.size(), [&](int i) {
			
		shared_ptr<Surfel>  surfel = surfelBuffer[i];
		if (notNull(surfel)) {
			Random& rng = Random::threadCommon();
			Vector3 wo = -rayBuffer[i].direction();
			surfel->scatter(PathDirection::EYE_TO_SOURCE, wo, false, Random::threadCommon(), weight[i], newRayDirection[i], wasImpulse);
			newRayDirection[i] = normalize(newRayDirection[i]);
			debugAssertM(newRayDirection[i].isUnit(), "Ray is not unit direction");
			debugAssertM(weight[i].isFinite(), "Nonfinite weight");
			debugAssertM(weight[i].min() >= 0.0f, "Negative weight");
			//rayBuffer[i] = Ray(Ray(surfelBuffer[i]->position, newRayDirection).bumpedRay(epsilon));
			//rayBuffer[i] = Ray(newRay.origin(), newRayDirection);
			//rayBuffer[i] = Ray(surfelBuffer[i]->position + epsilon * geometricNormal, normalize(newRayDirection), 0.1f);
			rayBuffer[i] = Ray(surfel->position + epsilon * surfel->geometricNormal * sign(newRayDirection[i].dot(surfel->geometricNormal)), newRayDirection[i]);
			modulationBuffer[i] *= weight[i];
		}
		else {
			//Degenerate Ray for rays that missed scene completely 
			rayBuffer[i] = Ray(Vector3(10000.0f, 10000.0f, 10000.0f), Vector3::unitX(),0.01f, 0.02f);
		}
		//debugAssertM(rayBuffer[i].direction().isUnit(), "Ray is not unit direction");

	}, false);

	//Test all triangles for intersection with the current ray
	m_triTree->intersectRays(rayBuffer, surfelBuffer);
	bounces++;		
	L_o(rayBuffer, LightArray, surfelBuffer, outputBuffer, modulationBuffer, bounces);		
	return L;
}