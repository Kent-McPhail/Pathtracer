#include "RayTracer.h"




RayTracer::RayTracer()
{
}

RayTracer::~RayTracer()
{
}
void RayTracer::TraceImage(
	const shared_ptr<Camera>& cam,
	shared_ptr<Image3>& image, 
	Array<shared_ptr<Surface>>& surfaceArray, 
	const Array<shared_ptr<Light>>& LightArray) {
	
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
	m_maxNumberOfScatterEvents = 4;
	Surface::getTris(surfaceArray, m_vertexArray, m_triArray);
	Tri::setStorage(m_triArray, COPY_TO_CPU);

	Radiance3 sum = Radiance3::zero();	
	
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
		Array<Ray>& rayBuffer,
		const Array<shared_ptr<Surfel>>& surfelBuffer,
		Array<Biradiance3>& biradianceBuffer, 
		Array<Ray>& shadowRayBuffer, 
		const Array<shared_ptr<Light>>& LightArray){
	float epsilon = .0001;
	/*runConcurrently(int(0), biradianceBuffer.size(), [&](int i) {
		biradianceBuffer[i] = Biradiance3();
	}, false);*/
	//Don't importance sample if there is only one light in the array as there is no need
	runConcurrently(int(0), biradianceBuffer.size(), [&](int i) {
		if (LightArray.size() == 1) {
			if (notNull(surfelBuffer[i])) {
				biradianceBuffer[i] = LightArray[0]->biradiance(surfelBuffer[i]->position);
				const Vector4 Y(LightArray[0]->position());
				Vector3 overSurface = surfelBuffer[i]->position + surfelBuffer[i]->geometricNormal * epsilon;
				Vector3 wi(overSurface - Y.xyz());
				float distance = wi.length();
				wi /= distance;
				
				//distance -= epsilon * 2.0f;		
				debugAssert(distance > 0.0f);
				debugAssert(distance > .0001f);
				shadowRayBuffer[i] = Ray(Y.xyz(), wi, 0.0f, distance);
				//Regular surfel finite Scattering implementation
				const Color3 f(surfelBuffer[i]->finiteScatteringDensity(-wi, -rayBuffer[i].direction())); 


				biradianceBuffer[i] *= f;
			}
		}

		else {
			if (notNull(surfelBuffer[i])) {
				Vector3 overSurface = surfelBuffer[i]->position + surfelBuffer[i]->geometricNormal * epsilon;
				SmallArray<Biradiance3, 12> biradiancePerLight;
				SmallArray<Color3, 12> finiteScatterPerLight;
				biradiancePerLight.resize(LightArray.size());
				finiteScatterPerLight.resize(LightArray.size());
				Radiance totalRadiance(0);
				for (int j(0); j < LightArray.size(); ++j) {
					biradiancePerLight[j] = Biradiance3::zero();
					biradiancePerLight[j] = LightArray[j]->biradiance(surfelBuffer[i]->position);
					//finiteScatterPerLight[j] = Color3::zero();
					debugAssertM(biradianceBuffer[i].isFinite(), "Infinite/NaN biradiance");
					if (biradiancePerLight[j].sum() > 0.0f) {
						const Vector4 Y(LightArray[j]->position());
						Vector3 overSurface = surfelBuffer[i]->position + surfelBuffer[i]->geometricNormal * epsilon;
						Vector3 wi(overSurface - Y.xyz());
						float distance = wi.length();
						wi /= distance;
						debugAssert(distance > 0.0f);
						finiteScatterPerLight[j] = surfelBuffer[i]->finiteScatteringDensity(-wi, -rayBuffer[i].direction());
						//My Disney finite scattering
						debugAssertM(finiteScatterPerLight[j].isFinite(), "Infinite/NaN biradiance");
						biradiancePerLight[j] *= finiteScatterPerLight[j];
						totalRadiance += biradiancePerLight[j].sum();
					}
				}
				int j(0);
				Random& rng = Random::threadCommon();
				Radiance thisLightsRadiance(0);
				for (float rad = rng.uniform(0, totalRadiance); j < LightArray.size(); ++j) {
					rad -= biradiancePerLight[j].sum();
					if (rad <= 0.0f) {
						break;
					}
				}

				j = min(j, biradiancePerLight.size() - 1);
				Radiance probabilityWeight = totalRadiance / max(biradiancePerLight[j].sum(), 0.0001f);			
				biradianceBuffer[i] = biradiancePerLight[j] * probabilityWeight;
				//debugAssertM(probabilityWeight.isFinite(), "Infinite/NaN BSDF");
				debugAssertM(biradianceBuffer[i].isFinite(), "Infinite/NaN biradiance");
				
				const Vector4 Y(LightArray[j]->position());
				Vector3 wi(overSurface - Y.xyz());
				float distance = wi.length();
				wi /= distance;
				distance -= epsilon * 2.0f;
				debugAssert(distance > 0.0f);				
				shadowRayBuffer[i] = Ray(Y.xyz(), wi, 0.0f, distance);
			}
			else {
				shadowRayBuffer[i] = Ray(Vector3(10000.0f, 10000.0f, 10000.0f), Vector3::unitX(), 0.0f, 0.02f);
			}		
		}
	}, true);
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
				debugAssertM(modulationBuffer[i].isFinite(), "Non-finite modulation");				
				outputBuffer[i] += biradianceBuffer[i] * modulationBuffer[i];
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
	addEmissive(rayBuffer, surfelBuffer, outputBuffer, modulationBuffer);

	calcBiradiance(rayBuffer, surfelBuffer, biradianceBuffer, shadowRayBuffer, LightArray);

	//Checks if the point is in shadow from the chosen light
	m_triTree->intersectRays(shadowRayBuffer, lightShadowBuffer, TriTree::OCCLUSION_TEST_ONLY);
	
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
			rayBuffer[i] = Ray(surfel->position + epsilon * surfel->geometricNormal * sign(newRayDirection[i].dot(surfel->geometricNormal)), newRayDirection[i]);
			modulationBuffer[i] *= weight[i];
		}
		else {
			//Degenerate Ray for rays that missed scene completely 
			rayBuffer[i] = Ray(Vector3(10000.0f, 10000.0f, 10000.0f), Vector3::unitX(),0.001f, 0.02f);
		}
		//debugAssertM(rayBuffer[i].direction().isUnit(), "Ray is not unit direction");

	}, false);

	//Test all triangles for intersection with the current ray
	m_triTree->intersectRays(rayBuffer, surfelBuffer);
	bounces++;		
	L_o(rayBuffer, LightArray, surfelBuffer, outputBuffer, modulationBuffer, bounces);		
	return L;
}
