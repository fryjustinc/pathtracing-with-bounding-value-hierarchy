#include "RayTracer.h"
#include "Material.h"
#include "Lights.h"
#include <cfloat>
#include <cmath>
#include <vector>
#include <cmath>

using namespace std;


/* Constructors */

RayTracer::RayTracer(Scene* scene, RenderSettings settings) {

	tracingScene = scene;
this->settings = settings;

random = new CRandomMersenne(3);

}

RayTracer::~RayTracer()
{
    delete random;
}


/* Instance methods */

// Publicly accessible
rgb RayTracer::traceViewingRay(Ray& ray) {

    return trace(ray, 0);
}

rgb RayTracer::trace(Ray& ray, unsigned int depth, bool indirect)
{
	if (settings.mode == RAYTRACE)
return raytrace(ray, depth, indirect);
else if (settings.mode == PATHTRACE)
return pathtrace(ray, depth, indirect);
else return rgb::black;
}

rgb RayTracer::pathtrace(Ray& ray, unsigned int depth, bool indirect)
{
	IntersectRecord intersection;

/* If we hit nothing, query the environment */
if (!tracingScene->getHierarchy()->intersect(ray, &intersection))
    return tracingScene->environment->radiance(ray);

/*** Else, shade this intersection ***/

Reflectance refl = intersection.primitive->getReflectance(intersection.point);

// If direct lighting is on, this ray was an indirect illumination sample, and
// we hit a light source, we should return nothing (don't want to oversample direct illum.)
if (indirect && settings.directLighting && refl.type == Emissive)
    return rgb::black;

// Init color to emissive value
rgb pointColor = refl.kE;

// Russian Roulette
double survival = 1.0;
if (depth > settings.recursionDepth && settings.russianRoulette)
{
    rgb weight/* = rgb(.5,.5,.5)*/;
    if (refl.type == Diffuse) weight = refl.kD;
    if (refl.type == Glossy) weight = refl.kD + refl.kS;
    if (refl.type == Mirror) weight = refl.kR;
    if (refl.type == Dielectric) weight = refl.kT;
    if (russianRoulette(weight, survival))
        return pointColor;
}
    // If no Russian Roulette, just kill the path
else if (depth > settings.recursionDepth)
    return pointColor;

// DIFFUSE OBJECTS
if (refl.type == Diffuse)
{
    // Direct illumination
    if (settings.directLighting)
        pointColor += directIllumination(intersection, ray);

    // Sample diffuse interreflectance from visible hemisphere
    if (settings.indirectLighting)
        pointColor += survival * diffuseInterreflect(ray, intersection, depth);
} 

// GLOSSY OBJECTS
if (refl.type == Glossy)
{
    // Direct illumination
    if (settings.directLighting)
        pointColor += directIllumination(intersection, ray);

    // Glossy specular highlights sampled from visible hemsiphere
    if (settings.indirectLighting)
    {
        double rrMult;
        if (glossyRussianRoulette(refl.kS, refl.kD, rrMult))
            pointColor += survival * (1.0/(1-1.0/rrMult)) * diffuseInterreflect(ray, intersection, depth);
        else
            pointColor += survival * rrMult * specularInterreflect(ray, intersection, depth);
    }
}

// MIRROR OBJECTS
if (refl.type == Mirror)
{
    // Sample perfect mirror reflection
    pointColor += survival * mirrorReflect(ray, intersection, depth);
}

// DIELECTRIC OBJECTS
if (refl.type == Dielectric)
{
    Ray refrRay;
    double schlick;
    bool refracted = dielectricCalc(ray, intersection, refrRay, schlick);

    // If total internal reflection, just send reflected ray
    if (!refracted)
        pointColor += survival * mirrorReflect(ray, intersection, depth);
    else
    {
        //pointColor += survival*(1-schlick)*dielectricTransmit(refrRay,intersection,depth);

        // OPTION1: Decide which ray to send based on some probability.
        rgb fakeColor(.5,.5,.5);
        double rrMult;
        if (russianRoulette(fakeColor, rrMult))
            pointColor += survival*(1.0/(1-1.0/rrMult))*(1-schlick)*dielectricTransmit(refrRay,intersection,depth);
        else pointColor += survival*rrMult*schlick*mirrorReflect(ray,intersection,depth);

        //// OPTION2: Always send both rays
        //pointColor += survival*(1-schlick)*dielectricTransmit(refrRay,intersection,depth);
        //pointColor += survival*schlick*mirrorReflect(ray,intersection,depth);

        //// OPTION3: Russian roulette on schlick value to send reflected ray, transmitted ray, or both
        //rgb reflWeight(schlick,schlick,schlick);
        //rgb transWeight(1-schlick,1-schlick,1-schlick);
        //double rrMult;
        //if (!russianRoulette(reflWeight, rrMult))
        //	pointColor += survival*rrMult*schlick*mirrorReflect(ray,intersection,depth);
        //if (!russianRoulette(transWeight, rrMult))
        //	pointColor += survival*rrMult*(1-schlick)*dielectricTransmit(refrRay,intersection,depth);
    }
}

return pointColor;
}

bool RayTracer::russianRoulette(const rgb& refl, double& survivorMult)
{
    double p = MAX(refl[0], MAX(refl[1], refl[2]));
    survivorMult = 1.0/p;
    if (random->Random() > p) return true;
    return false;
}

bool RayTracer::glossyRussianRoulette(const rgb& kS, const rgb& kD, double& survivorMult)
{
    double spec = MAX(kS[0], MAX(kS[1], kS[2]));
    double diffuse = MAX(kD[0], MAX(kD[1], kD[2]));
    double p = spec/(spec + diffuse);
    survivorMult = 1.0/p;
    
    if (random->Random() > p)
        return true;
    else 
        return false;
}

bool RayTracer::dielectricCalc(Ray& ray, IntersectRecord& intersection, Ray& refrRay, double& schlick)
{
    vec3 direction = ray.getDirection();
    direction.normalize();
    vec3 normal = intersection.surfaceNormal;
    bool into;

    // Checking whether ray is going into the object or going out.
    // Negative dot product means going in, positive means going out (so we need to look at flipped normal)
    double cosAngle = direction * normal.normalize();
    if (cosAngle  > 0)
        into = false;
    else into = true;

    Reflectance refl = intersection.primitive->getReflectance(intersection.point);
    double index = intersection.primitive->getReflectance(intersection.point).indexOfRefraction;

    /* Compute indices of refraction for (from, to) transmission media. */
    // CASE: From an object into open air.
    double n, nt;
    if (!into/*ray.getLastHitPrim() == intersection.primitive*/)
    {
        n = index;
        nt = 1.0;
    }
        // CASE: From open air into an object.
    else
    {
        n = 1.0;
        nt = index;
    }

    /* Now compute the refraction direction (returns false if total internal reflection). */
    vec3 refractDirection;
    bool refracted = refract(ray, intersection, n, nt, refractDirection);

    // Calculate Schlick approximation of Fresnel equations
    vec3 d = ray.getDirection(); d.normalize();
    vec3 rd = refractDirection; if (refracted) rd.normalize();
    schlick = schlickCalc(n, nt, d, rd, intersection.surfaceNormal);

    // Generate refracted ray
    if (refracted)
        refrRay = Ray(intersection.point, settings.rayBias, DBL_MAX, refractDirection, ray.getSample(), intersection.primitive);

    return refracted;
}

rgb RayTracer::dielectricTransmit(Ray& ray, IntersectRecord& intersection, int depth)
{
	Reflectance refl = intersection.primitive->getReflectance(intersection.point);
rgb mult = (refl.kT == rgb::black ? refl.kR : refl.kT);
return mult * trace(ray, depth+1);
}

rgb RayTracer::directIllumination(const IntersectRecord& intersection, Ray& ray)
{
    Reflectance refl = intersection.primitive->getReflectance(intersection.point);
    rgb pointColor;
    vector<Light*> lights = tracingScene->getLights();

    // Direct illumination
    for (unsigned int i = 0; i < lights.size(); i++) {
		rgb intensity;
		vec3 lightincidence;
		lights[i]->sample(intersection.point, intersection.surfaceNormal, intensity, lightincidence);
		Ray shadowRay(intersection.point, settings.rayBias, 1-settings.rayBias, lightincidence, ray.getSample(), ray.getLastHitPrim());
		if (!traceShadowRay(shadowRay)) {
			if (refl.kD != rgb::black)
        // Diffuse: (kD/PI) * G(x,x') * V(x,x') * lightintensity * (1/probablility) -- prob. is 1/area
				pointColor += (refl.kD/M_PI)* intensity;
			if (refl.kS != rgb::black)
    {
        // Specular: (kS(pExp+1)/2PI) * cos^pExp(reflangle) * G(x,x') * V(x,x') * lightintensity * (1/probablility) -- prob. is 1/area
				vec3 r = reflect(lightincidence, intersection.surfaceNormal);
				r.normalize();
				vec3 v = -ray.getDirection();
				v.normalize();
				pointColor += (refl.kS*(refl.pExp+1)/(2*M_PI)) * pow(r*v, refl.pExp) * intensity;
    }
}
}

return pointColor;
}

rgb RayTracer::specularInterreflect(Ray& ray, IntersectRecord& intersection, int depth)
{
    Reflectance refl = intersection.primitive->getReflectance(intersection.point);
vec3 perfectReflDir = reflect(ray.getDirection(), intersection.surfaceNormal);
perfectReflDir.normalize();
vec3 rayDir = sampleUpperHemisphere(perfectReflDir, refl.pExp);
    
Ray specRay(intersection.point, settings.rayBias, DBL_MAX, rayDir, ray.getSample(), ray.getLastHitPrim());
rgb specColor = trace(specRay, depth + 1, true);
    
// brdf = kS(pExp+1)/(2PI)
if (settings.sampleType == UNIFORM)
    // Probablity: 1/(2PI) -- (1/probability)*cos(theta)*brdf*radiancealongray
    return pow(rayDir*perfectReflDir, refl.pExp) * (rayDir * intersection.surfaceNormal) * refl.kS * (refl.pExp+1) * specColor;
else if (settings.sampleType == IMPORTANCE)
    // Probability: cos^pExp(theta)*(pExp+1)/(2PI) -- (1/probability)*cos(theta)*brdf*radiancealongray
    return refl.kS * (rayDir * intersection.surfaceNormal) * specColor;
else return rgb::black;
}

rgb RayTracer::diffuseInterreflect(Ray& ray, IntersectRecord& intersection, int depth)
{
	vec3 rayDir = sampleUpperHemisphere(intersection.surfaceNormal, 1);
Ray diffRay(intersection.point, settings.rayBias, DBL_MAX, rayDir, ray.getSample(), ray.getLastHitPrim());
// albedo = brdf*PI
rgb albedo = intersection.primitive->getReflectance(intersection.point).kD;
rgb diffColor = trace(diffRay, depth + 1, true);

// brdf = kD/PI
if (settings.sampleType == UNIFORM)
    // Probablity: 1/(2PI) -- (1/probability)*cos(theta)*brdf*radiancealongray
    return 2 * (intersection.surfaceNormal * rayDir) * albedo * diffColor;
else if (settings.sampleType == IMPORTANCE)
    // Probability: cos(theta)/PI -- (1/probability)*cos(theta)*brdf*radiancealongray
    return albedo * diffColor;
else return rgb::black;
}

vec3 RayTracer::sampleUpperHemisphere(vec3& alignWithZ, double n)
{
	vec3 rayDir;
if (settings.sampleType == UNIFORM)
    rayDir = uniformSampleUpperHemisphere(alignWithZ);
else if (settings.sampleType == IMPORTANCE)
    rayDir = importanceSampleUpperHemisphere(alignWithZ, n);

return rayDir;
}

vec3 RayTracer::uniformSampleUpperHemisphere(vec3& alignWithZ)
{
    double r1, r2, r3;
    r1 = random->Random();
    r2 = random->Random();
    r3 = random->Random();

    double x, y, z;
    //do {
    //	x = 1 - 2*r1;
    //	y = 1 - 2*r2;
    //	z = 1 - 2*r3;
    //}
    //while ((x*x + y*y + z*z) > 1.0);
    x = 1 - 2*r1;
    y = 1 - 2*r2;
    z = 1 - 2*r3;

    vec3 sample(x, y, z);
    if (sample * alignWithZ < 0)
        sample = -sample;
    sample.normalize();
    return sample;
}

// n = phong exponent (n = 1 for lambertian reflectance)
vec3 RayTracer::importanceSampleUpperHemisphere(vec3& alignWithZ, double n)
{
// Generate random numbers
	double z, phi, theta;
z = random->Random();
phi = random->Random() * 2 * M_PI;
theta = (n == 1 ? acos(sqrt(z)) : acos(pow(z, 1/(n+1))));

// Create vector aligned with z=(0,0,1)
double sintheta = sin(theta);
vec3 sample(sintheta*cos(phi), sintheta*sin(phi), z);

// Rotate sample to be aligned with normal
vec3 t(random->Random(), random->Random(), random->Random());
vec3 u = t ^ alignWithZ; u.normalize();
vec3 v = alignWithZ ^ u;
mat3 rot(u, v, alignWithZ);
rot = rot.transpose();
return rot * sample;
}

rgb RayTracer::raytrace(Ray& ray, unsigned int depth, bool indirect) {

	/* Recursive base case */
	if (depth > settings.recursionDepth)
return rgb::black;

IntersectRecord intersection;

/* We hit nothing */
if (!tracingScene->getHierarchy()->intersect(ray, &intersection))
    return rgb::black;

/*** Else, shade this intersection ***/

Reflectance refl = intersection.primitive->getReflectance(intersection.point);

// Init color to emissive value
rgb pointColor = refl.kE;
	
// Direct illumination
if (refl.type == Diffuse || refl.type == Glossy)
    pointColor += directIllum(ray, intersection);

rgb reflColor;
// Mirror reflection
if ((refl.type == Mirror || refl.type == Dielectric) && depth+1 <= settings.recursionDepth)
{
    reflColor = mirrorReflect(ray, intersection, depth);
    // If not a dieletric, add in this contribution immediately (no need to do Fresnel)
    if (refl.type == Mirror)
        pointColor += reflColor;
}
    
// Refraction Rays
if (refl.type == Dielectric && depth+1 <= settings.recursionDepth)
{
    pointColor += dieletricRefract(ray, intersection, depth, reflColor);
}

return pointColor;
}

rgb RayTracer::directIllum(Ray& ray, IntersectRecord& intersection)
{
    Reflectance refl = intersection.primitive->getReflectance(intersection.point);
    rgb pointColor;
    vector<Light*> lights = tracingScene->getLights();

    // Direct illumination
    for (unsigned int i = 0; i < lights.size(); i++) {
		Ray shadowRay = lights[i]->getShadowRay(intersection.point, settings.rayBias, ray);
		if (!traceShadowRay(shadowRay)) {
			vec3 lightIncidence = shadowRay.getDirection();
			lightIncidence.normalize();
			if (refl.kD != rgb::black)
				pointColor += diffComp(intersection, lightIncidence, lights[i]->getIntensity(intersection.point));
			if (refl.kS != rgb::black)
				pointColor += specComp(intersection, lightIncidence, lights[i]->getIntensity(intersection.point), ray);
    }
}

return pointColor;
}

rgb RayTracer::mirrorReflect(Ray& ray, IntersectRecord& intersection, int depth)
{
	Reflectance refl = intersection.primitive->getReflectance(intersection.point);
rgb reflColor;

rgb mult = ((refl.kR == rgb::black) ? refl.kT : refl.kR);
vec3 rayDirection = ray.getDirection();
vec3 reflectDirection = rayDirection -
    2*(rayDirection * intersection.surfaceNormal) * intersection.surfaceNormal;
Ray bounceRay(intersection.point, settings.rayBias, DBL_MAX, reflectDirection, ray.getSample(), ray.getLastHitPrim());
reflColor = trace(bounceRay, depth + 1);

// Mirror reflection
if (refl.kT == rgb::black)
{
    reflColor = refl.kR * reflColor;
}
// Dielectric reflection
// Use kT for reflection coefficient if kR is not specified (is black)
else
{
		reflColor = mult * reflColor;
}

return reflColor;
}

rgb RayTracer::dieletricRefract(Ray& ray, IntersectRecord& intersection, int depth, rgb reflColor)
{
	Reflectance refl = intersection.primitive->getReflectance(intersection.point);

double index = intersection.primitive->getReflectance(intersection.point).indexOfRefraction;

/* Compute indices of refraction for (from, to) transmission media. */
// CASE: From and object into open air.
double n, nt;
if (ray.getLastHitPrim() == intersection.primitive)
{
    n = index;
    nt = 1.0;
}
    // CASE: From open air into an object.
else
{
    n = 1.0;
    nt = index;
}

/* Now compute the refraction direction (returns false if total internal reflection). */
vec3 refractDirection;
bool refracted = refract(ray, intersection, n, nt, refractDirection);

// Calculate Schlick approximation of Fresnel equations
vec3 d = ray.getDirection(); d.normalize();
vec3 rd = refractDirection; if (refracted) rd.normalize();
double schlick = schlickCalc(n, nt, d, rd, intersection.surfaceNormal);

rgb refrColor;
if (refracted) {
    // Trace refracted ray;
    Ray refractRay(intersection.point, settings.rayBias, DBL_MAX, refractDirection, ray.getSample(), intersection.primitive);
    refrColor = refl.kT * trace(refractRay, depth + 1);
}
// Add up final reflection/refraction color contributions according to Fresnel
return schlick*reflColor + (1-schlick)*refrColor;
}

double RayTracer::schlickCalc(double n, double nt, vec3& rayDir, vec3& refrDir, vec3& surfNorm)
{
    double nbig, nsmall;
    nbig = MAX(n,nt); nsmall = MIN(n,nt);
    double R0 = ((nbig-nsmall)/(nbig+nsmall)); R0 = R0*R0;
    bool into = (rayDir * surfNorm) < 0;
    double c = 1 - (into ? (-rayDir * surfNorm) :
		                   (refrDir * surfNorm));
    double schlick = R0 + (1-R0)* c * c * c * c * c;
    return schlick;
}

vec3 RayTracer::reflect(vec3& dir, const vec3& normal)
{
    return dir - (2 * (dir * normal) * normal);
}

bool RayTracer::traceShadowRay(Ray& ray) {

    IntersectRecord rec;
    return (tracingScene->getHierarchy()->intersect(ray, &rec));
}

rgb RayTracer::diffComp(const IntersectRecord& intersection, const vec3& incidence, const rgb& color) {

    return intersection.primitive->getReflectance(intersection.point).kD * color
		* MAX(intersection.surfaceNormal * incidence, 0);

}

rgb RayTracer::specComp(const IntersectRecord& intersection, const vec3& incidence, const rgb& color, Ray& viewRay) {

    vec3 reflectVec = -incidence + (2 * (incidence * intersection.surfaceNormal) * intersection.surfaceNormal);
    vec3 viewerVec = -viewRay.getDirection();
    viewerVec.normalize();
    double scalarTerm = MAX(reflectVec * viewerVec, 0);
    Reflectance reflec = intersection.primitive->getReflectance(intersection.point);
    return reflec.kS * color * pow(scalarTerm, reflec.pExp);
}

bool RayTracer::refract(Ray& ray, const IntersectRecord& intersect, double oldIndex, double newIndex, vec3& refractDirection) {
    double n = oldIndex/newIndex;
    vec3 direction = ray.getDirection();
    direction.normalize();
    vec3 normal = intersect.surfaceNormal;

    // Checking whether ray is going into the object or going out.
    // Negative dot product means going in, positive means going out (so we need to look at flipped normal)
    double cosAngle = direction * normal.normalize();
    if(cosAngle  > 0) {
        normal = -intersect.surfaceNormal;
    }
    
    double c = direction * normal;
    double cosPhi2 = (1 - ((n * n) * (1 - (c * c))));
    
    // If cos(phi)^2 is less than 0, then no refraction ray exists and all 
    // the energy is reflected (TOTAL INTERNAL REFLECTION).
    if (cosPhi2 < 0) 
        return false;
    else {
        double cosPhi = sqrt(cosPhi2);
        vec3 term1 = n * (direction - normal * (c));
        refractDirection = term1 - normal * cosPhi;
        return true;
    }
}