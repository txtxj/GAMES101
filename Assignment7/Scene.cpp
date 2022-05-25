//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
	// TODO Implement Path Tracing Algorithm here
	Intersection r = intersect(ray);
	if (!r.happened)
	{
		return {0, 0, 0};
	}
	if (r.emit.norm() > 0)
	{
		return {1, 1, 1};
	}
	Vector3f L_dir = 0, L_indir = 0;
	Vector3f p = r.coords;
	Vector3f wo = normalize(-ray.direction);
	Intersection inter;
	float pdf_light;
	sampleLight(inter, pdf_light);
	Vector3f x = inter.coords;
	Vector3f dir = x - p;
	float dis = dir.norm();
	Vector3f ws = normalize(dir);
	Vector3f N = r.normal;
	Vector3f NN = inter.normal;
	Vector3f emit = inter.emit;
	Intersection i = intersect(Ray(p, ws));
	if ((i.coords - x).norm() < EPSILON)
	{
		L_dir = emit * r.m->eval(wo, ws, N) * dotProduct(ws, N) * dotProduct(-ws, NN) / (dis * dis * pdf_light);
	}

	if (get_random_float() < RussianRoulette)
	{
		Vector3f wi = r.m->sample(wo, N);
		L_indir = castRay(Ray(p, wi), depth) * r.m->eval(wi, wo, N) * dotProduct(wi, N) / (r.m->pdf(wo, wi, N) * RussianRoulette);
	}
	return L_dir + L_indir;
}