#ifndef SPHEREH
#define SPHEREH

#include "hitable.h"

class sphere: public hitable {
    public:
        sphere(){};
        sphere(vec3 cen, float r) {center = cen; radius = r;};
        sphere(vec3 cen, float r, material* mat) {center = cen; radius = r; material = mat;};
       
        virtual bool hit(const ray& r, float tmin, float tmax, hit_record& rec) const;
        virtual bool bounding_box(float t0, float t1, aabb& box) const;

        vec3 center;
        float radius;
        material* material;
};

bool sphere::hit(const ray& r, float t_min, float t_max, hit_record& rec) const {
    vec3 oc = r.origin() - center;
    float a = dot(r.direction(), r.direction());
    float b = dot(oc, r.direction());
    float c = dot(oc, oc) - radius * radius;
    float D = b*b - a*c;
    if(D>0){
        float temp = (-b-sqrt(D))/a;
        if (temp < t_max && temp > t_min) {
            rec.t = temp;
            rec.p = r.point_at_parameter(rec.t);
            rec.normal = (rec.p-center)/radius;
            rec.mat_ptr = material;
            return true;
        }
        temp = (-b+sqrt(D))/a;
        if (temp < t_max && temp > t_min) {
            rec.t = temp;
            rec.p = r.point_at_parameter(rec.t);
            rec.normal = (rec.p-center)/radius;
            rec.mat_ptr = material;
            return true;
        }
    }
    return false;
}

bool sphere::bounding_box(float t0, float t1, aabb& box) const {
    box = aabb(center - vec3(radius, radius, radius), center + vec3(radius, radius, radius));
    return true;
}

class moving_sphere: public hitable {
    public:
        moving_sphere() {}
        moving_sphere(vec3 cen0, vec3 cen1, float t0, float t1, float r, material* m) : 
            center0(cen0), center1(cen1), time0(t0), time1(t1), radius(r), material(m) {};
        
        virtual bool hit(const ray& r, float t_min, float t_max, hit_record& rec) const;
        virtual bool bounding_box(float t0, float t1, aabb& box) const;
        vec3 center(float time) const;
        
        vec3 center0, center1;
        float time0, time1, radius;
        material* material;
};

vec3 moving_sphere::center(float time) const {
    return center0 + (time-time0) * (center1-center0) / (time1-time0);
}

bool moving_sphere::hit(const ray& r, float t_min, float t_max, hit_record& rec) const {
    vec3 oc = r.origin() - center(r.time());
    float a = dot(r.direction(), r.direction());
    float b = dot(oc, r.direction());
    float c = dot(oc, oc) - radius * radius;
    float D = b*b - a*c;
    if(D>0){
        float temp = (-b-sqrt(D))/a;
        if (temp < t_max && temp > t_min) {
            rec.t = temp;
            rec.p = r.point_at_parameter(rec.t);
            rec.normal = (rec.p-center(r.time()))/radius;
            rec.mat_ptr = material;
            return true;
        }
        temp = (-b+sqrt(D))/a;
        if (temp < t_max && temp > t_min) {
            rec.t = temp;
            rec.p = r.point_at_parameter(rec.t);
            rec.normal = (rec.p-center(r.time()))/radius;
            rec.mat_ptr = material;
            return true;
        }
    }
    return false;
}

bool moving_sphere::bounding_box(float t0, float t1, aabb& box) const {
    aabb box_t0 = aabb(center0 - vec3(radius, radius, radius), center0 + vec3(radius, radius, radius));
    aabb box_t1 = aabb(center1 - vec3(radius, radius, radius), center1 + vec3(radius, radius, radius)); 
    box = surrounding_box(box_t0, box_t1); 
    return true;
}

#endif
