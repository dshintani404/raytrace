#include<iostream>
#include "hitable_list.h"
#include "float.h"
#include "camera.h"
#include "material.h"
#include "sphere.h"

vec3 color(const ray& r, hitable_list* world, int depth) {
    hit_record rec;
    if(world->hit(r, 0.001, MAXFLOAT, rec)){
        ray scattered;
        vec3 attenuation;
        if(depth < 50 && rec.mat_ptr->scatter(r, rec, attenuation, scattered)){
            return attenuation* color(scattered, world, depth+1);
        }
        return vec3(0,0,0);
    }
    vec3 unit_direction = unit_vector(r.direction());
    float t = 0.5 * (unit_direction.y() + 1.0);
    return (1.0-t)*vec3(1.0, 1.0, 1.0) + t*vec3(0.5, 0.7, 1.0);
}

hitable_list* random_scene(){
  int n = 50000;
  hitable** list = new hitable*[n+1];
    
  list[0] = new sphere(
      vec3(0, -1000, 0), 1000, new lambertian(new checker_texture(
          new constant_texture(vec3(0.2, 0.3, 0.1)), new constant_texture(vec3(0.9, 0.9, 0.9))
          )
        )
      );

  int i = 1;
  for (int a=-10;a<10;a++) {
    for (int b=-10;b<10;b++) {
      float choose_mat = drand48();
      vec3 center(a+0.9*drand48(), 0.2, b+0.9*drand48());
      if ((center-vec3(4.0 ,0.2, 0.0)).length() > 0.9) {
        if (choose_mat < 0.8) {
          list[i++] = new moving_sphere(
              center, center + vec3(0, 0.5 * drand48(), 0), 0.0, 1.0, 0.2,
              new lambertian(new constant_texture(
                  vec3(drand48()*drand48(),drand48()*drand48(),drand48()*drand48()))
                )
              );          
        } else if(choose_mat < 0.95) {
          list[i++] = new sphere(
              center, 0.2, new metal(
                vec3(0.5*(1+drand48()),0.5*(1+drand48()),0.5*(1+drand48())), 0.5*(1+drand48()))
              );
        } else {
          list[i++] = new sphere(center, 0.2, new dielectric(1.5));
        }
      }
    }
  }

  list[i++] = new sphere(vec3(0, 1, 0), 1.0, new dielectric(1.5));
  list[i++] = new sphere(
      vec3(-4, 1, 0), 1.0, new lambertian(new constant_texture(vec3(0.4,0.2, 0.1)))
      );
  list[i++] = new sphere(vec3(4, 1, 0), 1.0, new metal(vec3(0.7, 0.6, 0.5), 0.0));
  return new hitable_list(list, i);
}

hitable_list* two_sphere() {
  texture* checker = new checker_texture(
      new constant_texture(vec3(0.2, 0.3, 0.1)), new constant_texture(vec3(0.9, 0.9, 0.9)) 
      );
  int n = 50;
  hitable** list = new hitable*[n+1];
  list[0] = new sphere(vec3(0, -10, 0), 10, new lambertian(checker));
  list[1] = new sphere(vec3(0, 10, 0), 10, new lambertian(checker));

  return new hitable_list(list, 2);
}

int main()
{
    int nx = 200;
    int ny = 100;
    int ns = 100;
    std::cout << "P3\n" << nx << " " << ny << "\n255\n";
    vec3 origin(0.0, 0.0, 0.0);
    vec3 lower_left_corner(-2.0, -1.0, -1.0);
    vec3 horizontal(4.0, 0.0, 0.0);
    vec3 vertical(0.0, 2.0, 0.0);

    hitable_list* world = two_sphere(); 
    
    vec3 lookfrom(13,2,3);
    vec3 lookat(0,0,0);
    float dist_to_focus = 10.0;
    float aperture = 0.0;
    camera cam(lookfrom, lookat, vec3(0,1,0), 20, float(nx)/float(ny), aperture, dist_to_focus, 0.0, 1.0);

    for(int j = ny-1; j >= 0; j--) {
        for(int i = 0; i < nx; i++) {
            vec3 col(0,0,0);
            for(int s=0;s<ns;s++){
                float u = float(i + drand48()) / float(nx);
                float v = float(j + drand48()) / float(ny);
                ray r = cam.get_ray(u,v);
                col += color(r, world, 0);        
            }
            col /= float(ns);
            
            col =vec3(sqrt(col[0]), sqrt(col[1]), sqrt(col[2]));
            int ir = int(255.99 * col[0]);
            int ig = int(255.99 * col[1]);
            int ib = int(255.99 * col[2]);

            std::cout << ir << " " << ig << " " << ib << "\n"; 
        }
   }
}

