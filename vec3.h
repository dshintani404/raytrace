#include<math.h>
#include<stdlib.h>

class vec3 {
public:
    vec3() {}
    float e[3];
    vec3(float e0, float e1, float e2) { e[0]=e0; e[1]=e1; e[2]=e2; }
    inline float x() const {return e[0];}
    inline float y() const {return e[1];}
    inline float z() const {return e[2];}
    inline float r() const {return e[0];}
    inline float g() const {return e[1];}
    inline float b() const {return e[2];}
    
    inline float operator[](int i) const { return e[i]; }
    
    inline vec3& operator+=(const vec3& v2);
    inline vec3& operator/=(const float t);

    inline float length() const {
        return sqrt(e[0]*e[0]+e[1]*e[1]+e[2]*e[2]);
    }
};

inline float dot(const vec3 &v1, const vec3 &v2) {
     return v1.e[0] * v2.e[0] + v1.e[1] * v2.e[1] + v1.e[2] * v2.e[2];
}

inline vec3 operator+(const vec3 &v1, const vec3 &v2) {
    return vec3(v1[0] + v2[0], v1[1] + v2[1], v1[2] + v2[2]);
}

inline vec3 operator-(const vec3 &v1, const vec3 &v2) {
    return vec3(v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2]);
}

inline vec3 operator*(float t, const vec3 &v) {
    return vec3(t * v.e[0], t * v.e[1], t * v.e[2]); 
}
    
inline vec3 operator*(const vec3 &v, float t) {
    return vec3(t * v.e[0], t * v.e[1], t * v.e[2]); 
}

inline vec3 operator*(const vec3 &v1, const vec3 &v2) {
    return vec3(v1.e[0] * v2.e[0], v1.e[1] * v2.e[1], v1.e[2] * v2.e[2]); 
}

inline vec3 operator/(const vec3 &v, float t) {
    return vec3(v.e[0]/t, v.e[1]/t, v.e[2]/t); 
}
 
inline vec3 unit_vector(const vec3 &v) {
    return v/v.length();
}

inline vec3& vec3::operator+=(const vec3& v) {
    e[0] += v[0];
    e[1] += v[1];
    e[2] += v[2];
    return *this;
}

inline vec3& vec3::operator/=(const float t) {
    float k = 1.0 / t;
    e[0] *= k;
    e[1] *= k;
    e[2] *= k;
    return *this;
}

