#ifndef _VEC3_H
#define _VEC3_H

enum COORD {X = 0, Y, Z};

class vec3 {
    public:
        vec3 (float a);
        vec3 (float x, float y, float z);
        ~vec3 ();

        vec3 operator+ (const vec3& a) const;
        vec3& operator+= (const vec3& a);
        vec3 operator- (const vec3& a) const;
        vec3& operator-= (const vec3& a);
        vec3 operator* (float s) const;
        vec3& operator*= (float s);
        friend float dot (const vec3& a, const vec3& b);
        friend vec3 cross (const vec3& a, const vec3& b);
        friend vec3 normalize (const vec3& a);
        friend float length (const vec3& a);
        float x() const;
        float y() const;
        float z() const;

    private:
        float _data[3];
};

#endif
