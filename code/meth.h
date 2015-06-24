#include "misc.h"
#include <math.h>

#define sq(a) ((a)*(a))

struct v3f
{
    union
    {
        struct
        {
            float x;
            float y;
            float z;
        };
        float data[3];
    };

    inline float & operator[](int a)
    {
        return data[a];
    }
};

struct v4f
{
    union
    {
        struct
        {
            float x;
            float y;
            float z;
            float w;
        };
        struct
        { //quaternion notation
            float r;
            float i;
            float j;
            float k;
        };
        float data[4];
    };

    inline float & operator[](int a)
    {
        return data[a];
    }
};

struct m3x3f
{
    union
    {
        float data[9];
        struct
        {
            v3f r0;
            v3f r1;
            v3f r2;
        };
        v3f rows[3];
    };
};

v3f negative(v3f a)
{
    v3f neg = {-a[0],-a[1],-a[2]};
    return neg;
}

v3f scale(v3f v, float s)
{
    v3f product = {v[0]*s,v[1]*s,v[2]*s};
    return product;
}

float dot(v3f a, v3f b)
{
    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
}

v3f cross(v3f a, v3f b)
{
    v3f product = {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};
    return product;
}

v3f add(v3f a, v3f b)
{
    v3f sum = {a[0]+b[0],a[1]+b[1],a[2]+b[2]};
    return sum;
}

v3f sub(v3f a, v3f b)
{
    v3f sum = {a[0]-b[0],a[1]-b[1],a[2]-b[2]};
    return sum;
}

inline float32 invsqrt(float32 a)
{
    uint32 ai = *(uint32 *) &a;
    union
    {
        uint32 isqrti;
        float32 isqrt;
    };
    isqrti = 0x5f375a86 - (ai >> 1); //magic meth
    isqrt *= (1.5f - (a*0.5f*isqrt*isqrt));
    return isqrt;
}

inline float invNorm(v3f a)
{
    float square = dot(a, a);
    return invsqrt(square);
}

inline v3f normalize(v3f a)
{
    return scale(a, invNorm(a));
}

v3f projection(v3f a, v3f b)
{
    return scale(b, dot(a, b)/dot(b, b));
}

v3f rejection(v3f a, v3f b)
{
    return sub(a, projection(a, b));
}

v3f multiply(m3x3f m, v3f v)
{
    v3f product;
    for(int r = 0; r < 3; r++)
    {
        product[r] = dot(m.rows[r], v);
    }
    return product;
}

inline v4f inverseQuaternion(v4f q)
{
    q.r = -q.r;
    return q;
}

inline m3x3f quaternionToMatrix(v4f q)
{
    m3x3f matrix = {
        1-2*sq(q.j)-2*sq(q.k), 2*(q.i*q.j-q.k*q.r)  , 2*(q.i*q.k+q.j*q.r)  ,
        2*(q.i*q.j+q.k*q.r)  , 1-2*sq(q.i)-2*sq(q.k), 2*(q.j*q.k-q.i*q.r)  ,
        2*(q.i*q.k+q.j*q.r)  , 2*(q.j*q.k+q.i*q.r)  , 1-2*sq(q.i)-2*sq(q.j),
    };
    return matrix;
}

inline v3f applyQuaternion(v4f q, v3f p)
{
    return multiply(quaternionToMatrix(q), p);
}
