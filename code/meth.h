#include "misc.h"
#include <emmintrin.h>
#include <math.h>

#ifndef METH
#define METH

#pragma pack(push, 16)

#define pi 3.1415926535897932384626433832795

#define sq(a) ((a)*(a))

struct v2f
{
    union
    {
        struct
        {
            float x;
            float y;
        };
        float data[2];
    };
    
    inline float & operator[](int a)
    {
        return data[a];
    }
};

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
        struct
        {
            float i;
            float j;
            float k;
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
            float i;
            float j;
            float k;
            float r;
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

    inline float & operator[](int a)
    {
        return data[a];
    }
};

struct __attribute__((aligned(16)))  m4x4f
{
    union
    {
        float data[16];
        struct
        {
            v4f r0;
            v4f r1;
            v4f r2;
            v4f r3;
        };
        v4f rows[4];
    };

    inline float & operator[](int a)
    {
        return data[a];
    }
};
#pragma pack(pop)

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

v3f multiply(v3f v, m3x3f m)
{
    v3f product;
    for(int c = 0; c < 3; c++)
    {
        product[c] = 0;
        for(int i = 0; i < 3; i++)
        {
            product[c] += m[c+i*3]*v[i];
        }
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
        2*(q.i*q.k-q.j*q.r)  , 2*(q.j*q.k+q.i*q.r)  , 1-2*sq(q.i)-2*sq(q.j),
    };
    return matrix;
}

inline m4x4f quaternionTo4x4Matrix(v4f q)
{
    m4x4f matrix = {
        1-2*sq(q.j)-2*sq(q.k), 2*(q.i*q.j-q.k*q.r)  , 2*(q.i*q.k+q.j*q.r)  , 0.0,
        2*(q.i*q.j+q.k*q.r)  , 1-2*sq(q.i)-2*sq(q.k), 2*(q.j*q.k-q.i*q.r)  , 0.0,
        2*(q.i*q.k-q.j*q.r)  , 2*(q.j*q.k+q.i*q.r)  , 1-2*sq(q.i)-2*sq(q.j), 0.0,
        0.0                  , 0.0                  , 0.0                  , 1.0,
    };
    return matrix;
}

v4f quaternionMultiply(v4f a, v4f b)
{
    v4f product = {
        a.r*b.i+a.i*b.r+a.j*b.k-a.k*b.j,
        a.r*b.j-a.i*b.k+a.j*b.r+a.k*b.i,
        a.r*b.k+a.i*b.j-a.j*b.i-a.k*b.r,
        a.r*b.r-a.i*b.i-a.j*b.j-a.k*b.k,
    };
    
    return product;
}

v4f quaternionMultiply(v4f a, v3f b)
{
    v4f product = {
        a.r*b.i+a.j*b.k-a.k*b.j,
        a.r*b.j-a.i*b.k+a.k*b.i,
        a.r*b.k+a.i*b.j-a.j*b.i,
        -a.i*b.i-a.j*b.j-a.k*b.k,
    };
    
    return product;
}

inline v3f applyQuaternion(v4f q, v3f p)
{
    return multiply(p, quaternionToMatrix(q));
}

/* inline v3f applyQuaternion(v4f q, v3f p) */
/* { */
/*     union */
/*     { */
/*         v4f product; */
/*         v3f out; */
/*     }; */
/*     product = quaternionMultiply(quaternionMultiply(q, p), inverseQuaternion(q)); */
/*     return out; */
/* } */

//matrix multiplication of aligned matrices a and b
inline m4x4f multiplyA( m4x4f a, m4x4f b)
{
    {
        __m128 a_element = _mm_load1_ps(&a[0]);
        __m128 b_row = _mm_load_ps((float *) b.rows);
        __m128 current_row = _mm_mul_ps(a_element, b_row);
        a_element = _mm_load1_ps(&a[1]);
        b_row = _mm_load_ps((float *) (b.rows+1));
        current_row = _mm_add_ps(current_row, _mm_mul_ps(a_element, b_row));
        a_element = _mm_load1_ps(&a[2]);
        b_row = _mm_load_ps((float *) (b.rows+2));
        current_row = _mm_add_ps(current_row, _mm_mul_ps(a_element, b_row));
        a_element = _mm_load1_ps(&a[3]);
        b_row = _mm_load_ps((float *) (b.rows+3));
        current_row = _mm_add_ps(current_row, _mm_mul_ps(a_element, b_row));
        _mm_store_ps((float *) a.rows, current_row);
    }
    
    {
        __m128 a_element = _mm_load1_ps(&a[4]);
        __m128 b_row = _mm_load_ps((float *) b.rows);
        __m128 current_row = _mm_mul_ps(a_element, b_row);
        a_element = _mm_load1_ps(&a[5]);
        b_row = _mm_load_ps((float *) (b.rows+1));
        current_row = _mm_add_ps(current_row, _mm_mul_ps(a_element, b_row));
        a_element = _mm_load1_ps(&a[6]);
        b_row = _mm_load_ps((float *) (b.rows+2));
        current_row = _mm_add_ps(current_row, _mm_mul_ps(a_element, b_row));
        a_element = _mm_load1_ps(&a[7]);
        b_row = _mm_load_ps((float *) (b.rows+3));
        current_row = _mm_add_ps(current_row, _mm_mul_ps(a_element, b_row));
        _mm_store_ps((float *) (a.rows+1), current_row);
    }
    
    {
        __m128 a_element = _mm_load1_ps(&a[8]);
        __m128 b_row = _mm_load_ps((float *) b.rows);
        __m128 current_row = _mm_mul_ps(a_element, b_row);
        a_element = _mm_load1_ps(&a[9]);
        b_row = _mm_load_ps((float *) (b.rows+1));
        current_row = _mm_add_ps(current_row, _mm_mul_ps(a_element, b_row));
        a_element = _mm_load1_ps(&a[10]);
        b_row = _mm_load_ps((float *) (b.rows+2));
        current_row = _mm_add_ps(current_row, _mm_mul_ps(a_element, b_row));
        a_element = _mm_load1_ps(&a[11]);
        b_row = _mm_load_ps((float *) (b.rows+3));
        current_row = _mm_add_ps(current_row, _mm_mul_ps(a_element, b_row));
        _mm_store_ps((float *) (a.rows+2), current_row);
    }
    
    {
        __m128 a_element = _mm_load1_ps(&a[12]);
        __m128 b_row = _mm_load_ps((float *) b.rows);
        __m128 current_row = _mm_mul_ps(a_element, b_row);
        a_element = _mm_load1_ps(&a[13]);
        b_row = _mm_load_ps((float *) (b.rows+1));
        current_row = _mm_add_ps(current_row, _mm_mul_ps(a_element, b_row));
        a_element = _mm_load1_ps(&a[14]);
        b_row = _mm_load_ps((float *) (b.rows+2));
        current_row = _mm_add_ps(current_row, _mm_mul_ps(a_element, b_row));
        a_element = _mm_load1_ps(&a[15]);
        b_row = _mm_load_ps((float *) (b.rows+3));
        current_row = _mm_add_ps(current_row, _mm_mul_ps(a_element, b_row));
        _mm_store_ps((float *) (a.rows+3), current_row);
    }
    
    return a;
}

//NOTE: this is fast for small values of a, there are better algorithms for random 32 bit integers
//ceiling of the log base 2
inline uint clog_2(uint a)
{
    uint log = 0;
    for(; a; log++)
    {
        a >>= 1;
    }
    return log;
}

/* inline float32 floor(float32 x) */
/* { */
/*     __m128 floor_x = _mm_set_ss(x); */

/*     //floor_x = _mm_round_ps(floor_x, _MM_FROUND_TO_NEG_INF);//TODO: use roundps when available */
/*     floor_x = _mm_cvtepi32_ps(_mm_cvttps_epi32(floor_x)); */
    
/*     float32 out; */
/*     _mm_store_ss(&out, floor_x); */
/*     return out; */
/* } */

inline float mod(float x, float base)
{
    return x-floor(x/base)*base;
}

#endif
