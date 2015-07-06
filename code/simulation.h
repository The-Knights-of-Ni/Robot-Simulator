#include "meth.h"
#include "misc.h"

struct box
{
    v3f upper;
    v3f lower;
};

struct mesh
{
    union
    {
        float * coords;
        v3f * verts;
    };
    uint n_verts;
    
    uint * indecies;
    uint n_indecies;
    
    uint * convex_indecies; //sorted starting with the lowest x
    int * convex_starts;
    int * convex_ends;
    box * bounding_boxes;
    uint n_convex_groups;
};

struct physics_object
{
    int mesh_id;
    v4f orientation; //this is a quaternion
    v3f position;

    float mass;
    m3x3f inertia;
    
    v3f angular_velocity;
    v3f velocity;
};

void generateConvexGroups()
{
    
}

struct support_return
{
    v3f point;
    float distance;
};

//GJK support function for one convex object
inline support_return support(physics_object a, int a_group, mesh * mesh_list, v3f dir)
{
    dir = applyQuaternion(inverseQuaternion(a.orientation), dir);
    
    mesh & a_mesh = mesh_list[a.mesh_id];
    
    support_return highest = {
        a_mesh.verts[a_mesh.convex_indecies[a_group]],
        dot(a_mesh.verts[a_mesh.convex_indecies[a_group]], dir)
    };
    
    for(uint v = a_mesh.convex_starts[a_group]; v < a_mesh.convex_ends[a_group]; v++)
    {
        float proj = dot(a_mesh.verts[a_mesh.convex_indecies[v]], dir);
        if(proj > highest.distance)
        {
            highest.distance = proj;
            highest.point = a_mesh.verts[a_mesh.convex_indecies[v]];
        }
    }
    
    highest.point = add(highest.point, applyQuaternion(a.orientation, a.position));
    highest.distance += dot(a.position, dir);
    
    return highest;
}

//GJK support function for the Minkowski differnce of two convex polygonal objects
inline support_return support(physics_object a, int a_group, physics_object b, int b_group, mesh * mesh_list, v3f dir)
{
    support_return a_support = support(a, a_group, mesh_list, dir);
    support_return b_support = support(b, b_group, mesh_list, negative(dir));

    support_return out = {
        sub(a_support.point, b_support.point),
        a_support.distance - b_support.distance
    };
    return out;
}

bool8 doSimplex(v3f * simplex, int & n_simplex_points, v3f & dir)
{
    assert(n_simplex_points > 2);
    
    if(n_simplex_points == 3)
    {
        v3f axis = cross(sub(simplex[0], simplex[2]), sub(simplex[1], simplex[2]));
        v3f norm0 = cross(axis, sub(simplex[0], simplex[2]));
        v3f norm1 = cross(sub(simplex[0], simplex[2]), axis);
        v3f to_origin = negative(simplex[2]);
        if(dot(norm0, to_origin) < 0)
        {
            if(dot(norm1, to_origin) < 0)
            { //middle
                if(dot(axis, to_origin))
                {
                    //top
                    dir = axis;
                }
                else
                {
                    //bottom
                    dir = negative(axis);
                }
            }
            //0-2 side
            dir = rejection(to_origin, sub(simplex[0], simplex[2]));
            simplex[1] = simplex[--n_simplex_points];
        }
        else
        {
            if(dot(norm1, to_origin) > 0)
            { //1-2 side
                dir = rejection(to_origin, sub(simplex[1], simplex[2]));
                simplex[0] = simplex[--n_simplex_points];
            }
            assert(0 && "You broke math");
        }
    }
    else
    {
        v3f norm0 = cross(sub(simplex[0], simplex[3]), sub(simplex[1], simplex[3]));
        v3f norm1 = cross(sub(simplex[1], simplex[3]), sub(simplex[2], simplex[3]));
        v3f norm2 = cross(sub(simplex[2], simplex[3]), sub(simplex[0], simplex[3]));
        v3f to_origin = negative(simplex[3]);
        if(dot(norm0, to_origin) < 0)
        {
            if(dot(norm1, to_origin) < 0)
            {
                if(dot(norm2, to_origin) < 0)
                { //inside
                    return true;
                }
                else
                {
                    dir = norm2;
                    simplex[1] = simplex[--n_simplex_points];
                }
            }
            else if(dot(norm2, to_origin) < 0)
            {
                dir = norm1;
                simplex[0] = simplex[--n_simplex_points];
            }
            else
            {
                dir = rejection(to_origin, sub(simplex[2], simplex[3]));
                simplex[1] = simplex[--n_simplex_points];
                simplex[0] = simplex[--n_simplex_points];
            }
        }
        else if(dot(norm1, to_origin) < 0)
        {
            if(dot(norm2, to_origin) < 0)
            {
                dir = norm0;
                simplex[2] = simplex[--n_simplex_points];
            }
            else
            {
                dir = rejection(to_origin, sub(simplex[0], simplex[3]));
                simplex[2] = simplex[--n_simplex_points];
                simplex[1] = simplex[--n_simplex_points];
            }
        }
        else if(dot(norm2, to_origin) < 0)
        {
            dir = rejection(to_origin, sub(simplex[1], simplex[3]));
            simplex[2] = simplex[--n_simplex_points];
            simplex[0] = simplex[--n_simplex_points];
        }
        assert(0 && "origin is path the new simplex point");
    }
    
    return false;
}

inline v3f colliding(physics_object a, physics_object b, mesh * mesh_list)
{
    mesh & a_mesh = mesh_list[a.mesh_id];
    mesh & b_mesh = mesh_list[b.mesh_id];
    
    float nearest = -1.0;
    
    for(int a_group = 0; a_group < a_mesh.n_convex_groups; a_group++)
    {
        for(int b_group = 0; b_group < b_mesh.n_convex_groups; b_group++)
        {//TODO: possible use bounding boxes to speed this up
            v3f dir = {1.0, 0.0, 0.0};
            v3f simplex[4];
            int n_simplex_points = 0;
            
            support_return initial_support = support(a, a_group, b, b_group, mesh_list, dir);
            if(initial_support.distance < 0) continue;
            simplex[n_simplex_points++] = initial_support.point;
            dir = negative(simplex[0]);
            
            //the 2 point case can only happen once
            support_return second_support = support(a, a_group, b, b_group, mesh_list, dir);
            if(second_support.distance < 0) continue;
            simplex[n_simplex_points++] = second_support.point;
            dir = rejection(negative(simplex[0]), sub(simplex[0], simplex[1]));
            
            for ever
            {
                support_return new_support = support(a, a_group, b, b_group, mesh_list, dir);
                if(new_support.distance < 0) break;
                simplex[n_simplex_points++] = new_support.point;
                
                if(doSimplex(simplex, n_simplex_points, dir))
                { //intersection found
                    return true;
                }
            }
            //these two conxex hulls do not intersect
            if()
            for()
        }
    }
    
    return false;
}

//TODO: swept objects
