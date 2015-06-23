#include "misc.h"

struct mesh
{
    union
    {
        float * coords;
        v3f * verts;
    }
    uint n_verts;
    
    uint * indecies;
    uint n_indecies;
    
    int * convex_groups;
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
        a_mesh.verts[a_mesh.convex_groups[a_group]],
        dot(a_mesh.verts[a_mesh.convex_groups[a_group]], dir)
    };
    
    for(uint v = a_mesh.convex_groups[a_group]+1; v < a_mesh.convex_groups[a_group+1]; v++)
    {
        proj = dot(a_mesh.verts[v], dir);
        if(proj > highest.distance)
        {
            highest.distance = proj;
            highest.point = a_mesh.verts[v];
        }
    }
    
    highest.point = add(highest.point, applyQuaternion(a.orientation, a.position));
    highest.distance += dot(a.position, dir);
    
    return highest;
}

//GJK support function for the Minkowski differnce of two convex polygonal objects
inline v3f support(physics_object a, int a_group, physics_object b, int b_group, mesh * mesh_list, v3f dir)
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
        
    }
    else
    {
        
    }
}

//returns the number of collision points
inline int collisionPoints(physics_object a, physics_object b, mesh * mesh_list, v3f * out)
{
    int n_collision_points = 0;
    
    mesh & a_mesh = mesh_list[a.mesh_id];
    mesh & b_mesh = mesh_list[b.mesh_id];

    for(int a_group = 0; a_group < a_mesh.n_convex_groups; a_group++)
    {
        for(int b_group = 0; b_group < b_mesh.n_convex_groups; b_group++)
        {
            v3f dir = {1.0, 0.0, 0.0};
            v3f simplex[4];
            int n_simplex_points = 0;
            
            support_return initial_support = support(a, a_group, b, b_group, mesh_list, dir);
            if(initial_support.distance < 0) continue;
            simplex[n_simplex_points++] = initial_support.point;
            dir = negative(dir);
            
            //the 2 point case can only happen once
            support_return second_support = support(a, a_group, b, b_group, mesh_list, dir);
            if(second_support.distance < 0) continue;
            simplex[n_simplex_points++] = second_support.point;
            dir = rejection(sub(simplex[0], origin), sub(simplex[0], simplex[1]));
            
            for ever
            {
                support_return new_support = support(a, a_group, b, b_group, mesh_list, dir);
                if(new_support.distance < 0) break;
                simplex[n_simplex_points++] = new_support.point;
                
                if(doSimplex(simplex, n_simplex_points, dir))
                {
                    out[n_collision_points++] = ;
                    break;
                }
            }
        }
    }
    
    return n_collision_points;
}

//TODO: swept objects
