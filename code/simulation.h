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

/* #define max_edges 8 */
/* struct vert_edges */
/* { */
/*     uint edges[max_edges]; */
/*     uint n_edges; */
/* }; */

/* void addEdge(vert_edges * graph, uint v0, uint v1) */
/* { */
/*     if(n_edges == max_edges) printf("error, vertex v0 has more than %d connections", max_edges); */
/*     for(int i = 0; i < graph[v0].n_edges++; i++) if(graph[v0].edges[i] == v1) return; */
/*     graph[v0].edges[graph[v0].n_edges++] = v1; */
/* } */

struct support_return
{
    v3f point;
    float distance;
};

inline support_return convexHullSupport(v3f * connection_points, uint n_connection_points, v3f point, v3f dir)
{
    support_return out = {connection_points[0], dot(connection_points[0], dir)};
    
    for(int i = 1; i < n_connection_points; i++)
    {
        float dot_product = dot(connection_points[i], dir);
        if(dot_product > out.distance)
        {
            out.point = connection_points[i];
            out.distance = dot_product;
        }
    }

    out.point = sub(out.point, point);
    out.distance = dot(out.point, dir);
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

typedef struct
{
    uint left_edge;
    uint right_edge;
} convex_hull_range;

//m.convex_indecies and such must be valid pointers be valid pointers
void convexHull(mesh m, uint start_index, uint end_index, void * free_memory)
{
    /* vert_edges * graph = free_memory; */
    /* free_memory = (void *) (byte *) free_memory + sizeof(vert_edges)*m.n_verts*/
    
    /* for(int i = 0; i < m,n_indecies; i+=3) */
    /* { */
    /*     graph[m.indecies[i+0]].n_edges = 0; */
    /*     addEdge(graph, m.indecies[i+0], m.indecies[i+1]); */
    /*     addEdge(graph, m.indecies[i+0], m.indecies[i+2]); */
                                                                                      
    /*     graph[m.indecies[i+1]].n_edges = 0;               */
    /*     addEdge(graph, m.indecies[i+1], m.indecies[i+0]); */
    /*     addEdge(graph, m.indecies[i+1], m.indecies[i+2]); */
                                                                                      
    /*     graph[m.indecies[i+2]].n_edges = 0;               */
    /*     addEdge(graph, m.indecies[i+2], m.indecies[i+0]); */
    /*     addEdge(graph, m.indecies[i+2], m.indecies[i+1]); */
    /* } */

    uint n_hull_verts = end_index-start_index;
    
    //radix sort TODO: prefetch
    uint histograms[3*(2<<11)];
    
    for(uint i = start_index; i < end_index; i++)
    {
        union
        {
            float value;
            uint32 value_bits;
        };
        
        value = m.coords[i*3];
        value_bits ^= 0x80000000|(-(value_bits>>31));
        for(int h = 0; h < 3; h++) histograms[h*(2<<11) + (value_bits>>(h*11) & 0x7FF)]++;
    }
    
    uint offset_tables[3*(2<<11)];
    
    for(int i = 1; i < 2<<11; i++)
    {
        for(int h = 0; h < 3; h++) offset_tables[i+h*(2<<11)] += histograms[i-1+h*(2<<11)]+offset_tables[i-1+h*(2<<11)];
    }
    
    uint * hull_indecies = m.convex_indecies + m.convex_ends[m.n_convex_groups-1];
    uint * sorted_indecies = (uint *) ((char *) free_memory);
    free_memory = (void *)((byte *) free_memory + sizeof(uint)*n_hull_verts);
    
    for(uint i = start_index; i < end_index; i++)
    {
        union
        {
            float value;
            uint32 value_bits;
        };
        
        value = m.coords[i*3];
        value_bits ^= 0x80000000|(-(value_bits>>31));

        hull_indecies[offset_tables[value_bits & 0x7FF]++] = i;
    }
    
    for(uint i = 0; i < end_index-start_index; i++)
    {
        union
        {
            float value;
            uint32 value_bits;
        };
        
        value = m.coords[3*hull_indecies[i]];
        value_bits ^= 0x80000000|(-(value_bits>>31));
        
        sorted_indecies[offset_tables[((value_bits>>11) & 0x7FF)+(2<<11)]++] = hull_indecies[i];
    }
    
    for(uint i = 0; i < end_index-start_index; i++)
    {
        union
        {
            float value;
            uint32 value_bits;
        };
        
        value = m.coords[3*sorted_indecies[i]];
        value_bits ^= 0x80000000|(-(value_bits>>31));
        
        hull_indecies[offset_tables[((value_bits>>22) & 0x7FF)+2*(2<<11)]++] = sorted_indecies[i];
    }
    //hull_indecies is sorted by increasing x
    
    convex_hull_range * ranges = (convex_hull_range *) free_memory;
    ranges[0].left_edge = 0;
    ranges[0].left_edge = n_hull_verts;
    uint n_ranges = 1;
    
    { //connect the two halves and remove useless points
        void * temp_free_memory = (void *)((convex_hull_range *) free_memory + n_ranges);
        
        uint left_edge = ranges[--n_ranges].left_edge;
        uint right_edge = ranges[n_ranges].right_edge;
        
        uint middle_vert = (left_edge+right_edge+1)/2; //TODO: loop over the sub-endpoints from a stack
    
        uint left_bridge_vert = middle_vert-1;
        uint right_bridge_vert = middle_vert;
    
        {//initialize bridge verts to the highest verts on each side
            int i = left_edge;
            for(; i < middle_vert; i++)
            {
                if(m.coords[1+3*hull_indecies[i]] >= m.coords[1+3*hull_indecies[left_bridge_vert]]) left_bridge_vert = i;
            }
            for(; i < right_edge; i++)
            {
                if(m.coords[1+3*hull_indecies[i]] > m.coords[1+3*hull_indecies[right_bridge_vert]]) right_bridge_vert = i;
            }
        }
        
        for ever
        {//check if there are better candidates for the bridge start
            float test_normal_x = m.coords[1+3*hull_indecies[left_bridge_vert]] - m.coords[1+3*hull_indecies[right_bridge_vert]];
            float test_normal_y = m.coords[3*hull_indecies[right_bridge_vert]] - m.coords[3*hull_indecies[left_bridge_vert]];
        
            float current_left_dot = test_normal_x*m.coords[3*hull_indecies[left_bridge_vert]]
                + test_normal_y*m.coords[1+3*hull_indecies[left_bridge_vert]];
            float current_right_dot = test_normal_x*m.coords[3*hull_indecies[right_bridge_vert]]
                + test_normal_y*m.coords[1+3*hull_indecies[right_bridge_vert]];
            
            int i = left_edge;
            for(; i < middle_vert; i++)
            {
                if((test_normal_x*m.coords[3*hull_indecies[i]]
                    + test_normal_y*m.coords[1+3*hull_indecies[i]])
                   >= current_left_dot)
                {
                    left_bridge_vert = i;
                    
                    test_normal_x = m.coords[1+3*hull_indecies[left_bridge_vert]] - m.coords[1+3*hull_indecies[right_bridge_vert]];
                    test_normal_y = m.coords[3*hull_indecies[right_bridge_vert]] - m.coords[3*hull_indecies[left_bridge_vert]];
                    current_left_dot = test_normal_x*m.coords[3*hull_indecies[left_bridge_vert]]
                        + test_normal_y*m.coords[1+3*hull_indecies[left_bridge_vert]];
                    current_right_dot = test_normal_x*m.coords[3*hull_indecies[right_bridge_vert]]
                        + test_normal_y*m.coords[1+3*hull_indecies[right_bridge_vert]];
                    //no need to restart the search since any of the
                    //points that failed the test for the old left bridge
                    //point will also fail the new test
                }
            }
            for(; i < right_edge; i++)
            {
                if(i >= n_hull_verts) goto stop_searching_for_bridge;
                if((test_normal_x*m.coords[3*hull_indecies[i]]
                    + test_normal_y*m.coords[1+3*hull_indecies[i]])
                   >= current_right_dot)
                {
                    right_bridge_vert = i;
                    break;
                    //need to restart the search here since all the left
                    //verts need to be rechecked
                }
            }
        }
    stop_searching_for_bridge:;
        
        //gift wrap connection
        v3f current_axis = sub(m.verts[right_bridge_vert], m.verts[left_bridge_vert]);
        v3f ccw_direction = {-current_axis.z, 0.0, current_axis.x};//cross(current_axis, v3f(0, 1, 0));
        uint last_left_vert = left_bridge_vert;
        uint last_right_vert = right_bridge_vert;
        uint next_vert;
        
        uint inmost_left_vert = left_bridge_vert;
        uint inmost_right_vert = right_bridge_vert;
        uint outmost_left_vert = left_bridge_vert;
        uint outmost_right_vert = right_bridge_vert;

        
        v3f * connection_points = (v3f *) temp_free_memory;
        uint n_connection_points = 0;
        
        for ever
        {
            float highest_dot = -1.0;
            for(int i = 0; i < n_hull_verts; i++)
            {
                float current_dot = dot(ccw_direction, rejection(normalize(sub(m.verts[hull_indecies[i]], m.verts[hull_indecies[last_right_vert]])), current_axis));
                if(current_dot > highest_dot)
                {
                    highest_dot = current_dot;
                    next_vert = i;
                }
            }
            
            if((next_vert == left_bridge_vert && last_right_vert == right_bridge_vert)
               || (next_vert == right_bridge_vert && last_left_vert == left_bridge_vert))
            {
                break;
            }
            
            if(m.coords[3*hull_indecies[next_vert]] > m.coords[3*hull_indecies[inmost_left_vert]])
            {
                inmost_left_vert = next_vert;
            }
            
            if(m.coords[3*hull_indecies[next_vert]] < m.coords[3*hull_indecies[inmost_right_vert]])
            {
                inmost_right_vert = next_vert;
            }
            
            if(m.coords[3*hull_indecies[next_vert]] < m.coords[3*hull_indecies[outmost_left_vert]])
            {
                outmost_left_vert = next_vert;
            }
            
            if(m.coords[3*hull_indecies[next_vert]] > m.coords[3*hull_indecies[outmost_right_vert]])
            {
                outmost_right_vert = next_vert;
            }
            
            connection_points[n_connection_points++] = m.verts[hull_indecies[next_vert]];
            //the normal should face outward
            
            if(next_vert < middle_vert)
            { //left side
                v3f new_axis = normalize(sub(m.verts[hull_indecies[next_vert]], m.verts[hull_indecies[last_right_vert]]));
                ccw_direction = rejection(new_axis, current_axis);
                current_axis = new_axis;
                
                last_left_vert = next_vert;
            }
            else
            { //right side
                v3f new_axis = normalize(sub(m.verts[hull_indecies[next_vert]], m.verts[hull_indecies[last_left_vert]]));
                ccw_direction = rejection(new_axis, current_axis);
                current_axis = new_axis;
                
                last_right_vert = next_vert;
            }
        }
        temp_free_memory = (void *)((v3f *) temp_free_memory + n_connection_points);

        //delete points that cannot contribute to the convex hull
        uint n_useless_verts = inmost_right_vert-inmost_left_vert;
        for(int i = inmost_left_vert; i < n_hull_verts-n_useless_verts; i++)
        { //TODO: this is simdizable
            hull_indecies[i] = hull_indecies[i+n_useless_verts];
        }
        outmost_right_vert -= n_useless_verts;
        right_edge -= n_useless_verts;
        n_hull_verts -= n_useless_verts;
        
        uint * keep_list = (uint *) temp_free_memory;
        uint n_to_keep = 0;
        for(uint i = outmost_left_vert; i < outmost_right_vert; i++)
        {
            //GJK to test if each point is contained in the tunnel
            v3f simplex[4];
            simplex[0] = sub(connection_points[0], m.verts[hull_indecies[i]]);
            int n_simplex_points = 1;
            v3f dir = negative(simplex[0]);
            
            //the 2 point case can only happen once
            support_return second_support = convexHullSupport(connection_points, n_connection_points, m.verts[hull_indecies[i]], dir);
            if(second_support.distance < 0) continue;
            simplex[n_simplex_points++] = second_support.point;
            dir = rejection(negative(simplex[0]), sub(simplex[0], simplex[1]));
            
            for ever
            {
                support_return new_support = convexHullSupport(connection_points, n_connection_points, m.verts[hull_indecies[i]], dir);
                if(new_support.distance < 0) break;
                simplex[n_simplex_points++] = new_support.point;
                
                if(doSimplex(simplex, n_simplex_points, dir))
                { //intersection found
                    goto dont_keep;
                }
            }
            keep_list[n_to_keep++] = i;
            dont_keep:;
        }

        uint current_write_location = inmost_left_vert+1;
        for(int i = 0; i < n_to_keep; i++)
        {
            hull_indecies[current_write_location++] = keep_list[i];
        }
        for(int i = outmost_right_vert; i < right_edge; i++)
        { //TODO: simdize
            hull_indecies[i-(outmost_right_vert-outmost_left_vert)+n_to_keep] = hull_indecies[i];
        }

        n_hull_verts -= outmost_right_vert-outmost_left_vert-n_to_keep;
        right_edge -= outmost_right_vert-outmost_left_vert-n_to_keep;
        
        uint middle_edge = (right_edge+left_edge+1)/2;
        ranges[n_ranges].left_edge = left_edge;
        ranges[n_ranges].right_edge = middle_edge;
        n_ranges++;
        ranges[n_ranges].left_edge = left_edge;
        ranges[n_ranges].right_edge = middle_edge;
        n_ranges++;
    }
}

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

//TODO: generate contact points
inline v3f colliding(physics_object a, physics_object b, mesh * mesh_list)
{
    mesh & a_mesh = mesh_list[a.mesh_id];
    mesh & b_mesh = mesh_list[b.mesh_id];
    
    float nearest = -1.0;
    
    for(int a_group = 0; a_group < a_mesh.n_convex_groups; a_group++)
    {
        for(int b_group = 0; b_group < b_mesh.n_convex_groups; b_group++)
        {//TODO: possibly use bounding boxes to speed this up
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
