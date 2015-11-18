#ifndef SIMULATION
#define SIMULATION

#include "meth.h"
#include "misc.h"

#include <string.h>

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
    
    uint * convex_indecies; //sorted starting with the lowest x
    int * convex_starts; //stores the edges of each convex group
    uint * convex_neighbors_starts; //table of the first edge in convex_neighbors for a given vert
    uint * n_convex_neighbors; //number of neighbors for each vert
    uint * convex_neighbors; //index to the neighbors, set of linked lists
    uint n_total_convex_neighbors; //the total number of convex_neighbors for all convex groups
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

#define epsilon 0.005 // the thickness of planes, lines, and points

struct convex_neighbors_link
{
    uint neighbor;
    uint next_link;
};

void addOneWayEdgeToLinkedLists(uint * first_neighbor_links, convex_neighbors_link * neighbors, uint & n_neighbor_links, uint v1, uint v2)
{
    if(first_neighbor_links[v1] == 0)
    {
        first_neighbor_links[v1] = n_neighbor_links;
    }
    else
    {
        uint current_link = first_neighbor_links[v1];
        for(; neighbors[current_link].next_link != 0;
            current_link = neighbors[current_link].next_link)
        {
            //TODO: If the edge exist one way it also exist the other way, this is redundent the second time
            if(neighbors[current_link].neighbor == v2) return;
        }
        neighbors[current_link].next_link = n_neighbor_links;
    }
    neighbors[n_neighbor_links].next_link = 0;
    neighbors[n_neighbor_links].neighbor = v2;
    n_neighbor_links++;
}

struct debug_edge //DEBUG
{
    uint a;
    uint b;
};

debug_edge debug_edges[100]; //DEBUG
uint n_debug_edges = 0;

void addEdgeToLinkedLists(uint * first_neighbor_links, convex_neighbors_link * neighbors, uint & n_neighbor_links, uint v1, uint v2)
{
    if(v1 == v2) return;
    debug_edges[n_debug_edges].a = v1;
    debug_edges[n_debug_edges].b = v2;
    n_debug_edges++;
    
    addOneWayEdgeToLinkedLists(first_neighbor_links,neighbors, n_neighbor_links, v1, v2);
    addOneWayEdgeToLinkedLists(first_neighbor_links,neighbors, n_neighbor_links, v2, v1);
}

struct search_edge
{
    uint v0;
    uint v1;
    v3f dir;
};

//m->convex_indecies and such must be valid pointers
void convexHull(mesh * m, uint start_index, uint end_index, void * free_memory)
{
    uint n_hull_verts = end_index-start_index;
    
    //radix sort TODO: prefetch
    uint histograms[3*(2<<11)];
    memset(histograms, 0, sizeof(histograms));
    
    for(uint i = start_index; i < end_index; i++)
    {
        union
        {
            float value;
            uint32 value_bits;
        };
        
        value = m->coords[i*3];
        value_bits ^= 0x80000000|(-(value_bits>>31));
        for(int h = 0; h < 3; h++) histograms[h*(2<<11) + ((value_bits>>(h*11)) & 0x7FF)]++;
    }
    
    uint offset_tables[3*(2<<11)];
    offset_tables[0] = 0;
    offset_tables[2<<11] = 0;
    offset_tables[2*(2<<11)] = 0;
    
    for(int i = 1; i < 2<<11; i++)
    {
        for(int h = 0; h < 3; h++) offset_tables[i+h*(2<<11)] += histograms[i-1+h*(2<<11)]+offset_tables[i-1+h*(2<<11)];
    }

    if(m->n_convex_groups == 0) m->convex_starts[m->n_convex_groups] = 0;
    
    uint * hull_indecies = m->convex_indecies + m->convex_starts[m->n_convex_groups];
    uint * sorted_indecies = (uint *) ((char *) free_memory);
    free_memory = (void *)((byte *) free_memory + sizeof(uint)*n_hull_verts);
    
    for(uint i = start_index; i < end_index; i++)
    {
        union
        {
            float value;
            uint32 value_bits;
        };
        
        value = m->coords[i*3];
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
        
        value = m->coords[3*hull_indecies[i]];
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
        
        value = m->coords[3*sorted_indecies[i]];
        value_bits ^= 0x80000000|(-(value_bits>>31));

        hull_indecies[offset_tables[((value_bits>>22) & 0x7FF)+2*(2<<11)]++] = sorted_indecies[i];
    }
    //sorted_indecies is sorted by increasing x
    //TODO: might not need to sort by x anymore
    
    for(int i = 0; i < n_hull_verts; i++)
        printf("%d, (%f, %f, %f)\n",
               i,
               m->verts[hull_indecies[i]].x,
               m->verts[hull_indecies[i]].y,
               m->verts[hull_indecies[i]].z);
    printf("\n");
    
    uint8 * verts_to_keep = (uint8 *) free_memory;
    memset(verts_to_keep, 0, (n_hull_verts+7)/8);
    free_memory = (void *)((uint8 *) free_memory + (n_hull_verts+7)/8);
    
    //a must be less than b
    #define verts_to_sedge_sorted(a, b) verts_to_sedge_map[((b)*(b-1))/2+(a)]
    //no order requirement
    #define verts_to_sedge(a, b) (((a) < (b)) ? verts_to_sedge_sorted(a, b) : verts_to_sedge_sorted(b, a))
    #define no_sedge 0xFFFFFFFF
    #define used_sedge 0xFFFFFFFE
    
    //NOTE: this can be a hash table to save memory
    uint max_edges = ((n_hull_verts)*(n_hull_verts-1))/2;
    printf("max edges: %d\n", max_edges);
    
    uint * verts_to_sedge_map = (uint *) free_memory;
    memset(verts_to_sedge_map, no_sedge, sizeof(verts_to_sedge_map[0])*max_edges);
    free_memory = (void *)((uint *) free_memory + max_edges);
    
    search_edge * sedges = (search_edge *) free_memory;
    free_memory = (void *)((search_edge *) free_memory + max_edges);
    uint first_vert = 0;
    for(int i = 0; m->verts[hull_indecies[i]].x == m->verts[hull_indecies[0]].x; i++)
    {
        if(m->verts[hull_indecies[i]].y <= m->verts[hull_indecies[first_vert]].y)
        {
            first_vert = i;
        }
    }
    uint second_vert = 1;
    #define slope(a, b) ((m->verts[hull_indecies[b]].y-m->verts[hull_indecies[a]].y)/(m->verts[hull_indecies[b]].x-m->verts[hull_indecies[a]].x))
    float highest_slope = slope(first_vert, second_vert);
    for(int i = 0; i < n_hull_verts; i++)
    {
        if(i == first_vert) continue;
        float new_slope = slope(first_vert, i);
        if(new_slope > highest_slope)
        {
            second_vert = i;
            highest_slope = new_slope;
        }
    }
    #undef slope
    
    sedges[0] = (search_edge){first_vert, second_vert, cross((v3f){1.0, 0.0, 0.0},
                                                             sub(m->verts[hull_indecies[second_vert]],
                                                                 m->verts[hull_indecies[first_vert]]))};
    int n_sedges = 1;
    
    verts_to_sedge(first_vert, second_vert) = 0;

    #define keep_vert(v) verts_to_keep[(v)/8] |= 1<<((v)%8)
    keep_vert(first_vert);
    keep_vert(second_vert);
    
    //two way mapping, neighbor id's always refer to the same final vert
    uint n_original_verts = n_hull_verts;
    uint * neighbor_to_hull_vert_id = (uint *) free_memory; //values are decremented when hull verts are deleted
    free_memory = (void *) ((uint *) free_memory + n_original_verts);
    uint * hull_vert_to_neighbor_id = (uint *) free_memory; //elements are shifted when hull verts are deleted
    free_memory = (void *) ((uint *) free_memory + n_hull_verts);
    for(int i = 0; i < n_hull_verts; i++)
    {//initialize arrays
        neighbor_to_hull_vert_id[i] = i;
        hull_vert_to_neighbor_id[i] = i;
    }
    
    uint * first_neighbor_links = (uint *) free_memory; //table of first links for each vert
    //NOTE: this^ can be replaced with a hash table to save memory
    memset(free_memory, 0, sizeof(first_neighbor_links[0])*n_hull_verts);
    free_memory = (void *) ((uint *) free_memory + n_hull_verts);
    
    convex_neighbors_link * neighbors = (convex_neighbors_link *) free_memory-1;
    uint n_neighbor_links = 1;
    
    addEdgeToLinkedLists(first_neighbor_links, neighbors, n_neighbor_links, first_vert, second_vert);
    
    while(n_sedges-- > 0)
    {
        uint v0 = sedges[n_sedges].v0;
        uint v1 = sedges[n_sedges].v1;
        //printf("%d, %d\n", v0, v1);
        v3f dir = sedges[n_sedges].dir;
        //printf("dir: (%f, %f, %f)\n", dir.x, dir.y, dir.z);
        
        verts_to_sedge(v0, v1) = used_sedge;
        
        uint next_vert = 0;
        while(next_vert == v0 || next_vert == v1) next_vert++;
        float highest_dot = dot(normalize(rejection(sub(m->verts[hull_indecies[next_vert]], m->verts[hull_indecies[v0]]),
                                                    sub(m->verts[hull_indecies[v1]], m->verts[hull_indecies[v0]]))), dir);
        for(int i = 0; i < n_hull_verts; i++)
        {
            if(i == v0 || i == v1) continue;
            float new_dot = dot(normalize(rejection(sub(m->verts[hull_indecies[i]], m->verts[hull_indecies[v0]]),
                                                    sub(m->verts[hull_indecies[v1]], m->verts[hull_indecies[v0]]))), dir);
            if(new_dot > highest_dot)
            {
                next_vert = i;
                highest_dot = new_dot;
            }
        }
        //printf("next: %d\n", next_vert);
        
        if(verts_to_sedge(v0, next_vert) == used_sedge){}
        else if(verts_to_sedge(v0, next_vert) == no_sedge)
        {
            //printf("adding %d, %d edge\n", v0, next_vert);
            verts_to_sedge(v0, next_vert) = n_sedges;
            sedges[n_sedges++] = (search_edge){v0, next_vert,
                                               rejection(sub(m->verts[hull_indecies[next_vert]], m->verts[hull_indecies[v1]]),
                                                         sub(m->verts[hull_indecies[next_vert]], m->verts[hull_indecies[v0]]))};
            addEdgeToLinkedLists(first_neighbor_links, neighbors, n_neighbor_links, v0, next_vert);
            keep_vert(next_vert);
        }
        else
        {
            //printf("closing %d, %d edge\n", v0, next_vert);
            sedges[verts_to_sedge(v0, next_vert)] = sedges[--n_sedges];
            /* n_sedges--; */
            /* for(int i = verts_to_sedge(v0, next_vert); i < n_sedges; i++) sedges[i] = sedges[i+1]; */
            verts_to_sedge(v0, next_vert) = used_sedge;
        }
        
        if(verts_to_sedge(v1, next_vert) == used_sedge){}
        else if(verts_to_sedge(v1, next_vert) == no_sedge)
        {
            //printf("adding %d, %d edge\n", v1, next_vert);
            verts_to_sedge(v1, next_vert) = n_sedges;
            sedges[n_sedges++] = (search_edge){v1, next_vert,
                                               rejection(sub(m->verts[hull_indecies[next_vert]], m->verts[hull_indecies[v0]]),
                                                         sub(m->verts[hull_indecies[next_vert]], m->verts[hull_indecies[v1]]))};
            //printf("(%f, %f, %f)\n", sedges[n_sedges-1].dir.x, sedges[n_sedges-1].dir.y, sedges[n_sedges-1].dir.z);
            addEdgeToLinkedLists(first_neighbor_links, neighbors, n_neighbor_links, v1, next_vert);
            keep_vert(next_vert);
        }
        else
        {
            //printf("closing %d, %d edge\n", v1, next_vert);
            sedges[verts_to_sedge(v1, next_vert)] = sedges[--n_sedges];
            /* n_sedges--; */
            /* for(int i = verts_to_sedge(v1, next_vert); i < n_sedges; i++) sedges[i] = sedges[i+1]; */
            verts_to_sedge(v1, next_vert) = used_sedge;
        }
        
        /* //printf("n_sedges: %d\n", n_sedges); */
        /* for(int i = 0; i < max_edges; i++) */
        /* { */
        /*     //printf("verts_to_sedge %d: %d\n", i, verts_to_sedge_map[i]); */
        /* } */
    }
    
    uint n_verts_to_keep = 0;
    for(int i = 0; i < n_hull_verts; i++)
    {
        if((verts_to_keep[i/8]>>i%8)&1)
        {
            neighbor_to_hull_vert_id[i] -= i-n_verts_to_keep;
            hull_indecies[n_verts_to_keep] = hull_indecies[i];
            hull_vert_to_neighbor_id[n_verts_to_keep] = hull_vert_to_neighbor_id[i];
            n_verts_to_keep++;
        }
    }
    n_hull_verts = n_verts_to_keep;
    
    for(int i = 0; i < n_hull_verts; i++)
    {
        if(first_neighbor_links[hull_vert_to_neighbor_id[i]] != 0)
        {
            uint current_link = first_neighbor_links[hull_vert_to_neighbor_id[i]];
            
            uint current_convex_index = m->convex_starts[m->n_convex_groups]+i;
            m->n_convex_neighbors[current_convex_index] = 0;
            m->convex_neighbors_starts[current_convex_index] = m->n_total_convex_neighbors;
            for(; current_link != 0; current_link = neighbors[current_link].next_link)
            {
                m->n_convex_neighbors[current_convex_index]++;
                m->convex_neighbors[m->n_total_convex_neighbors++] =
                    neighbor_to_hull_vert_id[neighbors[current_link].neighbor];
            }
        }
        else
        {
            uint current_convex_index = m->convex_starts[m->n_convex_groups]+i;
            m->n_convex_neighbors[current_convex_index] = 0;
            printf("error, vertex %d with no edges added to convex hull\n", i);
        }
    }
    m->convex_starts[m->n_convex_groups+1] = m->convex_starts[m->n_convex_groups]+n_hull_verts;
    m->n_convex_groups++;
    
    for(int i = 0; i < n_hull_verts; i++)
    {
        printf("%d, (%f, %f, %f)\n", hull_indecies[i], m->verts[hull_indecies[i]].x, m->verts[hull_indecies[i]].y, m->verts[hull_indecies[i]].z);
    }
    
    printf("debug_edges:\n");
    for(int i = 0; i < n_debug_edges; i++) printf("%d, %d\n", neighbor_to_hull_vert_id[debug_edges[i].a], neighbor_to_hull_vert_id[debug_edges[i].b]);
}

struct support_return
{
    v3f point;
    float distance;
};

//TODO: (maybe) epsiloning this would make it more robust, but it would also ~double the number of ifs
bool8 doSimplex(v3f * simplex, int & n_simplex_points, v3f & dir)
{
    assert(n_simplex_points > 2);
    
    if(n_simplex_points == 3)
    {
        v3f axis = cross(sub(simplex[0], simplex[2]), sub(simplex[1], simplex[2]));
        v3f norm0 = cross(axis, sub(simplex[1], simplex[2]));
        v3f norm1 = cross(sub(simplex[0], simplex[2]), axis);
        v3f to_origin = negative(simplex[2]);
        
        /* if(dot(norm0, dir) < 0) norm0 = negative(norm0); */
        /* if(dot(norm1, dir) < 0) norm1 = negative(norm1); */
        
        if(dot(norm0, to_origin) < 0)
        {
            if(dot(norm1, to_origin) < 0)
            { //middle
                if(dot(axis, to_origin) > 0)
                { //top
                    dir = axis;
                }
                else
                { //bottom
                    dir = negative(axis);
                }
            }
            else
            { //0-2 side
                dir = rejection(to_origin, sub(simplex[0], simplex[2]));
                simplex[1] = simplex[--n_simplex_points];
            }
        }
        else
        {
            if(dot(norm1, to_origin) < 0)
            { //1-2 side
                dir = rejection(to_origin, sub(simplex[1], simplex[2]));
                simplex[0] = simplex[--n_simplex_points];
            }
            else
            {
                return true;
                assert(0 && "You broke math");
            }
        }
    }
    else
    { //TODO
        v3f norm0 = cross(sub(simplex[0], simplex[3]), sub(simplex[1], simplex[3]));
        v3f norm1 = cross(sub(simplex[1], simplex[3]), sub(simplex[2], simplex[3]));
        v3f norm2 = cross(sub(simplex[2], simplex[3]), sub(simplex[0], simplex[3]));
        v3f to_origin = negative(simplex[3]);
        
        if(dot(norm0, sub(simplex[3], simplex[2])) < 0) norm0 = negative(norm0);
        if(dot(norm1, sub(simplex[3], simplex[0])) < 0) norm1 = negative(norm1);
        if(dot(norm2, sub(simplex[3], simplex[1])) < 0) norm2 = negative(norm2);
        
        if(dot(norm0, to_origin) < 0)
        {
            if(dot(norm1, to_origin) < 0)
            {
                if(dot(norm2, to_origin) < 0)
                { //inside
                    return true;
                }
                else
                { //face 2
                    dir = norm2;
                    simplex[1] = simplex[--n_simplex_points];
                }
            }
            else if(dot(norm2, to_origin) < 0)
            { //face 1
                dir = norm1;
                simplex[0] = simplex[--n_simplex_points];
            }
            else
            { //1-2 edge
                dir = rejection(to_origin, sub(simplex[2], simplex[3]));
                simplex[0] = simplex[2];
                simplex[1] = simplex[3];
                n_simplex_points -= 2;
            }
        }
        else if(dot(norm1, to_origin) < 0)
        {
            if(dot(norm2, to_origin) < 0)
            { //face 0
                dir = norm0;
                simplex[2] = simplex[--n_simplex_points];
            }
            else
            { //0-2 edge
                dir = rejection(to_origin, sub(simplex[0], simplex[3]));
                simplex[0] = simplex[0];
                simplex[1] = simplex[3];
                n_simplex_points -= 2;
            }
        }
        else if(dot(norm2, to_origin) < 0)
        { //0-1 edge
            dir = rejection(to_origin, sub(simplex[1], simplex[3]));
            simplex[0] = simplex[1];
            simplex[1] = simplex[3];
            n_simplex_points -= 2;
        }
        else
        {
            return true;
            printf("\n");
            printf("(%f, %f, %f)\n", simplex[0].x, simplex[0].y, simplex[0].z);
            printf("(%f, %f, %f)\n", simplex[1].x, simplex[1].y, simplex[1].z);
            printf("(%f, %f, %f)\n", simplex[2].x, simplex[2].y, simplex[2].z);
            printf("(%f, %f, %f)\n", simplex[3].x, simplex[3].y, simplex[3].z);
            assert(0 && "origin is past the new simplex point");
        }
    }
    
    return false;
}

//TODO: use neighbor data to speed up search
//GJK support function for one convex object
inline support_return support(physics_object a, int a_group, mesh * mesh_list, v3f dir)
{
    v3f original_dir = dir;//DEBUG
    float displacement = dot(a.position, dir);
    dir = applyQuaternion(inverseQuaternion(a.orientation), dir);
    mesh & a_mesh = mesh_list[a.mesh_id];
    
    support_return highest = {
        a_mesh.verts[a_mesh.convex_indecies[a_group]],
        dot(a_mesh.verts[a_mesh.convex_indecies[a_group]], dir)
    };

    //NOTE: don't need to test the first one since it is the initial value
    for(uint v = a_mesh.convex_starts[a_group]+1; v < a_mesh.convex_starts[a_group+1]; v++)
    {
        float proj = dot(a_mesh.verts[a_mesh.convex_indecies[v]], dir);
        if(proj > highest.distance)
        {
            highest.distance = proj;
            highest.point = a_mesh.verts[a_mesh.convex_indecies[v]];
        }
    }
    highest.point = applyQuaternion(a.orientation, highest.point);
    
    highest.point = add(highest.point, a.position);
    highest.distance += displacement;
    
    return highest;
}

//GJK support function for the Minkowski differnce of two convex polygonal objects
inline support_return support(physics_object a, int a_group, physics_object b, int b_group, mesh * mesh_list, v3f dir)
{
    support_return a_support = support(a, a_group, mesh_list, dir);
    support_return b_support = support(b, b_group, mesh_list, negative(dir));
    
    support_return out = {
        sub(a_support.point, b_support.point),
        a_support.distance + b_support.distance
    };
    return out;
}

//TODO: generate contact points
inline bool isColliding(physics_object a, physics_object b, mesh * mesh_list)
{
    mesh & a_mesh = mesh_list[a.mesh_id];
    mesh & b_mesh = mesh_list[b.mesh_id];
    
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
            
            #define max_itterations 30
            for(int i = 0;; i++)
            {
                dir = scale(normalize(dir), 1000);//TODO: this is an awful hack, do a real fix for precission issues
                support_return new_support = support(a, a_group, b, b_group, mesh_list, dir);
                if(new_support.distance < 0) break;
                simplex[n_simplex_points++] = new_support.point;
                
                if(doSimplex(simplex, n_simplex_points, dir)) return true; //intersection found
                if(i >= max_itterations) return true; //max itterations reached
            }
            #undef max_itterations
            //these two convex hulls do not intersect
        }
    }
    
    return false;
}

/* //find the point(s if multiple points are equidistant) on each hull that is nearest to the other hull */
/* //or mabye find the two closest tetrahedrons and intersect those */
/* inline v3f findContactPoint(physics_object a, physics_object b, mesh * mesh_list) */
/* { //TODO: use previous knowledge to pick start parameters */
/*     mesh & a_mesh = mesh_list[a.mesh_id]; */
/*     mesh & b_mesh = mesh_list[b.mesh_id]; */
    
/*     float nearest = -1.0; */
    
/*     for(int a_group = 0; a_group < a_mesh.n_convex_groups; a_group++) */
/*     { */
/*         for(int b_group = 0; b_group < b_mesh.n_convex_groups; b_group++) */
/*         { //TODO: possibly use bounding boxes to speed this up */
/*             uint current_point = 0; */
            
/*             for ever */
/*             { */
                
/*             } */
/*         } */
/*     } */
/* } */

//TODO: swept objects

#endif
