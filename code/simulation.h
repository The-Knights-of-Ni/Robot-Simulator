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
        v3f norm0 = cross(axis, sub(simplex[1], simplex[2]));
        v3f norm1 = cross(sub(simplex[0], simplex[2]), axis);
        v3f to_origin = negative(simplex[2]);
        
        if(dot(norm0, dir) < 0) norm0 = negative(norm0);
        if(dot(norm1, dir) < 0) norm1 = negative(norm1);
        
        if(dot(norm0, to_origin) < 0)
        {
            if(dot(norm1, to_origin) < 0)
            { //middle
                if(dot(axis, to_origin))
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
        
        if(dot(norm0, dir) < 0) norm0 = negative(norm0);
        if(dot(norm1, dir) < 0) norm1 = negative(norm1);
        if(dot(norm2, dir) < 0) norm2 = negative(norm2);
        
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
            assert(0 && "origin is past the new simplex point");
        }
    }
    
    return false;
}

struct convex_hull_range
{
    uint left_edge;
    uint right_edge;
};

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
            current_link = neighbors[current_link].next_link);
        neighbors[current_link].next_link = n_neighbor_links;
    }
    neighbors[n_neighbor_links].next_link = 0;
    neighbors[n_neighbor_links].neighbor = v2;
    n_neighbor_links++;
}

void addEdgeToLinkedLists(uint * first_neighbor_links, convex_neighbors_link * neighbors, uint & n_neighbor_links, uint v1, uint v2)
{
    addOneWayEdgeToLinkedLists(first_neighbor_links,neighbors, n_neighbor_links, v1, v2);
    addOneWayEdgeToLinkedLists(first_neighbor_links,neighbors, n_neighbor_links, v2, v1);
}

//m->convex_indecies and such must be valid pointers
void convexHull(mesh * m, uint start_index, uint end_index, void * free_memory)
{
    /* vert_edges * graph = free_memory; */
    /* free_memory = (void *) (byte *) free_memory + sizeof(vert_edges)*m->n_verts*/
    
    /* for(int i = 0; i < m,n_indecies; i+=3) */
    /* { */
    /*     graph[m->indecies[i+0]].n_edges = 0; */
    /*     addEdge(graph, m->indecies[i+0], m->indecies[i+1]); */
    /*     addEdge(graph, m->indecies[i+0], m->indecies[i+2]); */
                                                                                      
    /*     graph[m->indecies[i+1]].n_edges = 0;               */
    /*     addEdge(graph, m->indecies[i+1], m->indecies[i+0]); */
    /*     addEdge(graph, m->indecies[i+1], m->indecies[i+2]); */
                                                                                      
    /*     graph[m->indecies[i+2]].n_edges = 0;               */
    /*     addEdge(graph, m->indecies[i+2], m->indecies[i+0]); */
    /*     addEdge(graph, m->indecies[i+2], m->indecies[i+1]); */
    /* } */

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
    free_memory = (void *)((byte *) free_memory + sizeof(uint)*n_hull_verts);
    uint * sorted_indecies = (uint *) ((char *) free_memory);
    
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
    //hull_indecies is sorted by increasing x
    
    for(int i = 0; i < n_hull_verts; i++)
        printf("%d, (%f, %f, %f)\n",
               i,
               m->verts[hull_indecies[i]].x,
               m->verts[hull_indecies[i]].y,
               m->verts[hull_indecies[i]].z);
    
    convex_hull_range * ranges = (convex_hull_range *) free_memory;
    free_memory = (void *) (ranges + 2*(clog_2(n_hull_verts))); //at most one node of each unexplored branch will be stored
    ranges[0].left_edge = 0;
    ranges[0].right_edge = n_hull_verts;
    uint n_ranges = 1;
    
    uint * first_neighbor_links = (uint *) free_memory; //table of first links for each vert
    //NOTE: this^ can be replaced with a hash table to save memory
    memset(free_memory, 0, sizeof(first_neighbor_links[0])*n_hull_verts);
    free_memory = (void *) ((uint *) free_memory + n_hull_verts);
    
    convex_neighbors_link * neighbors = (convex_neighbors_link *) free_memory-1;
    uint n_neighbor_links = 1;
    
    while(n_ranges--)
    { //connect the two halves and remove useless points
        uint max_new_edges = ranges[n_ranges].right_edge - ranges[n_ranges].left_edge;
        max_new_edges *= (max_new_edges-1);
        max_new_edges >>= 1; //max_edges = n choose 2
        void * temp_free_memory = (void *)((convex_neighbors_link *) free_memory + n_neighbor_links + max_new_edges);
        
        uint left_edge = ranges[n_ranges].left_edge;
        uint right_edge = ranges[n_ranges].right_edge;
        
        uint middle_vert = (left_edge+right_edge)/2; //TODO: loop over the sub-endpoints from a stack
    
        uint left_bridge_vert = middle_vert-1;
        uint right_bridge_vert = middle_vert;
        
        {//initialize bridge verts to the highest verts on each side
            int i = left_edge;
            for(; i < middle_vert; i++)
            {
                if(m->coords[1+3*hull_indecies[i]] >= m->coords[1+3*hull_indecies[left_bridge_vert]]) left_bridge_vert = i;
            }
            for(; i < right_edge; i++)
            {
                if(m->coords[1+3*hull_indecies[i]] >= m->coords[1+3*hull_indecies[right_bridge_vert]]) right_bridge_vert = i;
            }
        }
                
        for ever
        {//check if there are better candidates for the bridge start
            float test_normal_x = m->coords[1+3*hull_indecies[left_bridge_vert]] - m->coords[1+3*hull_indecies[right_bridge_vert]];
            float test_normal_y = m->coords[3*hull_indecies[right_bridge_vert]] - m->coords[3*hull_indecies[left_bridge_vert]];
        
            float current_left_dot = test_normal_x*m->coords[3*hull_indecies[left_bridge_vert]]
                + test_normal_y*m->coords[1+3*hull_indecies[left_bridge_vert]];
            float current_right_dot = test_normal_x*m->coords[3*hull_indecies[right_bridge_vert]]
                + test_normal_y*m->coords[1+3*hull_indecies[right_bridge_vert]];
            
            int i = left_edge;
            for(; i < middle_vert; i++)
            {
                if((test_normal_x*m->coords[3*hull_indecies[i]]
                    + test_normal_y*m->coords[1+3*hull_indecies[i]])
                   > current_left_dot)
                {
                    left_bridge_vert = i;
                    
                    test_normal_x = m->coords[1+3*hull_indecies[left_bridge_vert]] - m->coords[1+3*hull_indecies[right_bridge_vert]];
                    test_normal_y = m->coords[3*hull_indecies[right_bridge_vert]] - m->coords[3*hull_indecies[left_bridge_vert]];
                    current_left_dot = test_normal_x*m->coords[3*hull_indecies[left_bridge_vert]]
                        + test_normal_y*m->coords[1+3*hull_indecies[left_bridge_vert]];
                    current_right_dot = test_normal_x*m->coords[3*hull_indecies[right_bridge_vert]]
                        + test_normal_y*m->coords[1+3*hull_indecies[right_bridge_vert]];
                    //no need to restart the search since any of the
                    //points that failed the test for the old left bridge
                    //point will also fail the new test
                }
            }
            for(;; i++)
            {
                if(i >= right_edge) goto stop_searching_for_bridge;
                if((test_normal_x*m->coords[3*hull_indecies[i]]
                    + test_normal_y*m->coords[1+3*hull_indecies[i]])
                   > current_right_dot)
                {
                    right_bridge_vert = i;
                    break;
                    //need to restart the search here since all the left
                    //verts need to be rechecked
                }
            }
        }
    stop_searching_for_bridge:;
        
        printf("%d, %d\n", left_bridge_vert, right_bridge_vert);
        
        addEdgeToLinkedLists(first_neighbor_links, neighbors, n_neighbor_links, left_bridge_vert, right_bridge_vert);
        
        //gift wrap connection
        v3f current_axis = normalize(sub(m->verts[hull_indecies[right_bridge_vert]], m->verts[hull_indecies[left_bridge_vert]]));
        //this^ normalize has an error of .002 with the test case
        v3f ccw_direction = {current_axis.z, 0.0, -current_axis.x};//cross(current_axis, v3f(0, -1, 0));
        uint last_left_vert = left_bridge_vert;
        uint last_right_vert = right_bridge_vert;
        uint next_vert = 0;
        
        uint inmost_left_vert = left_bridge_vert;
        uint inmost_right_vert = right_bridge_vert;
        uint outmost_left_vert = left_bridge_vert;
        uint outmost_right_vert = right_bridge_vert;
        
        v3f * connection_points = (v3f *) temp_free_memory;
        uint n_connection_points = 0;
        
        printf("%f, %f, %f\n", current_axis.x, current_axis.y, current_axis.z);
        printf("%f, %f, %f\n", ccw_direction.x, ccw_direction.y, ccw_direction.z);
        
        for ever
        {
            float highest_dot = -1.1;
            for(int i = 0; i < n_hull_verts; i++)
            {
                if(i == last_left_vert || i == last_right_vert) continue;
                float current_dot = dot(ccw_direction,
                                        normalize(sub(m->verts[hull_indecies[i]], m->verts[hull_indecies[last_right_vert]])));
                
                if(current_dot > highest_dot)
                {
                    highest_dot = current_dot;
                    next_vert = i;
                }
            }
            printf("next %d\n", next_vert);
            
            if((next_vert == left_bridge_vert && last_right_vert == right_bridge_vert)
               || (next_vert == right_bridge_vert && last_left_vert == left_bridge_vert))
            {
                printf("exiting loop\n");
                break;
            }
            
            //TODO: make sure points are only added once
            connection_points[n_connection_points++] = m->verts[hull_indecies[next_vert]];
            
            //add edges
            addEdgeToLinkedLists(first_neighbor_links,neighbors, n_neighbor_links, next_vert, left_bridge_vert);
            addEdgeToLinkedLists(first_neighbor_links,neighbors, n_neighbor_links, next_vert, right_bridge_vert);
            
            if(next_vert < middle_vert)
            { //left side
                if(m->coords[3*hull_indecies[next_vert]] > m->coords[3*hull_indecies[  inmost_left_vert]])
                    inmost_left_vert   = next_vert;
                if(m->coords[3*hull_indecies[next_vert]] < m->coords[3*hull_indecies[ outmost_left_vert]])
                    outmost_left_vert  = next_vert;
                
                printf("lefty %d\n", next_vert);
                v3f new_axis = normalize(sub(m->verts[hull_indecies[next_vert]], m->verts[hull_indecies[last_right_vert]]));
                ccw_direction = normalize(rejection(current_axis, new_axis));
                current_axis = new_axis;
                
                last_left_vert = next_vert;
            }
            else
            { //right side
                if(m->coords[3*hull_indecies[next_vert]] < m->coords[3*hull_indecies[ inmost_right_vert]])
                    inmost_right_vert  = next_vert;
                if(m->coords[3*hull_indecies[next_vert]] > m->coords[3*hull_indecies[outmost_right_vert]])
                    outmost_right_vert = next_vert;
                
                printf("righty %d\n", next_vert);
                v3f new_axis = normalize(sub(m->verts[hull_indecies[next_vert]], m->verts[hull_indecies[last_left_vert]]));
                ccw_direction = normalize(rejection(current_axis, new_axis));
                current_axis = new_axis;
                
                last_right_vert = next_vert;
            }
        }
        temp_free_memory = (void *)((v3f *) temp_free_memory + n_connection_points);

        printf("here\n");
        //delete points that cannot contribute to the convex hull
        printf("%d, %d\n", inmost_left_vert, inmost_right_vert);
        int left_kill = inmost_left_vert;
        while(m->coords[3*hull_indecies[++left_kill]] == m->coords[3*hull_indecies[inmost_left_vert]] && left_kill < middle_vert);
        
        int right_kill = inmost_right_vert;
        while(m->coords[3*hull_indecies[--right_kill]] == m->coords[3*hull_indecies[inmost_right_vert]] && right_kill >= middle_vert);
        
        uint n_useless_verts = right_kill-left_kill;
        printf("there\n");
        printf("%d, %d, %d\n", left_kill, right_kill, n_useless_verts);

        if(n_useless_verts > 0)
        {
            for(int i = left_kill; i < n_hull_verts-n_useless_verts; i++)
            { //TODO: this is simdizable
                hull_indecies[i] = hull_indecies[i+n_useless_verts];
            }
        }
        outmost_right_vert -= n_useless_verts;
        right_edge -= n_useless_verts;
        n_hull_verts -= n_useless_verts;
        
        uint * keep_list = (uint *) temp_free_memory;
        uint n_to_keep = 0;
        for(uint i = outmost_left_vert+1; i < outmost_right_vert; i++)
        { //GJK to test if each point is contained in the tunnel
            v3f simplex[4];
            simplex[0] = sub(connection_points[0], m->verts[hull_indecies[i]]);
            int n_simplex_points = 1;
            v3f dir = negative(simplex[0]);
            
            support_return second_support;
            
            for(int j = 0; j < n_connection_points; j++)
            {
                if(connection_points[j].x == m->verts[hull_indecies[i]].x &&
                   connection_points[j].y == m->verts[hull_indecies[i]].y &&
                   connection_points[j].z == m->verts[hull_indecies[i]].z) goto keep;
            }
            printf("%f, %f, %f\n", m->verts[hull_indecies[i]].x, m->verts[hull_indecies[i]].y, m->verts[hull_indecies[i]].z);
            
            //the 2 point case can only happen once
            second_support = convexHullSupport(connection_points, n_connection_points, m->verts[hull_indecies[i]], dir);
            if(second_support.distance < 0) continue;
            simplex[n_simplex_points++] = second_support.point;
            dir = rejection(negative(simplex[0]), sub(simplex[0], simplex[1]));
            
            printf("s0 %f, %f, %f\n", simplex[0].x, simplex[0].y, simplex[0].z);
            printf("s1 %f, %f, %f\n", simplex[1].x, simplex[1].y, simplex[1].z);
            for ever
            {
                support_return new_support = convexHullSupport(connection_points, n_connection_points, m->verts[hull_indecies[i]], dir);
                if(new_support.distance < 0) break;
                simplex[n_simplex_points++] = new_support.point;
                
                printf("ns %f, %f, %f\n", new_support.point.x, new_support.point.y, new_support.point.z);
                
                if(doSimplex(simplex, n_simplex_points, dir))
                { //intersection found
                    printf("get out, point!\n");
                    goto dont_keep;
                }
            }
        keep:;
            printf("keeping %d\n", i);
            keep_list[n_to_keep++] = i;
        dont_keep:;
        }
        
        printf("%d\n", n_to_keep);
        
        uint current_write_location = outmost_left_vert+1;
        for(int i = 0; i < n_to_keep; i++)
        {
            hull_indecies[current_write_location++] = hull_indecies[keep_list[i]];
        }
        for(int i = outmost_right_vert; i < right_edge; i++)
        { //TODO: simdize
            hull_indecies[i-(outmost_right_vert-outmost_left_vert-1)+n_to_keep] = hull_indecies[i];
        }
        
        n_hull_verts -= outmost_right_vert-outmost_left_vert-1-n_to_keep;
        right_edge -= outmost_right_vert-outmost_left_vert-n_to_keep;
        
        uint middle_edge = (right_edge+left_edge+1)/2;
        if(middle_edge-left_edge > 4)
        {
            ranges[n_ranges].left_edge = left_edge;
            ranges[n_ranges].right_edge = middle_edge;
            n_ranges++;
        }
        if(right_edge-middle_edge > 4)
        {
            ranges[n_ranges].left_edge = middle_edge;
            ranges[n_ranges].right_edge = right_edge;
            n_ranges++;
        }
    }
    
    for(int i = 0; i < n_hull_verts; i++)
    {
        if(first_neighbor_links[i] != 0)
        {
            uint current_link = first_neighbor_links[i];
            
            uint current_convex_index = m->convex_starts[m->n_convex_groups]+i;
            m->n_convex_neighbors[current_convex_index] = 0;
            m->convex_neighbors_starts[current_convex_index] = m->n_total_convex_neighbors;
            for(; current_link != 0; current_link = neighbors[current_link].next_link)
            {
                m->n_convex_neighbors[current_convex_index]++;
                m->convex_neighbors[m->n_total_convex_neighbors++] = neighbors[current_link].neighbor;
            }
        }
        else
        {
            printf("error, vertex with no edges added to convex hull\n");
        }
    }
    m->convex_starts[m->n_convex_groups+1] = m->convex_starts[m->n_convex_groups]+n_hull_verts;
    m->n_convex_groups++;
    
    for(int i = 0; i < n_hull_verts; i++)
    {
        printf("%d, (%f, %f, %f)\n", hull_indecies[i], m->verts[hull_indecies[i]].x, m->verts[hull_indecies[i]].y, m->verts[hull_indecies[i]].z);
    }
    
}

//TODO: use neighbor data to speed up search
//GJK support function for one convex object
inline support_return support(physics_object a, int a_group, mesh * mesh_list, v3f dir)
{
    float displacement = dot(a.position, dir);
    dir = applyQuaternion(inverseQuaternion(a.orientation), dir);
    mesh & a_mesh = mesh_list[a.mesh_id];
    
    support_return highest = {
        a_mesh.verts[a_mesh.convex_indecies[a_group]],
        dot(a_mesh.verts[a_mesh.convex_indecies[a_group]], dir)
    };
    
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
#define epsilon 0.005
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
            if(dot(dir, dir) <= epsilon) return true;
            
            //the 2 point case can only happen once
            support_return second_support = support(a, a_group, b, b_group, mesh_list, dir);
            
            if(second_support.distance < 0) continue;
            simplex[n_simplex_points++] = second_support.point;
            dir = rejection(negative(simplex[0]), sub(simplex[0], simplex[1]));
            if(dot(dir, dir) <= epsilon) return true;
            
            #define max_itterations 30
            for(int i = 0;; i++)
            {
                support_return new_support = support(a, a_group, b, b_group, mesh_list, dir);
                if(new_support.distance < 0) break;
                simplex[n_simplex_points++] = new_support.point;
                
                if(doSimplex(simplex, n_simplex_points, dir)) return true; //intersection found
                if(dot(dir, dir) <= epsilon) return true; //TODO: check if this is reasonable
                if(i >= max_itterations) return true; //max itterations reached
            }
            //these two convex hulls do not intersect
        }
    }
    
    return false;
}

#if 0
//NOTE: the objects must not be colliding
//find the point(s if multiple points are equidistant) on each hull that is nearest to the other hull
inline v3f findContactPoint(physics_object a, physics_object b, mesh * mesh_list)
{ //TODO: use previous knowledge to pick start parameters
    mesh & a_mesh = mesh_list[a.mesh_id];
    mesh & b_mesh = mesh_list[b.mesh_id];
    
    float nearest = -1.0;
    
    for(int a_group = 0; a_group < a_mesh.n_convex_groups; a_group++)
    {
        for(int b_group = 0; b_group < b_mesh.n_convex_groups; b_group++)
        { //TODO: possibly use bounding boxes to speed this up
            uint current_point = 0;
            
            for ever
            {
                
            }
        }
    }
}
#endif
//TODO: swept objects
