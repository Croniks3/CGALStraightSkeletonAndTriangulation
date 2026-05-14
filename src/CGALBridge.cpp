#include "CGALBridge.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/create_straight_skeleton_2.h>
#include <CGAL/number_utils.h>

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Constrained_triangulation_face_base_2.h>
#include <CGAL/Triangulation_data_structure_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>

#include <list>
#include <vector>

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point2 = Kernel::Point_2;
using Polygon2 = CGAL::Polygon_2<Kernel>;
using Skeleton2 = CGAL::Straight_skeleton_2<Kernel>;

using Vb = CGAL::Triangulation_vertex_base_with_info_2<int, Kernel>;
using Fb = CGAL::Constrained_triangulation_face_base_2<Kernel>;
using Tds = CGAL::Triangulation_data_structure_2<Vb, Fb>;
using Itag = CGAL::Exact_predicates_tag;
using CDT = CGAL::Constrained_Delaunay_triangulation_2<Kernel, Tds, Itag>;

struct FaceInfo2
{
    int nesting_level = -1;

    bool in_domain() const
    {
        return nesting_level % 2 == 1;
    }
};

using TriVb = CGAL::Triangulation_vertex_base_with_info_2<int, Kernel>;
using TriFbb = CGAL::Triangulation_face_base_with_info_2<FaceInfo2, Kernel>;
using TriFb = CGAL::Constrained_triangulation_face_base_2<Kernel, TriFbb>;
using TriTds = CGAL::Triangulation_data_structure_2<TriVb, TriFb>;
using TriCdt = CGAL::Constrained_Delaunay_triangulation_2<Kernel, TriTds, CGAL::Exact_predicates_tag>;

namespace
{
    void MarkDomains(
        TriCdt& cdt,
        TriCdt::Face_handle start,
        int index,
        std::list<TriCdt::Edge>& border
    )
    {
        if (start->info().nesting_level != -1)
        {
            return;
        }

        std::list<TriCdt::Face_handle> queue;
        queue.push_back(start);

        while (!queue.empty())
        {
            TriCdt::Face_handle face = queue.front();
            queue.pop_front();

            if (face->info().nesting_level != -1)
            {
                continue;
            }

            face->info().nesting_level = index;

            for (int i = 0; i < 3; ++i)
            {
                TriCdt::Edge edge(face, i);
                TriCdt::Face_handle neighbor = face->neighbor(i);

                if (neighbor->info().nesting_level != -1)
                {
                    continue;
                }

                if (cdt.is_constrained(edge))
                {
                    border.push_back(edge);
                }
                else
                {
                    queue.push_back(neighbor);
                }
            }
        }
    }

    void MarkDomains(TriCdt& cdt)
    {
        for (auto face = cdt.all_faces_begin(); face != cdt.all_faces_end(); ++face)
        {
            face->info().nesting_level = -1;
        }

        std::list<TriCdt::Edge> border;

        MarkDomains(cdt, cdt.infinite_face(), 0, border);

        while (!border.empty())
        {
            TriCdt::Edge edge = border.front();
            border.pop_front();

            TriCdt::Face_handle neighbor = edge.first->neighbor(edge.second);

            if (neighbor->info().nesting_level == -1)
            {
                MarkDomains(
                    cdt,
                    neighbor,
                    edge.first->info().nesting_level + 1,
                    border);
            }
        }
    }
}

namespace CGALBridge
{
    extern "C"
    int CGALBridge_BuildStraightSkeleton(
        const Vec2* points,
        int count,
        SkeletonVertex* outVertices,
        int maxVertices
    )
    {
        if (count < 3)
            return 0;

        Polygon2 poly;

        for (int i = 0; i < count; ++i)
        {
            poly.push_back(Point2(points[i].x, points[i].y));
        }

        if (!poly.is_simple())
            return 0;

        if (poly.is_clockwise_oriented())
            poly.reverse_orientation();

        auto skeleton = CGAL::create_interior_straight_skeleton_2(
            poly.vertices_begin(),
            poly.vertices_end());

        if (!skeleton)
            return 0;

        int outCount = 0;

        for (auto v = skeleton->vertices_begin(); v != skeleton->vertices_end(); ++v)
        {
            if (outCount >= maxVertices)
                break;

            outVertices[outCount].x = CGAL::to_double(v->point().x());
            outVertices[outCount].y = CGAL::to_double(v->point().y());
            outVertices[outCount].time = CGAL::to_double(v->time());

            outCount++;
        }

        return outCount;
    }
    
    extern "C"
    int CGALBridge_BuildStraightSkeletonGraph(
        const Vec2* points,
        int count,
        
        SkeletonVertex* outVertices,
        int maxVertices,
        int* outVertexCount,
        
        SkeletonEdge* outEdges,
        int maxEdges,
        int* outEdgeCount
    )
    {
        if (!points || !outVertices || !outVertexCount || !outEdges || !outEdgeCount)
            return 0;
        
        *outVertexCount = 0;
        *outEdgeCount = 0;
        
        if (count < 3)
            return 0;

        Polygon2 poly;

        for (int i = 0; i < count; ++i)
        {
            poly.push_back(Point2(points[i].x, points[i].y));
        }

        if (!poly.is_simple())
            return 0;

        if (poly.is_clockwise_oriented())
            poly.reverse_orientation();

        auto skeleton = CGAL::create_interior_straight_skeleton_2(
            poly.vertices_begin(),
            poly.vertices_end());

        if (!skeleton)
            return 0;

        const int neededVertices = static_cast<int>(skeleton->size_of_vertices());

        if (neededVertices > maxVertices)
            return 0;

        std::unordered_map<int, int> vertexIdToIndex;

        int vertexIndex = 0;

        for (auto v = skeleton->vertices_begin(); v != skeleton->vertices_end(); ++v)
        {
            outVertices[vertexIndex].x = CGAL::to_double(v->point().x());
            outVertices[vertexIndex].y = CGAL::to_double(v->point().y());
            outVertices[vertexIndex].time = CGAL::to_double(v->time());

            vertexIdToIndex[static_cast<int>(v->id())] = vertexIndex;

            ++vertexIndex;
        }

        int edgeIndex = 0;

        for (auto h = skeleton->halfedges_begin(); h != skeleton->halfedges_end(); ++h)
        {
            auto opposite = h->opposite();

            if (h->id() > opposite->id())
                continue;

            if (edgeIndex >= maxEdges)
                return 0;
            
            const int sourceId = static_cast<int>(opposite->vertex()->id());
            const int targetId = static_cast<int>(h->vertex()->id());

            const auto sourceIt = vertexIdToIndex.find(sourceId);
            const auto targetIt = vertexIdToIndex.find(targetId);

            if (sourceIt == vertexIdToIndex.end() || targetIt == vertexIdToIndex.end())
                continue;

            outEdges[edgeIndex].sourceVertexIndex = sourceIt->second;
            outEdges[edgeIndex].targetVertexIndex = targetIt->second;
            
            const bool bIsBisector = h->is_bisector() || opposite->is_bisector();
            const bool bIsInnerBisector = h->is_inner_bisector() || opposite->is_inner_bisector();
            outEdges[edgeIndex].isBisector = bIsBisector ? 1 : 0;
            outEdges[edgeIndex].isInnerBisector = bIsInnerBisector ? 1 : 0;

            ++edgeIndex;
        }

        *outVertexCount = vertexIndex;
        *outEdgeCount = edgeIndex;

        return 1;
    }
    
    extern "C"
    int CGALBridge_BuildStraightSkeletonFull(
        const Vec2* points,
        int count,

        SkeletonVertex* outVertices,
        int maxVertices,
        int* outVertexCount,

        SkeletonEdge* outEdges,
        int maxEdges,
        int* outEdgeCount,

        SkeletonFace* outFaces,
        int maxFaces,
        int* outFaceCount,

        int* outFaceVertexIndices,
        int maxFaceVertexIndices,
        int* outFaceVertexIndexCount,
        
        SkeletonTriangle* outTriangles,
        int maxTriangles,
        int* outTriangleCount
    )
    {
        if (!points || !outVertices || !outVertexCount ||
            !outEdges || !outEdgeCount ||
            !outFaces || !outFaceCount ||
            !outFaceVertexIndices || !outFaceVertexIndexCount ||
            !outTriangles || !outTriangleCount)
        {
            return 0;
        }
        
        *outVertexCount = 0;
        *outEdgeCount = 0;
        *outFaceCount = 0;
        *outFaceVertexIndexCount = 0;
        *outTriangleCount = 0;

        if (count < 3)
        {
            return 0;
        }

        Polygon2 poly;

        for (int i = 0; i < count; ++i)
        {
            poly.push_back(Point2(points[i].x, points[i].y));
        }

        if (!poly.is_simple())
        {
            return 0;
        }

        if (poly.is_clockwise_oriented())
        {
            poly.reverse_orientation();
        }

        auto skeleton = CGAL::create_interior_straight_skeleton_2(
            poly.vertices_begin(),
            poly.vertices_end());

        if (!skeleton)
        {
            return 0;
        }

        const int neededVertices = static_cast<int>(skeleton->size_of_vertices());

        if (neededVertices > maxVertices)
        {
            return 0;
        }

        std::unordered_map<int, int> vertexIdToIndex;

        int vertexIndex = 0;

        for (auto v = skeleton->vertices_begin(); v != skeleton->vertices_end(); ++v)
        {
            outVertices[vertexIndex].x = CGAL::to_double(v->point().x());
            outVertices[vertexIndex].y = CGAL::to_double(v->point().y());
            outVertices[vertexIndex].time = CGAL::to_double(v->time());

            vertexIdToIndex[static_cast<int>(v->id())] = vertexIndex;

            ++vertexIndex;
        }

        int edgeIndex = 0;

        for (auto h = skeleton->halfedges_begin(); h != skeleton->halfedges_end(); ++h)
        {
            auto opposite = h->opposite();

            if (h->id() > opposite->id())
            {
                continue;
            }

            if (edgeIndex >= maxEdges)
            {
                return 0;
            }

            const int sourceId = static_cast<int>(opposite->vertex()->id());
            const int targetId = static_cast<int>(h->vertex()->id());

            const auto sourceIt = vertexIdToIndex.find(sourceId);
            const auto targetIt = vertexIdToIndex.find(targetId);

            if (sourceIt == vertexIdToIndex.end() || targetIt == vertexIdToIndex.end())
            {
                continue;
            }

            const bool bIsBisector =
                h->is_bisector() || opposite->is_bisector();

            const bool bIsInnerBisector =
                h->is_inner_bisector() || opposite->is_inner_bisector();

            outEdges[edgeIndex].sourceVertexIndex = sourceIt->second;
            outEdges[edgeIndex].targetVertexIndex = targetIt->second;
            outEdges[edgeIndex].isBisector = bIsBisector ? 1 : 0;
            outEdges[edgeIndex].isInnerBisector = bIsInnerBisector ? 1 : 0;

            ++edgeIndex;
        }

        int faceIndex = 0;
        int faceVertexIndex = 0;

        for (auto f = skeleton->faces_begin(); f != skeleton->faces_end(); ++f)
        {
            if (faceIndex >= maxFaces)
            {
                return 0;
            }

            auto start = f->halfedge();
            auto h = start;

            const int firstFaceVertexIndex = faceVertexIndex;
            int localCount = 0;

            do
            {
                if (faceVertexIndex >= maxFaceVertexIndices)
                {
                    return 0;
                }

                const int vertexId = static_cast<int>(h->vertex()->id());
                const auto it = vertexIdToIndex.find(vertexId);

                if (it == vertexIdToIndex.end())
                {
                    return 0;
                }

                outFaceVertexIndices[faceVertexIndex] = it->second;

                ++faceVertexIndex;
                ++localCount;

                h = h->next();

            } while (h != start);

            if (localCount >= 3)
            {
                outFaces[faceIndex].firstIndex = firstFaceVertexIndex;
                outFaces[faceIndex].indexCount = localCount;
                ++faceIndex;
            }
            else
            {
                faceVertexIndex = firstFaceVertexIndex;
            }
        }

        *outVertexCount = vertexIndex;
        *outEdgeCount = edgeIndex;
        *outFaceCount = faceIndex;
        *outFaceVertexIndexCount = faceVertexIndex;
        
        int triangleIndex = 0;

        for (int fi = 0; fi < faceIndex; ++fi)
        {
            const SkeletonFace& face = outFaces[fi];

            if (face.indexCount < 3)
            {
                continue;
            }

            std::vector<int> polygonVertexIndices;
            polygonVertexIndices.reserve(face.indexCount);

            for (int i = 0; i < face.indexCount; ++i)
            {
                polygonVertexIndices.push_back(outFaceVertexIndices[face.firstIndex + i]);
            }

            // remove consecutive duplicates
            polygonVertexIndices.erase(
                std::unique(
                    polygonVertexIndices.begin(),
                    polygonVertexIndices.end()
                ),
                polygonVertexIndices.end()
            );
            
            if (polygonVertexIndices.size() >= 2 && polygonVertexIndices.front() == polygonVertexIndices.back())
            {
                polygonVertexIndices.pop_back();
            }
            
            if (polygonVertexIndices.size() < 3)
            {
                continue;
            }

            if (polygonVertexIndices.size() == 3)
            {
                if (triangleIndex >= maxTriangles)
                {
                    return 0;
                }

                outTriangles[triangleIndex].a = polygonVertexIndices[0];
                outTriangles[triangleIndex].b = polygonVertexIndices[1];
                outTriangles[triangleIndex].c = polygonVertexIndices[2];
                ++triangleIndex;

                continue;
            }

            Polygon2 facePolygon;

            for (const int vi : polygonVertexIndices)
            {
                facePolygon.push_back(Point2(outVertices[vi].x, outVertices[vi].y));
            }
            
            if (!facePolygon.is_simple())
            {
                continue;
            }

            const bool bFaceIsCCW = facePolygon.is_counterclockwise_oriented();

            CDT cdt;
            std::vector<CDT::Vertex_handle> handles;
            handles.reserve(polygonVertexIndices.size());

            for (const int vi : polygonVertexIndices)
            {
                const Point2 p(outVertices[vi].x, outVertices[vi].y);
                CDT::Vertex_handle handle = cdt.insert(p);
                handle->info() = vi;
                handles.push_back(handle);
            }

            for (size_t i = 0; i < handles.size(); ++i)
            {
                const size_t next = (i + 1) % handles.size();
                cdt.insert_constraint(handles[i], handles[next]);
            }

            for (auto fit = cdt.finite_faces_begin(); fit != cdt.finite_faces_end(); ++fit)
            {
                const Point2 p0 = fit->vertex(0)->point();
                const Point2 p1 = fit->vertex(1)->point();
                const Point2 p2 = fit->vertex(2)->point();

                const Point2 center(
                    (p0.x() + p1.x() + p2.x()) / 3.0,
                    (p0.y() + p1.y() + p2.y()) / 3.0
                );
                
                if (facePolygon.bounded_side(center) == CGAL::ON_UNBOUNDED_SIDE)
                {
                    continue;
                }

                int a = fit->vertex(0)->info();
                int b = fit->vertex(1)->info();
                int c = fit->vertex(2)->info();

                const double signedArea =
                    (outVertices[b].x - outVertices[a].x) * (outVertices[c].y - outVertices[a].y) -
                    (outVertices[b].y - outVertices[a].y) * (outVertices[c].x - outVertices[a].x);

                const bool bTriangleIsCCW = signedArea > 0.0;

                if (bTriangleIsCCW != bFaceIsCCW)
                {
                    std::swap(b, c);
                }

                if (triangleIndex >= maxTriangles)
                {
                    return 0;
                }

                outTriangles[triangleIndex].a = a;
                outTriangles[triangleIndex].b = b;
                outTriangles[triangleIndex].c = c;
                ++triangleIndex;
            }
        }

        *outTriangleCount = triangleIndex;

        return 1;
    }
    
    int CGALBridge_TriangulatePolygonWithHoles(
        const Vec2* points, 
        int pointCount, 
        
        const int* loopFirstIndices,
        const int* loopVertexCounts, 
        int loopCount,
        
        SkeletonTriangle* outTriangles, 
        int maxTriangles,
        int* outTriangleCount
    )
    {
        if (!points ||
            pointCount <= 0 ||
            !loopFirstIndices ||
            !loopVertexCounts ||
            loopCount <= 0 ||
            !outTriangles ||
            maxTriangles <= 0 ||
            !outTriangleCount)
        {
            return 0;
        }
        
        *outTriangleCount = 0;
        
        TriCdt cdt;

        for (int loopIndex = 0; loopIndex < loopCount; ++loopIndex)
        {
            const int firstIndex = loopFirstIndices[loopIndex];
            const int vertexCount = loopVertexCounts[loopIndex];

            if (firstIndex < 0 || vertexCount < 3 || firstIndex + vertexCount > pointCount)
            {
                return 0;
            }

            std::vector<TriCdt::Vertex_handle> handles;
            handles.reserve(vertexCount);

            for (int localIndex = 0; localIndex < vertexCount; ++localIndex)
            {
                const int pointIndex = firstIndex + localIndex;
                const Vec2& p = points[pointIndex];

                TriCdt::Vertex_handle handle = cdt.insert(Point2(p.x, p.y));
                handle->info() = pointIndex;

                handles.push_back(handle);
            }

            for (int localIndex = 0; localIndex < vertexCount; ++localIndex)
            {
                const int nextLocalIndex = (localIndex + 1) % vertexCount;

                cdt.insert_constraint(
                    handles[localIndex],
                    handles[nextLocalIndex]
                );
            }
        }
        
        MarkDomains(cdt);

        int triangleCount = 0;

        for (auto face = cdt.finite_faces_begin(); face != cdt.finite_faces_end(); ++face)
        {
            if (!face->info().in_domain())
            {
                continue;
            }

            if (triangleCount >= maxTriangles)
            {
                return 0;
            }

            outTriangles[triangleCount].a = face->vertex(0)->info();
            outTriangles[triangleCount].b = face->vertex(1)->info();
            outTriangles[triangleCount].c = face->vertex(2)->info();

            ++triangleCount;
        }

        *outTriangleCount = triangleCount;
        return 1;
    }
}