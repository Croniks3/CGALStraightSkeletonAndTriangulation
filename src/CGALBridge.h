#pragma once

#ifdef _WIN32
    #ifdef CGALBRIDGE_EXPORTS
        #define CGALBRIDGE_API __declspec(dllexport)
    #else
        #define CGALBRIDGE_API __declspec(dllimport)
    #endif
#else
    #define CGALBRIDGE_API
#endif

namespace CGALBridge
{
    struct Vec2
    {
        double x;
        double y;
    };

    struct SkeletonVertex
    {
        double x;
        double y;
        double time;
    };
    
    struct SkeletonEdge
    {
        int sourceVertexIndex;
        int targetVertexIndex;
        int isBisector;
        int isInnerBisector;
    };
    
    struct SkeletonFace
    {
        int firstIndex;
        int indexCount;
    };
    
    struct SkeletonTriangle
    {
        int a;
        int b;
        int c;
    };
    
    extern "C"
    {
        // Строит только вершины. Возвращает количество вершин skeleton.
        CGALBRIDGE_API int CGALBridge_BuildStraightSkeleton(
            const Vec2* points,
            int count,
            SkeletonVertex* outVertices,
            int maxVertices
        );
        
        // Строит вершины + грани. Возвращает флаг построился skeleton или нет.
        CGALBRIDGE_API int CGALBridge_BuildStraightSkeletonGraph(
            const Vec2* points,
            int count,
            SkeletonVertex* outVertices,
            int maxVertices,
            int* outVertexCount,
            SkeletonEdge* outEdges,
            int maxEdges,
            int* outEdgeCount
        );
        
        // Строит вершины + грани + поверхности + треугольники. Возвращает флаг построился skeleton или нет.
        CGALBRIDGE_API int CGALBridge_BuildStraightSkeletonFull(
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
        );
    }
}