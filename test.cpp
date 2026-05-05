#include <iostream>
#include "CGALBridge.h"

int main()
{
    CGALBridge::Vec2 pts[4] = {
        {0,0}, {100,0}, {100,100}, {0,100}
    };
    
    CGALBridge::SkeletonVertex out[64];

    int count = CGALBridge_BuildStraightSkeleton(pts, 4, out, 64);
    
    std::cout << "Vertices: " << count << "\n";

    for (int i = 0; i < count; ++i)
    {
        std::cout << out[i].x << " " << out[i].y << " t=" << out[i].time << "\n";
    }
	
	std::cout << "Press Enter...\n";
	std::cin.get();

    return 0;
}