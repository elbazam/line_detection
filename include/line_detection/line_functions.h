#ifndef LINE_FUNCTIONS_H_
#define LINE_FUNCTIONS_H_

#include <vector>
#include <string>
#include <math.h>
#include <iostream>



class point2D 
{
    public:
        float x;
        float y;
        bool intersect;
    point2D();
};

class lineParams
{
    public:
        float a;
        float b;
        float c;
        
    lineParams();
};

class straightLine
{
    public:
        int start;
        int end;
        lineParams params;
        std::vector<point2D> points;
        point2D realStartPoint;
        point2D realEndPoint;

    straightLine();
};

class unitedLineIndexes
{
    public:
        int start;
        int end;
    unitedLineIndexes();
};

class wholeLine
{
    public:
        point2D firstPoint;
        point2D lastPoint;
    wholeLine();
};

double distanceBetweenTwoPoints(const point2D point1 , const point2D point2);

lineParams fitLineToMeasurements(const std::vector<point2D> points);

double distanceFromLine(const point2D point , const lineParams params);

double maximunDistanceToLine(const std::vector<point2D> points ,
                             const lineParams params);
point2D createPredictedPosition( const point2D measurement ,const lineParams params);
double distanceMeasuredToPredictedPosition(const point2D measurement ,
                                           const lineParams params);

void insertPointsToLineCandidate(const int start , const int end ,
                                 const std::vector<point2D> points , straightLine& seed);

double lineLength(const straightLine line);

void updateRegions(straightLine& Lcurrent , straightLine& LNext , int k);

bool checkIfSame(const lineParams candidate , const lineParams previous);

point2D lineIntersection(const lineParams params1 , const lineParams params2 , float threshold = 0.02);

#endif