#include "line_functions.h"

point2D::point2D(){}
lineParams::lineParams(){}
straightLine::straightLine(){}
unitedLineIndexes::unitedLineIndexes(){}

double distanceBetweenTwoPoints(const point2D point1 , const point2D point2)
{
    double dxSqr = pow(point1.x - point2.x,2);
    double dySqr = pow(point1.y - point2.y,2);
    return pow(dxSqr + dySqr , 0.5);
}

lineParams fitLineToMeasurements(const std::vector<point2D> points)
{
    lineParams params;
    int SeedSize = points.size();
    float sumX = 0.0; /* sum of x      */
    float sumX2 = 0.0; /* sum of x^2   */
    float sumXY = 0.0; /* sum of x * y */
    float sumY = 0.0;  /* sum of y     */
    float sumY2 = 0.0;  /* sum of y     */
    // float rSqr = 0.0;
    for (int i = 0 ; i < SeedSize ; i++){
        sumX += points[i].x;
        sumX2 += points[i].x * points[i].x;
        sumXY += points[i].x * points[i].y;
        sumY += points[i].y;
        sumY2 += points[i].y * points[i].y;
    }



    // rSqr = (SeedSize * sumXY - sumX * sumY) / ((SeedSize*sumX2 - sumX*sumX)*(SeedSize*sumY2 - sumY*sumY));
    // params.r = rSqr * (SeedSize * sumXY - sumX * sumY);
    float xMean = sumX / SeedSize;
    float yMean = sumY / SeedSize;
    float denominator = sumX2 - sumX * xMean;
    
    params.b = -1.0;
    params.a = (sumXY - sumX * yMean) / denominator;
    params.c = yMean - params.a * xMean;

    float radToDeg = 180 / M_PI;
    bool reverseRoles = abs(abs(atan2(params.a,params.b)) - M_PI_2)*radToDeg < 45.0;

    if (reverseRoles)
    {
        float denominator2 = sumY2 - sumY * yMean;
        params.a = -1.0;
        params.b = (sumXY - sumY * xMean) / denominator2;
        params.c = xMean - params.b * yMean;
    }
    

    // Normalize line parameters. 
    double normal = pow((params.b * params.b + params.a * params.a + params.c * params.c),0.5);
    params.a /= normal;
    params.b /= normal;
    params.c /= normal;

    return params;
}

double distanceFromLine(const point2D point , const lineParams params)
{
    double numerator   = abs(params.a * point.x + params.b * point.y + params.c);
    double denominator = pow(pow(params.a,2) + pow(params.b,2),0.5);
    // d = |a*x + b*y + c|/[(a^2 + b^2)^0.5]
    double distance = numerator / denominator;
    return distance;
}

double maximunDistanceToLine(const std::vector<point2D> points ,const lineParams params)
{
    double maxDist = 0;
    int numberOfMeasurements = points.size();

    for (int i = 0; i < numberOfMeasurements ; i++){
        double dist = distanceFromLine(points[i] , params);
        if (dist > maxDist) {maxDist = dist;}
    }
    return maxDist;
}

point2D createPredictedPosition( const point2D measurement ,const lineParams params)
{
    /*Create predicted position on a line.*/
    point2D PredictedPosition;
    double theta = atan2(measurement.y , measurement.x);
    double numerator   = -params.c;
    double denominator = params.a * cos(theta) + params.b * sin(theta);
    if (denominator != 0)
    {
        double ratio = numerator / denominator;
        // x = ratio * cos(theta) ; y = ratio * sin(theta)
        PredictedPosition.x = ratio * cos(theta);
        PredictedPosition.y = ratio * sin(theta);
    }
    else 
    {
        if (params.b == 0) //  Vertical line.
        {

            PredictedPosition.x = -params.c / params.a;
            PredictedPosition.y = 0;
        }
        else if(params.a == 0) // Horizontal line.
        {
            PredictedPosition.x = 0;
            PredictedPosition.y = -params.c / params.b;
        }
        else // Trying to estimate the line from being on the line itself. 
        {
            
            PredictedPosition.x = 0;
            PredictedPosition.y = 0;
        }
    }
    return PredictedPosition;
}

double distanceMeasuredToPredictedPosition(const point2D measurement ,const lineParams params)
{
    /*Calculates distance from measured potision to predicted position*/
    point2D PredictedPosition = createPredictedPosition(measurement , params);
    return distanceBetweenTwoPoints(measurement , PredictedPosition);
}

void insertPointsToLineCandidate(const int start , const int end , const std::vector<point2D> points , straightLine& seed){
    
    seed.end = end;
    seed.start = start;
    // Include the end index:
    for (int i = start ; i <= end; i++)
    {
        seed.points.push_back(points[i]);
    }
}

double lineLength(const straightLine line)
{
    int last = line.points.size() - 1;
    return distanceBetweenTwoPoints(line.points[last] , line.points[0]);
}

void updateRegions(straightLine& Lcurrent , straightLine& LNext , int k)
{
    
    for (int i = Lcurrent.end ; i > k - 1 ; i--)
    {
        Lcurrent.points.erase(Lcurrent.points.end()-1);
        if (Lcurrent.points.size() == 0){break;}
    }
    
    Lcurrent.end = k;
    for (int i = LNext.start ; i < k ; i++)
    {
        LNext.points.erase(LNext.points.begin());
        if (LNext.points.size() == 0){break;}
    }
    LNext.start = k + 1;
    
}

bool checkIfSame(const lineParams candidate , const lineParams previous)
{

    // float parallelLinesError = candidate.a * previous.b - candidate.b * previous.a;
    // if (parallelLinesError > 0.01){return false;}
    float error = 0;
    error += pow(abs(candidate.a - previous.a),2);
    error += pow(abs(candidate.b - previous.b),2);
    error += pow(abs(candidate.c - previous.c),2);
    error = pow(error,0.5);

    return error < 0.02;
    
}
