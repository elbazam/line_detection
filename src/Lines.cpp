#include "Lines.h"


// Init
Lines::Lines(): epsilon(0.03) , delta(0.1) , Snum(6) , Pmin(10) , Np(360) , Lmin(0.4){}
Lines::Lines(float epsilon_ , float delta_ , int Snum_ , int Pmin_ , int Np_ , double Lmin_):
                epsilon(epsilon_) , delta(delta_) , Snum(Snum_) , Pmin(Pmin_) , Np(Np_) , Lmin(Lmin_){}

void Lines::uploadData(std::vector<point2D> Measurements_)
{
    Measurements = Measurements_;
}

void Lines::seedSegmentDetection()
{
    int stopIndex = Np - Snum;
    double lineDistance;
    for (int start = 0 ; start < stopIndex ; start++){
        bool flag = true;
        int end = start + Snum - 1;
        
        straightLine seedCandidate;
        for (int jj = start; jj <= end; jj++)
        {
            if (Measurements[jj].x > 30.0)
            {
                flag = false;
                break;
            }
        }
        insertPointsToLineCandidate(start , end , Measurements , seedCandidate);
        seedCandidate.params = fitLineToMeasurements(seedCandidate.points);
        for (int k = start ; k <= end; k++){
            if (!flag){break;}
            lineDistance = distanceFromLine(Measurements[k] , seedCandidate.params);
            double predictedDistance = distanceMeasuredToPredictedPosition(Measurements[k],seedCandidate.params);

            if (lineDistance > epsilon || predictedDistance > delta){flag = false;}    
        }
        if (flag){
            Seeds.push_back(seedCandidate);
        }
    }
}

void Lines::regionGrowing()
{
    int seedNumber = Lines::countSeedNumber();
    double Ll;
    int Pl;
    int Np = Measurements.size();
    double lineDistance;
    for (int i = 0 ; i < seedNumber ; i++)
    {
        straightLine regionCandidate = Seeds[i];    
        for (int end = regionCandidate.end + 1 ; end < Np ; end++)
        {   
            if (Measurements[end].x > 30.0){break;}
            lineDistance = distanceFromLine(Measurements[end] , regionCandidate.params);
            double predictedDistance = distanceMeasuredToPredictedPosition(Measurements[end],
                                                                           regionCandidate.params);
            if (predictedDistance > delta || lineDistance > epsilon){break;}
            
            std::vector<point2D> testPoints = regionCandidate.points;
            testPoints.push_back(Measurements[end]);
            lineParams testParams = fitLineToMeasurements(testPoints);

            lineDistance = distanceFromLine(Measurements[regionCandidate.start] , testParams);
            if (lineDistance > epsilon){break;}
            
            
            regionCandidate.points = testPoints;
            regionCandidate.params = testParams;
            regionCandidate.end += 1;
        }
        
        for (int start = regionCandidate.start -1 ; start >= 0 ; start--)
        {
            if (Measurements[start].x > 30.0){break;}
            lineDistance = distanceFromLine(Measurements[start] , regionCandidate.params);
            double predictedDistance = distanceMeasuredToPredictedPosition(Measurements[start],regionCandidate.params);
            if (predictedDistance > delta || lineDistance > epsilon){break;}
            std::vector<point2D> testPoints = regionCandidate.points;
            testPoints.push_back(Measurements[start]);
            lineParams testParams = fitLineToMeasurements(testPoints);

            lineDistance = distanceFromLine(Measurements[regionCandidate.end] , testParams);
            if (lineDistance > epsilon){break;}
            regionCandidate.points = testPoints;
            regionCandidate.params = testParams;
            regionCandidate.start -= 1;
        }
        
        Pl = regionCandidate.points.size();
        Ll = lineLength(regionCandidate);

        if (Ll >= Lmin && Pl >= Pmin)
        {
            Regions.push_back(regionCandidate);
        }

    }
}

void Lines::overlapRegionProcessing()
{
    int Nl = Regions.size();
    
    for (int current = 0 ; current < Nl - 1 ; current++)
    {
        int next = current + 1;
        if (Regions[current].points.size() == 0 || Regions[next].points.size() == 0)
        {
            continue;
        }

        int n1 = Regions[current].end;
        int m2 = Regions[next].start;
        int k = m2;
        
        if (m2 <= n1 )
        {
            for ( ; k <= n1 ; k++ )
            {
                double dCurrent = distanceMeasuredToPredictedPosition(Measurements[k] , Regions[current].params);
                double dNext = distanceMeasuredToPredictedPosition(Measurements[k] , Regions[next].params);
                if (dCurrent > dNext)
                {
                    break;
                }
            }
            updateRegions(Regions[current] , Regions[next] , k);     
            // Regions[current].params = fitLineToMeasurements(Regions[current].points);
            // Regions[next].params = fitLineToMeasurements(Regions[next].points);
        }
    }

    int RegionSize = Regions.size();
    for (int j = 0; j < RegionSize ; j++)
    {
        if (Regions[j].points.size() > 1)
        {
            double regionLength = lineLength(Regions[j]);
            if ( regionLength >= Lmin)
            {
                finishedLines.push_back(Regions[j]);
            }
        }
        
    }

}

void Lines::cleanSameLines()
{   
    int RegionSize = Regions.size();
    for (int i = 0; i < RegionSize ; i ++)
    {
        for (int k = RegionSize - 1; k > 0; k--)
        {
            if (k == i){continue;}
            RegionSize = Regions.size();
            if (Regions[i].start <= Regions[k].start && Regions[i].end >= Regions[k].end)
            {
                Regions.erase(Regions.begin() + k);
            }
        }

    }
}

void Lines::joinLines(double threshold)
{

    int numberOfSegments = finishedLines.size();
    for (int current = 0; current < numberOfSegments-1 ; current++)
    {

        int next = current+1;
        int endCurrent = finishedLines[current].end;
        int startNext = finishedLines[next].start;
        double dist = distanceBetweenTwoPoints(Measurements[endCurrent],Measurements[startNext]);
        if (dist > threshold){continue;}
        if (!checkIfSame(finishedLines[current].params , finishedLines[next].params)){continue;}
        
        
        finishedLines[next].start = finishedLines[current].start;
        finishedLines[next].points.insert(finishedLines[next].points.end() ,
                                          finishedLines[current].points.begin() ,
                                          finishedLines[current].points.end());
        finishedLines[next].params = fitLineToMeasurements(finishedLines[next].points);

        finishedLines.erase(finishedLines.begin() + current);
    }

}

void Lines::find()
{
    seedSegmentDetection();
    regionGrowing();
    cleanSameLines();
    overlapRegionProcessing();
    joinLines();
}


int Lines::countSeedNumber(){return Seeds.size();}
int Lines::countRegions(){return Regions.size();}
std::vector<straightLine> Lines::getSeeds(){return Seeds;}
std::vector<straightLine> Lines::getRegions(){return Regions;}
std::vector<straightLine> Lines::getFinishedLines(){return finishedLines;}
std::vector<point2D> Lines::getMeasurements(){return Measurements;}
