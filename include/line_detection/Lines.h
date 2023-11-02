#ifndef LINES_H
#define LINES_H

#include <vector>
#include <iostream>
#include "line_functions.h"
#include <math.h>
#include <fstream>


class Lines {

    private:
        // minimum accepted l1 distance
        
        float epsilon;
        float delta;
        int Snum;
        int Pmin;
        int Np;
        int Lmin;
        std::vector<straightLine> Seeds;
        std::vector<straightLine> Regions;
        std::vector<straightLine> finishedLines;
        std::vector<point2D> Measurements;
    
    public:


        // default constractor:
        Lines();

        // Full constractor:
        Lines(float epsilon_ , float delta_ , int Snum_ = 6 ,
             int Pmin_ = 10 , int Np_ = 360 , double Lmin_ = 0.4);

        

        void uploadData(std::vector<point2D> Measurements_);

        void seedSegmentDetection();

        int countSeedNumber();
        int countRegions();

        void regionGrowing();
        void cleanSameLines();
        void overlapRegionProcessing();
        void joinLines(double threshold = 0.2);
        void fixLines();
        void find();

        std::vector<straightLine> getSeeds();
        std::vector<straightLine> getRegions();
        std::vector<point2D> getMeasurements();
        std::vector<straightLine> getFinishedLines();
};





#endif
