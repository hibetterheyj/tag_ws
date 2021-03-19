#ifndef CCIRCLEDETECT_H
#define CCIRCLEDETECT_H

#include "CRawImage.h"
#include <math.h>

#define COLOR_PRECISION 32
#define COLOR_STEP 8
#define INNER 0
#define OUTER 1
#define MAX_PATTERNS 50

// this structure contains information related to image coordinates and dimensions of the detected pattern
typedef struct{
    float x;			// center in image coordinates
    float y;			// center in image coordinates
    float angle,horizontal;	// orientation (not really used in this case, see the SwarmCon version of this software)
    int size;			// number of pixels
    int maxy,maxx,miny,minx;	// bounding box dimensions
    int mean;			// mean brightness
    int type;			// black or white ?
    float roundness;		// result of the first roundness test, see Eq. 2 of paper [1]
    float bwRatio;		// ratio of white to black pixels, see Algorithm 2 of paper [1]
    bool round;			// segment passed the initial roundness test
    bool valid;			// marker passed all tests and will be passed to the transformation phase
    float m0,m1;		// eigenvalues of the pattern's covariance matrix, see Section 3.3 of [1]
    float v0,v1;		// eigenvectors of the pattern's covariance matrix, see Section 3.3 of [1]
    float r0,r1;		// ratio of inner vs outer ellipse dimensions (used to establish ID, see the SwarmCon version of this class)
    int ID;			// pattern ID (experimental, see the SwarmCon version of this class)
} SSegment;

class CCircleDetect
{
    public:
        //constructor, wi and he correspond to the image dimensions 
        CCircleDetect(int wi,int he, bool id, int bits, int samples, int dist);

        //deallocate the detector's structures
        ~CCircleDetect();

        // dynamic reconfigure of parameters
        void reconfigure(float ict,float fct,float art,float cdtr,float cdta, bool id, int minS);

        SSegment getInnerSegment();

        //main detection method, implements Algorithm 2 of [1] 
        SSegment findSegment(CRawImage* image, SSegment init);

        //local pattern search - implements Algorithm 1 of [1]
        bool examineSegment(CRawImage* image,SSegment *segmen,int ii,float areaRatio);

        //calculate the pattern dimensions by means of eigenvalue decomposition, see 3.3 of [1]
        SSegment calcSegment(SSegment segment,int size,long int x,long int y,long int cm0,long int cm1,long int cm2);

        //cleanup the shared buffers - see 3.6 of [1] 
        void bufferCleanup(SSegment init);

        //load descriptions for circle ID's
        int loadCircleID(const char* id);

        //change threshold if circle not detected, see 3.2 of [1]
        bool changeThreshold();

        //normalise the angle
        float normalizeAngle(float a);

        // adjust the dimensions of the image, when the image size changes
        int adjustDimensions(int wi, int he);

        bool draw,lastTrackOK;      // flags to draw results - used for debugging
        int debug;                  // debug level
        bool localSearch;           // used when selecting the circle by mouse click
        bool identify;              // attempt to identify segments
    private:
        //see the constructor in CCircleDetection.cpp for description of the following parameters
        int idBits;
        int idSamples;
        int hammingDist;
        bool track;
        int maxFailed;
        int numFailed;
        int threshold; 

        int minSize; 
        int lastThreshold; 
        int thresholdBias; 
        int maxThreshold; 

        int thresholdStep;
        float circularTolerance;
        float circularityTolerance;
        float ratioTolerance;
        float centerDistanceToleranceRatio;
        int centerDistanceToleranceAbs;
        bool enableCorrections;

        int ID;
        int step;
        SSegment inner;
        SSegment outer;
        float outerAreaRatio,innerAreaRatio,areasRatio;
        int queueStart,queueEnd,queueOldStart,numSegments;
        int width,height,len,siz;
        int expand[4];
        unsigned char *ptr;
        int tima,timb,timc,timd,sizer,sizerAll;
        float diameterRatio;
        bool ownBuffer;
        static int *buffer;
        static int *queue;
        // static int *mask;
        // static int maskNum;
        float idx[MAX_PATTERNS];
        float idy[MAX_PATTERNS];
        int numberIDs;
};

#endif

/* end of CCircleDetect.h */
