/* Given a funnel and a description of obstacles (in terms of vertices), 
 * returns the minimum distance needed to bring the funnel out of collision (f)
 * along with the gradient of this function w.r.t. the funnel's starting position. 
 * So, for example, if the funnel is just on the boundary of collition, we have f = 0.

Inputs:
x: state from which funnel is executed (only the cyclic dimensions matter, really)

forest: cell array containing vertices of obstacles

funnelLibrary: struct containing funnel library
	funnelLibrary(*).xyz: xyz positions at time points (double 3 X N)
	funnelLibrary(*).cS: cholesky factorization of projection of S matrix (cell array)

funnelIdx: index of funnel to be checked

Outputs: f, df (as described above).

Author: Anirudha Majumdar
Date: March 6, 2015
*/

// BLAS
#if !defined(_WIN32)
#define dgemm dgemm_
#endif
#include <mex.h>
#include <blas.h>
#include <math.h>
#include <matrix.h>

// Timer functions
// #include <chrono>
// #include <ctime>
// #include <time.h>

// Internal access to bullet
#include "LinearMath/btTransform.h"

#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"
// #include <iostream>

#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btPointCollector.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "LinearMath/btTransformUtil.h"

using namespace std;

// Set solvers for bullet
static btVoronoiSimplexSolver sGjkSimplexSolver;
static btGjkEpaPenetrationDepthSolver epaSolver;

/*Sphere of radius r representing the point. 
Bullet expects things to be non-degenrate, so we have non-zero radius.
*/
static const double g_radius = 1.0; 
static btSphereShape* g_point = new btSphereShape(g_radius); 

double ptToPolyBullet(double *vertsPr, size_t nRows, size_t nCols, mxArray *normal_vec){

	// Initialize polytope object with single point
	btConvexHullShape polytope(btVector3(vertsPr[0],vertsPr[1],vertsPr[2]), 1);
    
	// Add rest of the points (note the indexing starts from 1 on the loop)
	for(int i=1;i<nCols;i++){
			polytope.addPoint(btVector3(vertsPr[i*nRows],vertsPr[i*nRows+1],vertsPr[i*nRows+2]));

	}

	 // Assign elements of verts (input) to polytope
	btTransform tr;
	btGjkPairDetector::ClosestPointInput input; 
	tr.setIdentity();
	input.m_transformA = tr;
	input.m_transformB = tr;

	btGjkPairDetector convexConvex(g_point,&polytope,&sGjkSimplexSolver,&epaSolver);
		
	// Output
	btPointCollector gjkOutput; 

	convexConvex.getClosestPoints(input, gjkOutput, 0);
    
    // mexPrintf("1: %f\n", gjkOutput.m_normalOnBInWorld[0]);
    // mexPrintf("2: %f\n", gjkOutput.m_normalOnBInWorld[1]);
    // mexPrintf("3: %f\n", gjkOutput.m_normalOnBInWorld[2]);
    
    double *normal_vec_d = mxGetPr(normal_vec); 
    
    normal_vec_d[0] = gjkOutput.m_normalOnBInWorld[0];
    normal_vec_d[1] = gjkOutput.m_normalOnBInWorld[1];
    normal_vec_d[2] = gjkOutput.m_normalOnBInWorld[2];
	
    return gjkOutput.m_distance + CONVEX_DISTANCE_MARGIN + g_radius;

}


/* Shift and transform vertices
 */
double *shiftAndTransform(double *verts, double *vertsT, const mxArray *x, mxArray *x0, int k, mxArray *cSk, size_t nRows, size_t nCols)
{
    /* This can and maybe should be sped up. For example, we know that cSk is upper triangular.*/
    double *dcSk = mxGetPr(cSk);
    double *dx0 = mxGetPr(x0);
    double *dx = mxGetPr(x); 
    
    for(int i=0;i<nRows;i++){
        for(int j=0;j<nCols;j++){
            vertsT[j*nRows+i] = dcSk[i]*(verts[j*nRows+0]-dx0[k*nRows+0]-dx[0]) + dcSk[nRows+i]*(verts[j*nRows+1]-dx0[k*nRows+1]-dx[1]) + dcSk[2*nRows+i]*(verts[j*nRows+2]-dx0[k*nRows+2]-dx[2]);
            // mexPrintf("i: %d, j: %d, val: %f\n", i, j, vertsT[j*nRows+i]);
            // mexPrintf("i: %d, j: %d, val: %f\n", i, j, dx0[k*nRows]);
        }
    }

    
    return vertsT;
  
}



/* Computes f and df (as described in comments at the top). Also returns a boolean which is true if 
 * the funnel is collision free and false otherwise
*/
bool isCollisionFree(int funnelIdx, const mxArray *x, const mxArray *funnelLibrary, const mxArray *forest, mwSize numObs, double *min_dist, double *normal_vec_transformed)
{

// Initialize some variables
double *verts; // cell element (i.e. vertices)
mxArray *x0 = mxGetField(funnelLibrary, funnelIdx, "xyz"); // all points on trajectory
mxArray *obstacle;
mxArray *cS = mxGetField(funnelLibrary, funnelIdx, "cS");
mxArray *cSk;
size_t nCols;
size_t nRows;
double distance;
mxArray *normal_vec;
normal_vec = mxCreateDoubleMatrix(1,3,mxREAL);

// Get number of time samples
mwSize N = mxGetNumberOfElements(mxGetField(funnelLibrary, funnelIdx, "cS"));

// Check size compatibilities
const mwSize *N_x0 = mxGetDimensions(x0); 
if (N_x0[1] != N) {
        mexErrMsgTxt("Sizes of x0 and cS do not match!");
}


// Initialize collFree to true
bool collFree = true;


// For each time sample, we need to check if we are collision free
// mxArray *collisions = mxCreateLogicalMatrix(numObs,N);

for(int k=0;k<N;k++) 
{
    // Get pointer to cholesky factorization of S at this time
    cSk = mxGetCell(cS,k);

    for(mwIndex jForest=0;jForest<numObs;jForest++)
    {

    // Get vertices of this obstacle
    obstacle = mxGetCell(forest, jForest);
    verts = mxGetPr(mxGetCell(forest, jForest)); // Get vertices
    nCols = mxGetN(obstacle);
    nRows = mxGetM(obstacle);
    
    double *vertsT = mxGetPr(mxCreateDoubleMatrix(nRows, nCols, mxREAL));
        
    // Shift vertices so that point on trajectory is at origin and transform by cholesky of S
    vertsT = shiftAndTransform(verts, vertsT, x, x0, k, cSk, nRows, nCols);

    // Call bullet to find f (distance) and df (normal_vec)
    distance = ptToPolyBullet(vertsT, nRows, nCols, normal_vec); 
    
    // Update min_dist
    if(distance < *min_dist){
        *min_dist = distance;
        
        // Multiply normal_vec by cSk to get it back in the correct coordinate frame (i.e., normal_vec'*cSk)
        double one = 1.0, zero = 0.0; // Seriously?
        long int ione = 1;
        long int dim = 3;
        
        char *chn = "N";
        dgemm(chn, chn, &ione, &dim, &dim, &one, mxGetPr(normal_vec), &ione, mxGetPr(cSk), &dim, &zero, normal_vec_transformed, &ione);

    }
    
        
    }
}

if(*min_dist < 1){
    collFree = false;
}

return collFree;

}

/* Main mex funtion*/
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{

// mexPrintf("prhs:%d \n",prhs[3]);
    
    
// Get x (state) from which funnel is to be executed
const mxArray *x;
x = prhs[0];
// x = mxGetPr(prhs[0]);
// mexPrintf("x: %f \n", x); 


// Deal with forest cell (second input)
const mxArray *forest; // Cell array containing forest (pointer)

forest = prhs[1]; // Get forest cell (second input)
mwSize numObs = mxGetNumberOfElements(forest); // Number of obstacles


// Now deal with funnel library object (third input)
const mxArray *funnelLibrary; // Funnel library (pointer)

funnelLibrary = prhs[2]; // Get funnel library (third input)

// Get funnelIdx (subtract 1 since index is coming from matlab)
int funnelIdx;
funnelIdx = (int )mxGetScalar(prhs[3]);
funnelIdx = funnelIdx-1;

// Initialize min_dist (f)
double min_dist = 1000000.0;

// Initialize normal vector (df)
mxArray *normal_vec = mxCreateDoubleMatrix(1,3,mxREAL);
double *normal_vec_d = mxGetPr(normal_vec);

// Initialize next funnel (output of this function)
bool collFree = isCollisionFree(funnelIdx, x, funnelLibrary, forest, numObs, &min_dist, normal_vec_d);


// Return collFree
plhs[0] = mxCreateLogicalScalar(collFree);

// mexPrintf("min_dist: %f", min_dist); 

// Return min_dist if asked for
if (nlhs > 1){
        plhs[1] = mxCreateDoubleScalar(min_dist);
}

// Return normal_vec if asked for
if (nlhs > 2){
        plhs[2] = normal_vec; 
}



return;
}




