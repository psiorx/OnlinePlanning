/* Finds a collision free funnel from a funnel library, given a decription of obstacle positions.

Inputs:
x: current state
forest: cell array containing vertices of obstacles
funnelLibrary: struct containing funnel library
	funnelLibrary(*).xyz: xyz positions at time points (double 3 X N)
	funnelLibrary(*).cS: cholesky factorization of projection of S matrix (cell array)

Output: First collision free funnel found

Author: Anirudha Majumdar
Date: Nov 9 2013
*/


// Mex stuff
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
#include <iostream>

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
We could probably make the radius 0 and be ok, but I'm not sure if bullet expects things to be non-degenrate.
*/
static const double g_radius = 1.0; // This should be 1.0.
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
	
    return gjkOutput.m_distance + CONVEX_DISTANCE_MARGIN;

}


/* Main mex funtion*/
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{

// Get x (state) at which sphere is centered
// const mxArray *x;
// x = prhs[0];


// Deal with forest cell (second input)
const mxArray *forest; // Cell array containing forest (pointer)

forest = prhs[0]; // Get forest cell (second input)
mwSize numObs = mxGetNumberOfElements(forest); // Number of obstacles

// Initialize some variables
double *verts; // cell element (i.e. vertices)
mxArray *obstacle;
size_t nCols;
size_t nRows;


// Get first obstacle in forest
int jForest = 0;
// Get vertices of this obstacle
obstacle = mxGetCell(forest, jForest);
verts = mxGetPr(mxGetCell(forest, jForest)); // Get vertices
nCols = mxGetN(obstacle);
nRows = mxGetM(obstacle);


// Call ptToPolyBullet
double min_dist = 0;
mxArray *normal_vec;
normal_vec = mxCreateDoubleMatrix(3,1,mxREAL);
min_dist = ptToPolyBullet(verts, nRows, nCols, normal_vec);


// Return translation that will make things collision free
plhs[0] = mxCreateDoubleScalar(min_dist);

// Return normal vector if asked for
if (nlhs > 1){
        plhs[1] = normal_vec;
}


return;
}
















