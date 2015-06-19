/* Computes the distance from the origin to a polytope (given as a list of vertices)*/

#include <iostream>
#include "../DrakeCollision.h"
#include "../Model.h"

// #include <math.h>
// #include <matrix.h>
#include <mex.h>

using namespace std;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{  
  // mexPrintf("Hello World!\n");
  
  /*
   * Basics:
   * - The interface to the DrakeCollision library is a collision model
   * - Collision models contain multiple collision bodies
   * - Collision bodies contain multiple collision elements
   * - Each collision element has a shape and a transformation relative to its
   *   body
   * - Collision bodies are really just groups of collision elements whose
   *   relative poses are fixed - there's no kinematics being done in the
   *   DrakeCollision library.
   */

  /* Check for proper number of arguments.*/
  if(nrhs!=1) {
    mexErrMsgIdAndTxt( "MATLAB:ptToPolyDist:invalidNumInputs",
            "One input required.");
  } else if(nlhs>1) {
    mexErrMsgIdAndTxt( "MATLAB:ptToPolyDist:maxlhs",
            "Too many output arguments.");
  }

  // Create a collision model
  shared_ptr<DrakeCollision::Model> model = DrakeCollision::newModel();

  // Location of point
  Vector3d point(0.0,0.0,0.0); // Always the origin

  // cout << "noRBMTest: Point" << endl;
  // cout << point << endl;

  // Treat the point as a collision element with a SPHERE shape. The only
  // parameter for the SPHERE shape is the radius of the sphere.
  vector<double> point_params; point_params.push_back(0.0);

  // The point will belong to body 0 and will be at the coordinates specified
  // by `point`. To add the collision element to the model, we need to define
  // the transform from the element to its containing body. Since we don't
  // care about a kinematic tree in this case we'll set the parent body for
  // this element to be body 0 as well.
  int point_body_idx = 0;
  Matrix4d point_transform;
  point_transform << MatrixXd::Identity(3,3), point, MatrixXd::Zero(1,3),1;

  model->addElement(point_body_idx,0,point_transform,DrakeCollision::Shape::SPHERE, point_params, false);

  // Check inputs
  double *vertsPr;
  vertsPr = mxGetPr(prhs[0]);
  const int nRows = mxGetM(prhs[0]); // number of rows
  const int nCols = mxGetN(prhs[0]); // number of columns

  // Check to see if number of rows of vertices is 3
  if(nRows != 3){
    mexErrMsgIdAndTxt( "MATLAB:ptToPolyDist:invalidInputs",
            "Input should be 3 x N");
  }

  // Location of the points defining the polytope
  MatrixXd polytope(nRows,nCols);

  // Assign elements of polytopes to verts
  for(int i=0;i<nRows;i++){
	for(int j=0;j<nCols;j++){
	   polytope(i, j) = vertsPr[j*nRows + i];

	}
  }


  // cout << "noRBMTest: Polytope points" << endl;
  // cout << polytope << endl;


  // Treat the polytope as a MESH element. The parameters for this shape are
  // the coordinates of the points relative to the body frame.
  vector<double> polytope_params(polytope.data(), 
                                 polytope.data() + polytope.rows()*polytope.cols());

  // The polytope will belong to body 1. Note that since the shape parameters
  // for a MESH shape are the coordinates of the points relative to the
  // element's frame, we can set the element frame equal to the body frame
  // here. We do this by passing in an identity transform. If you want to the
  // whole polytope after creation, you can use
  //
  // Matrix4d transform;
  // <set the new body-world transform here>
  // model->updateElementsForBody(polytope_body_idx, transform);
  //
  int polytope_body_idx = 1;
  model->addElement(polytope_body_idx,0,Matrix4d::Identity(),
                    DrakeCollision::Shape::MESH, polytope_params, false);

  // Create output variables
  Vector3d ptA;
  Vector3d ptB;
  Vector3d normal;
  double distance;

  // Now we ask the collision model for the distance between the body
  // containing the point and the body containing the polytope. We also get
  // back the closest points on each body and the surface normal.
  //
  // NOTE: The version of DrakeCollision currently in the drake trunk tells
  //       Bullet to use a collision margin of 0.01 units for MESH shaped
  //       collision elements. That means that the distance reported is 0.01
  //       units too small when considering point to polytope distances and
  //       0.02 units too small when considering polytope to polytope
  //       distances. You could almost certainly change this behavior without
  //       adverse consequences by searching for a call to `setMargin` in
  //       `BulletElement.cpp` and changing the input argument from `0.01` to
  //       `0.0`.
  bool success = model->getClosestPoints(point_body_idx, polytope_body_idx, 
                                         ptA, ptB, normal, distance);

  // cout << "noRBMTest: Closest point on polytope" << endl;
  // cout << ptB << endl;
  // cout << "noRBMTest: Closest point on point " << endl;
  // cout << ptA << endl;
  // cout << "noRBMTest: Normal between point and polytope: " << endl;
  // cout << normal << endl;
  // cout << "noRBMTest: Distance between point and polytope: " << endl;
  // cout << distance << endl;

  // Output distance plus margin (see comment above)
  plhs[0] = mxCreateDoubleScalar(distance+0.01);


  // return 0;
}
