#include "IK.h"
#include "FK.h"
#include "minivectorTemplate.h"
#include <Eigen/Dense>
#include <adolc/adolc.h>
#include <cassert>
#if defined(_WIN32) || defined(WIN32)
  #ifndef _USE_MATH_DEFINES
    #define _USE_MATH_DEFINES
  #endif
#endif
#include <math.h>
using namespace std;

// CSCI 520 Computer Animation and Simulation
// Jernej Barbic and Yijing Li

namespace
{

// Converts degrees to radians.
template<typename real>
inline real deg2rad(real deg) { return deg * M_PI / 180.0; }

template<typename real>
Mat3<real> Euler2Rotation(const real angle[3], RotateOrder order)
{
  Mat3<real> RX = Mat3<real>::getElementRotationMatrix(0, deg2rad(angle[0]));
  Mat3<real> RY = Mat3<real>::getElementRotationMatrix(1, deg2rad(angle[1]));
  Mat3<real> RZ = Mat3<real>::getElementRotationMatrix(2, deg2rad(angle[2]));

  switch(order)
  {
    case RotateOrder::XYZ:
      return RZ * RY * RX;
    case RotateOrder::YZX:
      return RX * RZ * RY;
    case RotateOrder::ZXY:
      return RY * RX * RZ;
    case RotateOrder::XZY:
      return RY * RZ * RX;
    case RotateOrder::YXZ:
      return RZ * RX * RY;
    case RotateOrder::ZYX:
      return RX * RY * RZ;
  }
  assert(0);
}

// Performs forward kinematics, using the provided "fk" class.
// This is the function whose Jacobian matrix will be computed using adolc.
// numIKJoints and IKJointIDs specify which joints serve as handles for IK:
//   IKJointIDs is an array of integers of length "numIKJoints"
// Input: numIKJoints, IKJointIDs, fk, eulerAngles (of all joints)
// Output: handlePositions (world-coordinate positions of all the IK joints; length is 3 * numIKJoints)
template<typename real>
void forwardKinematicsFunction(
    int numIKJoints, const int * IKJointIDs, const FK & fk,
    const std::vector<real> & eulerAngles, std::vector<real> & handlePositions)
{
  // Students should implement this.
  // The implementation of this function is very similar to function computeLocalAndGlobalTransforms in the FK class.
  // The recommended approach is to first implement FK::computeLocalAndGlobalTransforms.
  // Then, implement the same algorithm into this function. To do so,
  // you can use fk.getJointUpdateOrder(), fk.getJointRestTranslation(), and fk.getJointRotateOrder() functions.
    vector<Mat3<real>> localTransforms;
    vector<Vec3<real>> localTraslations;
    for (int i = 0; i < fk.getNumJoints(); i++) {
        // implement local transformation
        real angles[3]; Mat3<real> jointR;
        for (int j = 0; j < 3; j++) angles[j] = eulerAngles[3 * i + j];
        jointR = Euler2Rotation(angles, fk.getJointRotateOrder(i));      

        real orienAngles[3];Mat3<real> orientationR;
        for (int j = 0; j < 3; j++) orienAngles[j] = fk.getJointOrient(i)[j];
        orientationR = Euler2Rotation(orienAngles, fk.getJointRotateOrder(i));

        Mat3<real> transformedR = orientationR * jointR;
        localTransforms.push_back(transformedR);

        // implement local translation
        Vec3<real> currTranslation(3);
        for (int j = 0; j < 3; j++) currTranslation[j] = fk.getJointRestTranslation(i)[j];
        localTraslations.push_back(currTranslation);
    }
    vector<Mat3<real>> globalTransforms(fk.getNumJoints());
    vector<Vec3<real>> globalTranslations(fk.getNumJoints());
    for (int i = 0; i < fk.getNumJoints(); i++) {
        int curr = fk.getJointUpdateOrder(i);
        if (fk.getJointParent(curr) == -1) { // root joint
            globalTransforms[curr] = localTransforms[curr];
            globalTranslations[curr] = localTraslations[curr];
        }
        else { // child joint
            int parentIndx = fk.getJointParent(curr);
            Mat3<real> tempTransform = globalTransforms[parentIndx] * localTransforms[curr];
            Vec3<real> tempTranslation = globalTransforms[parentIndx] * localTraslations[curr] + globalTranslations[parentIndx];
            globalTransforms[curr] = tempTransform;
            globalTranslations[curr] = tempTranslation;
        }
    }
    for (int i = 0; i < numIKJoints; i++) {
        handlePositions[3 * i] = globalTranslations[IKJointIDs[i]][0];
        handlePositions[3 * i + 1] = globalTranslations[IKJointIDs[i]][1];
        handlePositions[3 * i + 2] = globalTranslations[IKJointIDs[i]][2];
    }
}

} // end anonymous namespaces

IK::IK(int numIKJoints, const int * IKJointIDs, FK * inputFK, int adolc_tagID)
{
  this->numIKJoints = numIKJoints;
  this->IKJointIDs = IKJointIDs;
  this->fk = inputFK;
  this->adolc_tagID = adolc_tagID;

  FKInputDim = fk->getNumJoints() * 3;
  FKOutputDim = numIKJoints * 3;

  train_adolc();
}

void IK::train_adolc()
{
  // Students should implement this.
  // Here, you should setup adol_c:
  //   Define adol_c inputs and outputs. 
  //   Use the "forwardKinematicsFunction" as the function that will be computed by adol_c.
  //   This will later make it possible for you to compute the gradient of this function in IK::doIK
  //   (in other words, compute the "Jacobian matrix" J).
  // See ADOLCExample.cpp .
    int n = FKInputDim; // input dimension is n
    int m = FKOutputDim; // output dimension is m

    // first, call trace_on to ask ADOL-C to begin recording how function f is implemented
    trace_on(adolc_tagID); // start tracking computation with ADOL-C

    vector<adouble> eulerAngles(n); // define the input of the function f
    for (int i = 0; i < n; i++)
        eulerAngles[i] <<= 0.0; // The <<= syntax tells ADOL-C that these are the input variables.

    vector<adouble> handlePositions(m); // define the output of the function f
    
    // Use the "forwardKinematicsFunction" as the function that will be computed by adol_c.
    forwardKinematicsFunction(numIKJoints, IKJointIDs, *fk, eulerAngles, handlePositions);

    vector<double> output(m);
    for (int i = 0; i < m; i++)
        handlePositions[i] >>= output[i]; // Use >>= to tell ADOL-C that y[i] are the output variables

    // Finally, call trace_off to stop recording the function f.
    trace_off(); // ADOL-C tracking finished
}

void IK::doIK(const Vec3d * targetHandlePositions, Vec3d * jointEulerAngles, bool flag)
{
  // Use adolc to evalute the forwardKinematicsFunction and its gradient (Jacobian). It was trained in train_adolc().
  // Specifically, use ::function, and ::jacobian .
  //
  // Use it implement the Tikhonov IK method (or the pseudoinverse method for extra credit).
  // Note that at entry, "jointEulerAngles" contains the input Euler angles. 
  // Upon exit, jointEulerAngles should contain the new Euler angles.
    int numJoints = fk->getNumJoints(); // Note that is NOT the same as numIKJoints!
    int n = FKInputDim; // input dimension is n
    int m = FKOutputDim; // output dimension is m
    double* output_y_values = new double[m];
    std::fill(output_y_values, output_y_values + m, 0.0);
    ::function(adolc_tagID, m, n, jointEulerAngles->data(), output_y_values);

    //double jacobianMatrix[2 * 3]; // We store the matrix in row-major order.
    //double* jacobianMatrixEachRow[] = { &jacobianMatrix[0], &jacobianMatrix[3] }; // pointer array where each pointer points to one row of the jacobian matrix
    double* jacobianMatrix = new double[n*m];
    double** jacobianMatrixEachRow = new double*[m];
    for (int i = 0; i < m; i++) jacobianMatrixEachRow[i] = &jacobianMatrix[i * n];
    ::jacobian(adolc_tagID, m, n, jointEulerAngles->data(), jacobianMatrixEachRow); // each row is the gradient of one output component of the function

    // matrix J has m rows and n columns
    Eigen::MatrixXd J(m, n);
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            J(i, j) = jacobianMatrix[i * n + j];
        }
    }
    // J^(T)
    Eigen::MatrixXd transposeJ = J.transpose();
    // I is a nxn identity matrix
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
    // ¦¤b is a mx1 vector representing the change of handle global positions
    Eigen::VectorXd deltaB(m);
    for (int i = 0; i < numIKJoints; i++) {
        for (int j = 0; j < 3; j++) {
            deltaB[3 * i + j] = targetHandlePositions[i][j] - output_y_values[3 * i + j];
        }
    }
    Eigen::VectorXd deltaTheta;
    if (flag) {
        /* Tikhonov regularization */
        // ¦¤¦È is a nx1 vector representing the change of Euler angles
        deltaTheta = (transposeJ * J + (0.01) * I).ldlt().solve(transposeJ * deltaB);        
    }
    else{
        /* pseudoinverse IK method => ¦¤¦È = J^T(J*J^T)^-1 * deltaB */
        deltaTheta = transposeJ * (J * transposeJ).inverse() * deltaB;
    }
    for (int i = 0; i < numJoints; i++) {
        for (int j = 0; j < 3; j++) {
            jointEulerAngles[i][j] += deltaTheta[3 * i + j];
        }
    }
}

