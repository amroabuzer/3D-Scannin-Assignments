#pragma once
#include "SimpleMesh.h"

class ProcrustesAligner {
public:
	Matrix4f estimatePose(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints) {
		ASSERT(sourcePoints.size() == targetPoints.size() && "The number of source and target points should be the same, since every source point is matched with corresponding target point.");

		// We estimate the pose between source and target points using Procrustes algorithm.
		// Our shapes have the same scale, therefore we don't estimate scale. We estimated rotation and translation
		// from source points to target points.

		auto sourceMean = computeMean(sourcePoints);
		auto targetMean = computeMean(targetPoints);
		
		Matrix3f rotation = estimateRotation(sourcePoints, sourceMean, targetPoints, targetMean);
		Vector3f translation = computeTranslation(sourceMean, targetMean, rotation);

		// TODO: Compute the transformation matrix by using the computed rotation and translation.
		// You can access parts of the matrix with .block(start_row, start_col, num_rows, num_cols) = elements
		
		Matrix4f estimatedPose = Matrix4f::Identity();
		return estimatedPose;
	}

private:
	Vector3f computeMean(const std::vector<Vector3f>& points) {
		// TODO: Compute the mean of input points.
		// Hint: You can use the .size() method to get the length of a vector.
		
		Vector3f mean = Vector3f::Zero();
		for(int i = 0; i < points.size(); i++){
			mean += points[i];
		}

		return (mean/points.size());
	}

	Matrix3f estimateRotation(const std::vector<Vector3f>& sourcePoints, const Vector3f& sourceMean, const std::vector<Vector3f>& targetPoints, const Vector3f& targetMean) {
		// TODO: Estimate the rotation from source to target points, following the Procrustes algorithm.
		// To compute the singular value decomposition you can use JacobiSVD() from Eigen.
		// Hint: You can initialize an Eigen matrix with "MatrixXf m(num_rows,num_cols);" and access/modify parts of it using the .block() method (see above).
		
		MatrixXf sourceMatrix(sourcePoints.size(), sourcePoints[0].size());
		MatrixXf targetMatrix(targetPoints.size(), targetPoints[0].size());
	
		for(int i = 0; i < sourcePoints.size(); i++){
			sourceMatrix.block(i,0,1,sourcePoints[i].size()) = sourcePoints[i];
			targetMatrix.block(i,0,1,targetPoints[i].size()) = targetPoints[i];			
		}
		
		Matrix3f XTX = targetMatrix.transpose() * sourceMatrix; 
		
		JacobiSVD<MatrixXf, ComputeFullU | ComputeFullV> svd(XTX);

		Matrix3f rotation = svd.matrixU() * svd.matrixV().transpose();

		if (rotation.determinant() == -1){
			Matrix3f antiMirror;
			antiMirror << 1, 0, 0,
						  0, 1, 0,
						  0, 0, -1;

			rotation = svd.matrixU() * antiMirror * svd.matrixV().transpose();
		}

        return rotation;
	}

	Vector3f computeTranslation(const Vector3f& sourceMean, const Vector3f& targetMean, const Matrix3f& rotation) {
		// TODO: Compute the translation vector from source to target points.

		Vector3f translation = Vector3f::Zero();
		translation = - rotation * sourceMean + targetMean;
        return translation;
	}
};
