#include <iostream>
#include <fstream>
#include <array>

#include "Eigen.h"
#include "VirtualSensor.h"

struct Vertex
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// position stored as 4 floats (4th component is supposed to be 1.0)
	Vector4f position;
	// color stored as 4 unsigned char
	Vector4uc color;
};

bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{
	float edgeThreshold = 0.01f; // 1cm

	// TODO 2: use the OFF file format to save the vertices grid (http://www.geomview.org/docs/html/OFF.html)
	// - have a look at the "off_sample.off" file to see how to store the vertices and triangles
	// - for debugging we recommend to first only write out the vertices (set the number of faces to zero)
	// - for simplicity write every vertex to file, even if it is not valid (position.x() == MINF) (note that all vertices in the off file have to be valid, thus, if a point is not valid write out a dummy point like (0,0,0))
	// - use a simple triangulation exploiting the grid structure (neighboring vertices build a triangle, two triangles per grid cell)
	// - you can use an arbitrary triangulation of the cells, but make sure that the triangles are consistently oriented
	// - only write triangles with valid vertices and an edge length smaller then edgeThreshold

	// TODO: Get number of vertices
	unsigned int nVertices = 0;
	nVertices = width*height;

	// TODO: Determine number of valid faces
	unsigned nFaces = 0;
	for(int y = 0; y < height-1; y++){
		for(int x = 0; x < width-1; x++){
			
			int idx1 = y*width+x; 
			int idx2 = y*width+(x+1); 
			int idx3 = (y+1)*width+x; 
			int idx4 = (y+1)*width+(x+1); 
			Vector4f point0 = vertices[idx1].position;
			Vector4f point1 = vertices[idx2].position;
			Vector4f point5 = vertices[idx3].position;
			Vector4f point6 = vertices[idx4].position;
			if(point0(0) != MINF && point1(0) != MINF && point5(0) != MINF ){
				if((point0-point1).norm()>edgeThreshold && (point0-point5).norm()>edgeThreshold){
				nFaces++;
				}}
			if(point5(0) != MINF && point6(0) != MINF && point1(0) != MINF ){
				if((point5-point6).norm()>edgeThreshold && (point1-point6).norm()>edgeThreshold){
				nFaces++;
			}}
		}
	}

	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()) return false;

	// write header
	outFile << "COFF" << std::endl;

	outFile << "# numVertices numFaces numEdges" << std::endl;

	outFile << nVertices << " " << nFaces << " 0" << std::endl;

	// TODO: save vertices
	outFile << "# list of vertices" << std::endl;

	outFile << "# X Y Z R G B A" << std::endl;

	for(int i = 0; i < width*height; i++){
		if (vertices[i].position[0] != MINF && vertices[i].position[1] != MINF && vertices[i].position[2] != MINF){
			outFile << vertices[i].position[0] 
					<< " " << vertices[i].position[1] 
					<< " " << vertices[i].position[2] 
					<< " " << (int)vertices[i].color[0]
					<< " " << (int)vertices[i].color[1] 
					<< " " << (int)vertices[i].color[2]
					<< " " << (int)vertices[i].color[3]
					<< std::endl;
					}
		else{
			outFile << 0.0 << " " << 0.0 << " "
					<< 0.0 << " " << (int)vertices[i].color[0]
					<< " " << (int)vertices[i].color[1] 
					<< " " << (int)vertices[i].color[2]
					<< " " << (int)vertices[i].color[3]
					<< std::endl;	
		}
	}

	// TODO: save valid faces
	outFile << "# list of faces" << std::endl;

	outFile << "# nVerticesPerFace idx0 idx1 idx2 ..." << std::endl;

	for(int y = 0; y < height-1; y++){
		for(int x = 0; x < width-1; x++){
			int idx1 = y*width+x; 
			int idx2 = y*width+(x+1); 
			int idx3 = (y+1)*width+x; 
			int idx4 = (y+1)*width+(x+1); 
			Vector4f point0 = vertices[idx1].position;
			Vector4f point1 = vertices[idx2].position;
			Vector4f point5 = vertices[idx3].position;
			Vector4f point6 = vertices[idx4].position;
			if(point0(0) != MINF && point1(0) != MINF && point5(0) != MINF ){
				if((point0-point1).norm()>edgeThreshold && (point0-point5).norm()>edgeThreshold){
				outFile << idx1 << " " << idx3 << " " << idx2<< std::endl; 
				// std::cout << idx1 << " " << idx3 << " " << idx2<< std::endl; 
				}
			}
			if(point5(0) != MINF && point6(0) != MINF && point1(0) != MINF ){

				if((point5-point6).norm()>edgeThreshold && (point1-point6).norm()>edgeThreshold){
				outFile << idx3 << " " << idx4 << " " << idx2 << std::endl; 
				// std::cout << idx3 << " " << idx4 << " " << idx2 << std::endl; 
				}
			}
		}
	}

	// close file
	outFile.close();

	return true;
}

int main()
{
	// Make sure this path points to the data folder
	std::string filenameIn = "../../Data/rgbd_dataset_freiburg1_xyz/";
	std::string filenameBaseOut = "mesh_";

	// load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.Init(filenameIn))
	{
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	// convert video to meshes
	while (sensor.ProcessNextFrame())
	{
		// get ptr to the current depth frame
		// depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
		float* depthMap = sensor.GetDepth();
		// get ptr to the current color frame
		// color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
		BYTE* colorMap = sensor.GetColorRGBX();

		// get depth intrinsics
		Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
		Matrix3f depthIntrinsicsInv = depthIntrinsics.inverse();

		float fX = depthIntrinsics(0, 0);
		float fY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		// compute inverse depth extrinsics
		Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

		Matrix4f trajectory = sensor.GetTrajectory();
		Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

		// TODO 1: back-projection
		// write result to the vertices array below, keep pixel ordering!
		// if the depth value at idx is invalid (MINF) write the following values to the vertices array
		// vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
		// vertices[idx].color = Vector4uc(0,0,0,0);
		// otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap
		Vertex* vertices = new Vertex[sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight()];
		
		for(int y = 0; y < sensor.GetDepthImageHeight(); y++){
			for(int x = 0; x < sensor.GetDepthImageWidth(); x++){			
				int idx = y*sensor.GetColorImageWidth()+x; 
				if(depthMap[idx] == MINF){
					vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
					vertices[idx].color = Vector4uc(0,0,0,0);
				}
				else{
					float x_shifted = (x-cX)/fX;
					float y_shifted = (y-cY)/fY;
					float z = depthMap[idx];

					Vector3f dehomogenized(x_shifted*z, y_shifted*z, z);
					Vector3f multiplied_vectors = depthIntrinsicsInv * dehomogenized; 
					vertices[idx].position = trajectoryInv * (depthExtrinsicsInv * Vector4f(multiplied_vectors(0),multiplied_vectors(1),multiplied_vectors(2), 1.0 ));
					vertices[idx].color = Vector4uc(static_cast<float>(colorMap[idx*4]),static_cast<float>(colorMap[idx*4+1]),static_cast<float>(colorMap[idx*4+2]),static_cast<float>(colorMap[idx*4+3]));

				}
			}
		}

		// write mesh file
		std::stringstream ss;
		ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
		if (!WriteMesh(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), ss.str()))
		{
			std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
			return -1;
		}

		// free mem
		delete[] vertices;
	}

	return 0;
}