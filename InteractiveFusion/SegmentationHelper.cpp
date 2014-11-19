#include "SegmentationHelper.h"
#include "InteractiveFusion.h"
#include "PCLProcessor.h"
#include "VcgMeshContainer.h"
#include "OpenGLShaders.h"
#include "OpenGLCamera.h"
#include "OpenGLText.h"
#include "MeshHelper.h"

PCLProcessor pclProcessor;
std::vector<Vertex> startingVertices;
std::vector<Triangle> startingIndices;

glm::vec4 planeC;

std::vector<shared_ptr<VCGMeshContainer>> meshData_segTmp;

bool SegmentationHelper::IsCloudReady()
{
	return pclProcessor.coloredCloudReady;
}

bool SegmentationHelper::IsPreviewInitialized()
{
	if (previewVertices.size() > 0)
		return true;
	else
		return false;
}

void SegmentationHelper::ClearPreviewVertices()
{
	previewVertices.clear();
}

void SegmentationHelper::LoadClusterData()
{
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		(*mI)->ClearMesh();
		//delete *mI;
	}
	meshData.clear();
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData_segTmp.begin(); mI != meshData_segTmp.end(); ++mI)
	{
		//if (tmp != glSelector.selectedIndex-1)
		meshData.push_back(*mI);
		//meshData.(*mI)->Draw();
		//tmp++;
	}

	/*for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData_segTmp.begin(); mI != meshData_segTmp.end(); ++mI)
	{
	(*mI)->ClearMesh();
	}*/
	meshData_segTmp.clear();

	meshHelper.GenerateBuffers();

	//segFinished = false;
	//openGLWin.state = DEFAULT;
	pclProcessor.coloredCloudReady = false;
}

int WINAPI SegThreadMain()
{

	if (meshData_segTmp.size() > 0 && openGLWin.segmentValuesChanged)
	{
		int cnt = 0;
		for (int i = pclProcessor.GetPlaneClusterCount(); i < meshData_segTmp.size(); i++)
		{
			meshData_segTmp[i]->ClearMesh();
			//delete meshData_segTmp[i];
		}
		meshData_segTmp.erase(meshData_segTmp.begin() + pclProcessor.GetPlaneClusterCount(), meshData_segTmp.end());
		/*for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData_segTmp.begin(); mI != meshData_segTmp.end(); ++mI)
		{
		if (pclProcessor.IsPlaneSegmented())
		if (cnt < pclProcessor.GetPlaneClusterCount())
		continue;
		(*mI)->ClearMesh();
		cnt++;
		}
		meshData_segTmp.clear();*/
	}
	//UNCOMMENT FRO HERE FOR SEGMENTATION

	LARGE_INTEGER frequency;        // ticks per second
	LARGE_INTEGER t1, t2;           // ticks
	double elapsedTime;
	if (meshData.size() == 1)
	{
		if (!pclProcessor.IsMainCloudInitialized())
			meshData[0]->CleanAndParse(startingVertices, startingIndices);
	}

	else
	{
		openGLWin.ShowStatusBarMessage(L"Can't segment mesh, as it is already segmented.");
		return 0;
		switch (openGLWin.testMode)
		{
		case 0:
			//CombineAndExport();
			meshHelper.CleanAndParse("data\\models\\output.ply", startingVertices, startingIndices);
			break;
		case 1:
			meshHelper.CleanAndParse("data\\models\\testScene.ply", startingVertices, startingIndices);
			break;
		case 2:
			meshHelper.CleanAndParse("data\\models\\cube.ply", startingVertices, startingIndices);
			break;
		}
	}
	//CleanAndParse("data\\models\\output.ply", startingVertices, startingIndices, startingNormals);
	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&t1);
	QueryPerformanceCounter(&t2);
	if (!pclProcessor.IsMainCloudInitialized())
	{
		statusMsg = L"Converting mesh to point cloud";
		pclProcessor.ConvertToCloud(startingVertices, startingIndices);
	}
	QueryPerformanceCounter(&t2);
	elapsedTime = (t2.QuadPart - t1.QuadPart) * 1000.0 / frequency.QuadPart;
	cDebug::DbgOut(L"Converted to cloud in ", elapsedTime);

	//pclProcessor.PlaneSegmentation();
	//pclProcessor.EuclideanSegmentation();
	cDebug::DbgOut(L"before plane segmentation ", elapsedTime);


	if (!pclProcessor.IsPlaneSegmented())
	{
		pclProcessor.PlaneSegmentation();
		pclProcessor.PlaneIndexEstimation();
		for (int i = 0; i < pclProcessor.GetPlaneClusterCount(); i++)
		{
			std::vector<Vertex> clusterVertices;
			std::vector<Triangle> clusterIndices;
			pclProcessor.ConvertToTriangleMesh(i, startingVertices, clusterVertices, clusterIndices);
			cDebug::DbgOut(L"Converted Plane Mesh #", i);
			shared_ptr<VCGMeshContainer> mesh(new VCGMeshContainer);
			mesh->SetColorCode(i + 1);
			//mesh->ParseData(clusterVertices, clusterIndices);
			mesh->SetWall(true);
			mesh->ConvertToVCG(clusterVertices, clusterIndices);
			float threshold = 0.005f;
			int total = mesh->MergeCloseVertices(threshold);
			cDebug::DbgOut(_T("Merged close vertices: "), total);
			//int total = mesh->MergeCloseVertices(0.005f);
			//cDebug::DbgOut(_T("Merged close vertices: "), total);
			//cDebug::DbgOut(L"indices: ", (int)mesh->GetVertices().size());
			//cDebug::DbgOut(L"vertices: " + (int)mesh->GetIndices().size());

			cDebug::DbgOut(_T("Clean PlaneSeg #1"));
			mesh->CleanMesh();
			mesh->RemoveSmallComponents(clusterVertices.size() / 5);
			//cDebug::DbgOut(_T("Clean PlaneSeg #2"));
			//mesh->CleanMesh();
			mesh->RemoveNonManifoldFace();
			mesh->FillHoles(10000);
			//cDebug::DbgOut(_T("Clean PlaneSeg #3"));
			//mesh->CleanMesh();
			mesh->RemoveSmallComponents(clusterVertices.size() / 2);
			//mesh->CleanMesh();
			mesh->ParseData();
			mesh->SetPlaneParameters(pclProcessor.planeCoefficients[i]->values[0], pclProcessor.planeCoefficients[i]->values[1],
				pclProcessor.planeCoefficients[i]->values[2], pclProcessor.planeCoefficients[i]->values[3]);
			planeC = glm::vec4(pclProcessor.planeCoefficients[i]->values[0], pclProcessor.planeCoefficients[i]->values[1], pclProcessor.planeCoefficients[i]->values[2], pclProcessor.planeCoefficients[i]->values[3]);
			//cDebug::DbgOut(L"vertices: ", (int)mesh->GetNumberOfVertices());
			//cDebug::DbgOut(L"indices: " + (int)mesh->GetNumberOfTriangles());
			meshData_segTmp.push_back(mesh);
		}
	}

	if (openGLWin.segmentationMode == REGION_GROWTH_SEGMENTATION)
	{
		if (openGLWin.GetWindowState() != SEGMENTATION_PREVIEW || openGLWin.previewMode || openGLWin.segmentValuesChanged)
		{

			//meshData_segTmp[0]->CleanAndParse(startingVertices, startingIndices, startingNormals);
			//pclProcessor.ConvertToCloud(startingVertices, startingIndices, startingNormals);
			pclProcessor.RegionGrowingSegmentation();

			openGLWin.segmentValuesChanged = false;
		}
		if (openGLWin.previewMode)
		{
			openGLWin.SetWindowState(SEGMENTATION_PREVIEW);
			cDebug::DbgOut(L"Number of clusters: ", pclProcessor.GetClusterCount());
			return 0;
		}

	}
	if (openGLWin.segmentationMode == EUCLIDEAN_SEGMENTATION)
	{
		pclProcessor.EuclideanSegmentation();
	}

	pclProcessor.IndexEstimation();
	cDebug::DbgOut(L"Clustered");


	//elapsedTime = (t2.QuadPart - t1.QuadPart) * 1000.0 / frequency.QuadPart;
	//cDebug::DbgOut(L"Time for map creation ", elapsedTime);
	cDebug::DbgOut(L"Number of clusters: ", pclProcessor.GetRegionClusterCount());
	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&t1);
	QueryPerformanceCounter(&t2);
	for (int i = 0; i < pclProcessor.GetRegionClusterCount(); i++)
	{
		std::vector<Vertex> clusterVertices;
		std::vector<Triangle> clusterIndices;
		if (!pclProcessor.ConvertToTriangleMesh(i, startingVertices, clusterVertices, clusterIndices))
			continue;
		//cDebug::DbgOut(L"Converted Triangle Mesh #",i);
		shared_ptr<VCGMeshContainer> mesh(new VCGMeshContainer);
		mesh->SetColorCode(i + pclProcessor.GetPlaneClusterCount() + 1);
		//mesh->ParseData(clusterVertices, clusterIndices);
		mesh->ConvertToVCG(clusterVertices, clusterIndices);
		cDebug::DbgOut(_T("Clean Seg #1"));
		mesh->CleanMesh();
		mesh->RemoveNonManifoldFace();

		//mesh->FillHoles((clusterVertices.size()) / 10);
		//mesh->CleanMesh();
		mesh->ParseData();
		//glm::vec3 cP = mesh->GetCenterPoint();
		//float iO = cP.x*planeC.x + cP.y*planeC.y + cP.z * planeC.z + planeC.w;
		//cDebug::DbgOut(L"mesh #" + to_wstring(i) + L" iO: ", iO);
		//if (iO >= 0.0f)
			meshData_segTmp.push_back(mesh);
		
	}
	QueryPerformanceCounter(&t2);
	elapsedTime = (t2.QuadPart - t1.QuadPart) * 1000.0 / frequency.QuadPart;
	cDebug::DbgOut(L"Filled MeshData in ", elapsedTime);
	//UNCOMMENT UNTIL HERE FOR SEGMENTATION
	openGLWin.SetWindowState(SEGMENTATION_FINISHED);
	//showColoredSegments = false;
	//segFinished = true;
	openGLWin.ShowStatusBarMessage(L"Segmented mesh in " + to_wstring(pclProcessor.GetRegionClusterCount()) + L"clusters.");

	return 0;

}

void SegmentationHelper::StartSegmentation()
{
	//segmenting = true;
	openGLWin.SetWindowState(SEGMENTATION);
	segmentationThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)&SegThreadMain, 0, 0, &sThreadId);
}

void SegmentationHelper::GeneratePreviewBuffers()
{
	glGenBuffers(1, &segmentVBO);
	glGenVertexArrays(1, &segmentVAO);
	glBindVertexArray(segmentVAO);
	glBindBuffer(GL_ARRAY_BUFFER, segmentVBO);
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(0));
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(sizeof(float)* 3));
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(sizeof(float)* 6));

	cDebug::DbgOut(L"seg mesh:");
	cDebug::DbgOut(L"vbo: ", (int)segmentVBO);
	cDebug::DbgOut(L"vao: ", (int)segmentVAO);
}

bool SegmentationHelper::InitializePreview()
{
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloud;
	if (openGLWin.GetWindowState() == WALL_SELECTION)
	{
		pointCloud = pclProcessor.wallSegmentCloud;
	}
	else if (openGLWin.GetWindowState() == SEGMENTATION_PREVIEW)
	{
		pointCloud = pclProcessor.coloredSegmentedCloud;
	}
	else return false;

	previewVertices.clear();
	for (int i = 0; i < pointCloud->points.size(); i++)
	{
		Vertex vertex;
		vertex.x = pointCloud->points[i].x;
		vertex.y = pointCloud->points[i].y;
		vertex.z = pointCloud->points[i].z;
		vertex.r = pointCloud->points[i].r / 255.0f;
		vertex.g = pointCloud->points[i].g / 255.0f;
		vertex.b = pointCloud->points[i].b / 255.0f;
		vertex.normal_x = 0;
		vertex.normal_y = 0;
		vertex.normal_z = 0;
		previewVertices.push_back(vertex);
	}

	if (segmentVBO == 0)
		GeneratePreviewBuffers();

	glBindBuffer(GL_ARRAY_BUFFER, segmentVBO);
	glBufferData(GL_ARRAY_BUFFER, previewVertices.size() * sizeof(Vertex), &previewVertices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	if (openGLWin.GetWindowState() == WALL_SELECTION)
		openGLWin.ShowStatusBarMessage(L"Showing wall preview.");
	else if (openGLWin.GetWindowState() == SEGMENTATION_PREVIEW)
	{
		openGLWin.ShowStatusBarMessage(L"Showing region growth segmentation preview with " + to_wstring(pclProcessor.GetClusterCount()) + L"clusters");
		pclProcessor.coloredCloudReady = false;
	}

}

void SegmentationHelper::RenderPreview()
{
	openGLWin.SetBackgroundColor(0, 0, 0);

	int w = openGLWin.glControl.GetViewportWidth();
	int h = openGLWin.glControl.GetViewportHeight();
	shaderColor.UseProgram();
	shaderColor.SetUniform("matrices.projectionMatrix", openGLWin.glControl.GetProjectionMatrix());
	shaderColor.SetUniform("matrices.viewMatrix", glCamera.GetViewMatrix());
	glm::mat4 modelMatrix = glm::mat4(1.0);

	shaderColor.SetUniform("matrices.modelMatrix", modelMatrix);

	glBindVertexArray(segmentVAO);
	glPointSize(1.5f);
	glDrawArrays(GL_POINTS, 0, previewVertices.size());
	glBindVertexArray(0);

	glUseProgram(0);

	if (openGLWin.GetWindowState() == SEGMENTATION_PREVIEW)
	{
		glText.PrepareForRender();
		glText.RenderText(L"Clusters: ", pclProcessor.GetClusterCount(), 20, -0.98f, 0.85f, 2.0f / w, 2.0f / h);
	}

	glEnable(GL_DEPTH_TEST);
}

void SegmentationHelper::CleanUp()
{
	glDeleteBuffers(1, &segmentVBO);
	glDeleteVertexArrays(1, &segmentVAO);
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData_segTmp.begin(); mI != meshData_segTmp.end(); ++mI)
	{
		(*mI)->ClearMesh();
	}
	meshData_segTmp.clear();
}