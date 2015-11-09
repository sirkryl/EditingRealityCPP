#include "SegmentationHelper.h"
#include "StateManager.h"
#include "GraphicsController.h"
#include "Parameters.h"
#include "PointCloudProcessor.h"
#include "MeshContainer.h"
#include "InteractionWindow.h"
#include "MainWindow.h"
#include "OpenGLShaderProgram.h"
#include "GraphicsCamera.h"
#include "OpenGLText.h"
#include "MeshHelper.h"
#include "SelectionHelper.h"
#include <random>

namespace InteractiveFusion {
	bool isPreviewInitialized = false;
	int currentPlaneIndex = -1;
	PointCloudProcessor pclProcessor;
	std::vector<Vertex> startingVertices;
	std::vector<Triangle> startingIndices;

	shared_ptr<MeshContainer> previewMesh(new MeshContainer);
	glm::vec4 planeC;

	std::vector<shared_ptr<MeshContainer>> meshData_segTmp;

	bool SegmentationHelper::IsCloudReady()
	{
		if (GetSegmentationState() == IF_SEGSTATE_OBJECT_SEGMENTATION)
			return pclProcessor.GetRegionClusterCount() > 0;
		else if (GetSegmentationState() == IF_SEGSTATE_PLANE_SEGMENTATION)
			return pclProcessor.GetInlierIndices().size() > 0;
		return false;
	}

	bool SegmentationHelper::IsPreviewInitialized()
	{
		return isPreviewInitialized;
	}

	void SegmentationHelper::ResetInitializedStatus()
	{
		isPreviewInitialized = false;
	}

	void SegmentationHelper::ClearPreviewVertices()
	{
		previewVertices.clear();
		isPreviewInitialized = false;
	}

	void SegmentationHelper::LoadClusterData()
	{
		for (vector <shared_ptr<MeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
		{
			(*mI)->CleanUp();
			//delete *mI;
		}
		meshData.clear();
		for (vector <shared_ptr<MeshContainer>>::iterator mI = meshData_segTmp.begin(); mI != meshData_segTmp.end(); ++mI)
		{
			//if (tmp != glSelector.selectedIndex-1)
			meshData.push_back(*mI);
			//meshData.(*mI)->Draw();
			//tmp++;
		}

		/*for (vector <shared_ptr<MeshContainer>>::iterator mI = meshData_segTmp.begin(); mI != meshData_segTmp.end(); ++mI)
		{
		(*mI)->CleanUp();
		}*/
		meshData_segTmp.clear();

		meshHelper.GenerateBuffers();

		//segFinished = false;
		//openGLWin.state = DEFAULT;
		pclProcessor.coloredCloudReady = false;
	}

	int WINAPI SegThreadMain()
	{

		if (meshData_segTmp.size() > 0 && glSegmentation.segmentValuesChanged)
		{
			int cnt = 0;
			for (int i = pclProcessor.GetPlaneClusterCount(); i < meshData_segTmp.size(); i++)
			{
				meshData_segTmp[i]->CleanUp();
			}
			meshData_segTmp.erase(meshData_segTmp.begin() + pclProcessor.GetPlaneClusterCount(), meshData_segTmp.end());
		}
		//UNCOMMENT FRO HERE FOR SEGMENTATION

		LARGE_INTEGER frequency;        // ticks per second
		LARGE_INTEGER t1, t2;           // ticks
		double elapsedTime;
		if (!pclProcessor.IsMainCloudInitialized())
			originalMesh->CleanAndParse(startingVertices, startingIndices);

		QueryPerformanceFrequency(&frequency);
		QueryPerformanceCounter(&t1);
		QueryPerformanceCounter(&t2);
		if (!pclProcessor.IsMainCloudInitialized())
		{
			appStatus.SetViewportStatusMessage(L"Converting mesh to point cloud");
			pclProcessor.ConvertToCloud(startingVertices, startingIndices);
		}
		QueryPerformanceCounter(&t2);
		elapsedTime = (t2.QuadPart - t1.QuadPart) * 1000.0 / frequency.QuadPart;
		DebugUtility::DbgOut(L"Converted to cloud in ", elapsedTime);

		DebugUtility::DbgOut(L"before plane segmentation ", elapsedTime);

		if (!pclProcessor.IsPlaneSegmented())
		{
			appStatus.SetViewportStatusMessage(L"Trying to find planes");
			glSegmentation.SetSegmentationState(IF_SEGSTATE_PLANE_SEGMENTATION);
			pclProcessor.PlaneSegmentation();
			pclProcessor.PlaneIndexEstimation();
			for (int i = 0; i < pclProcessor.GetPlaneClusterCount(); i++)
			{
				std::vector<Vertex> clusterVertices;
				std::vector<Triangle> clusterIndices;
				pclProcessor.ConvertToTriangleMesh(i, startingVertices, clusterVertices, clusterIndices, true);

				//pclProcessor.ConfirmPlaneIndices(i);
				DebugUtility::DbgOut(L"Converted Plane Mesh #", i);
				shared_ptr<MeshContainer> mesh(new MeshContainer);
				mesh->SetColorCode(i + 1);
				mesh->SetWall(true);
				mesh->CopyToInternalData(clusterVertices, clusterIndices);
				float threshold = 0.005f;
				int total = mesh->MergeCloseVertices(threshold);
				DebugUtility::DbgOut(_T("Merged close vertices: "), total);

				DebugUtility::DbgOut(_T("Clean PlaneSeg #1"));
				mesh->CleanMesh();
				mesh->RemoveSmallComponents(clusterVertices.size() / 5);

				mesh->RemoveNonManifoldFace();
				//mesh->FillHoles(100000);
				total = mesh->MergeCloseVertices(threshold);
				DebugUtility::DbgOut(_T("Merged close vertices 22: "), total);
				mesh->CleanMesh();

				//mesh->RemoveSmallComponents(clusterVertices.size() / 3);
				//mesh->CleanMesh();
				mesh->ParseData();
				mesh->SetPlaneParameters(pclProcessor.planeCoefficients[i]->values[0], pclProcessor.planeCoefficients[i]->values[1],
					pclProcessor.planeCoefficients[i]->values[2], pclProcessor.planeCoefficients[i]->values[3]);
				planeC = glm::vec4(pclProcessor.planeCoefficients[i]->values[0], pclProcessor.planeCoefficients[i]->values[1], pclProcessor.planeCoefficients[i]->values[2], pclProcessor.planeCoefficients[i]->values[3]);

				if (i == 0)
				{
					glm::vec3 normalVector = glm::normalize(glm::vec3(pclProcessor.planeCoefficients[i]->values[0], pclProcessor.planeCoefficients[i]->values[1], pclProcessor.planeCoefficients[i]->values[2]));

					glm::vec3 yAxis = glm::normalize(glm::vec3(0.0f, 1.0f, 0.0f));

					if (normalVector.y < 0)
						yAxis.y = -1.0f;

					glm::vec3 rotationAxis = glm::cross(normalVector, yAxis);

					//float xRotation = glm::acos(glm::dot(normalVector, glm::vec3(1.0f, 0.0f, 0.0f)));
					float rotY = glm::acos(glm::dot(normalVector, yAxis)) * 180.0f / M_PI;
					//float zRotation = glm::acos(glm::dot(normalVector, glm::vec3(0.0f, 0.0f, 1.0f)));

					meshHelper.SetGroundAlignmentRotation(glm::rotate(glm::mat4(1.0), rotY, rotationAxis));
				}
				meshData_segTmp.push_back(mesh);
			}
			//pclProcessor.PrepareForObjectSegmentation();

		}
		glSegmentation.SetSegmentationState(IF_SEGSTATE_OBJECT_SEGMENTATION);

		if (glSegmentation.GetSegmentationMode() == SEGMENTATION_REGIONGROWTH)
		{
			if (glSegmentation.previewMode || glSegmentation.segmentValuesChanged)
			{
				pclProcessor.RegionGrowingSegmentation();

				glSegmentation.segmentValuesChanged = false;
			}
			if (glSegmentation.previewMode)
			{
				//openGLWin.SetWindowState(SEGMENTATION_PREVIEW);
				stateManager.SetWindowBusyState(IF_BUSYSTATE_DEFAULT);
				DebugUtility::DbgOut(L"Number of clusters: ", pclProcessor.GetClusterCount());
				isPreviewInitialized = false;
				return 0;
			}

		}
		if (glSegmentation.GetSegmentationMode() == SEGMENTATION_EUCLIDEAN)
		{
			if (glSegmentation.previewMode || glSegmentation.segmentValuesChanged)
			{
				pclProcessor.EuclideanSegmentation();
				glSegmentation.segmentValuesChanged = false;
			}

			if (glSegmentation.previewMode)
			{
				//openGLWin.SetWindowState(SEGMENTATION_PREVIEW);
				stateManager.SetWindowBusyState(IF_BUSYSTATE_DEFAULT);
				DebugUtility::DbgOut(L"Number of clusters: ", pclProcessor.GetClusterCount());
				isPreviewInitialized = false;
				return 0;
			}
		}

		pclProcessor.IndexEstimation();
		DebugUtility::DbgOut(L"Clustered");


		//elapsedTime = (t2.QuadPart - t1.QuadPart) * 1000.0 / frequency.QuadPart;
		//DebugUtility::DbgOut(L"Time for map creation ", elapsedTime);
		DebugUtility::DbgOut(L"Number of clusters: ", pclProcessor.GetRegionClusterCount());
		QueryPerformanceFrequency(&frequency);
		QueryPerformanceCounter(&t1);
		QueryPerformanceCounter(&t2);
		for (int i = 0; i < pclProcessor.GetRegionClusterCount(); i++)
		{
			std::vector<Vertex> clusterVertices;
			std::vector<Triangle> clusterIndices;
			if (!pclProcessor.ConvertToTriangleMesh(i, startingVertices, clusterVertices, clusterIndices, false))
				continue;
			//DebugUtility::DbgOut(L"Converted Triangle Mesh #",i);
			shared_ptr<MeshContainer> mesh(new MeshContainer);
			mesh->SetColorCode(i + pclProcessor.GetPlaneClusterCount() + 1);
			//mesh->ParseData(clusterVertices, clusterIndices);
			mesh->CopyToInternalData(clusterVertices, clusterIndices);
			DebugUtility::DbgOut(_T("Clean Seg #1"));
			mesh->CleanMesh();
			mesh->RemoveNonManifoldFace();

			//mesh->FillHoles((clusterVertices.size()) / 10);
			//mesh->CleanMesh();
			mesh->ParseData();
			//glm::vec3 cP = mesh->GetCenterPoint();
			//float iO = cP.x*planeC.x + cP.y*planeC.y + cP.z * planeC.z + planeC.w;
			//DebugUtility::DbgOut(L"mesh #" + to_wstring(i) + L" iO: ", iO);
			//if (iO >= 0.0f)
			meshData_segTmp.push_back(mesh);

		}
		QueryPerformanceCounter(&t2);
		elapsedTime = (t2.QuadPart - t1.QuadPart) * 1000.0 / frequency.QuadPart;
		DebugUtility::DbgOut(L"Filled MeshData in ", elapsedTime);
		appStatus.SetViewportPercentMessage(L"");
		//UNCOMMENT UNTIL HERE FOR SEGMENTATION
		glSegmentation.SetSegmentationState(IF_SEGSTATE_FINISHED);
		//openGLWin.SetWindowState(SEGMENTATION_FINISHED);
		//openGLWin.SetWindowMode(MODE_INTERACTION);
		//showColoredSegments = false;
		//segFinished = true;
		appStatus.SetStatusBarMessage(L"Segmented mesh in " + to_wstring(pclProcessor.GetRegionClusterCount()) + L"clusters.");
		//openGLWin.SetWindowBusyState(IF_BUSYSTATE_DEFAULT);
		return 0;

	}

	void SegmentationHelper::StartSegmentation()
	{
		appStatus.SetViewportStatusMessage(L"Segmenting mesh");
		stateManager.SetWindowMode(IF_MODE_SEGMENTATION);
		stateManager.SetWindowBusyState(IF_BUSYSTATE_BUSY);
		segmentationThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)&SegThreadMain, 0, 0, &sThreadId);
	}

	void SegmentationHelper::GeneratePreviewBuffers()
	{
		glGenBuffers(1, &segmentVBO);

		glBindBuffer(GL_ARRAY_BUFFER, segmentVBO);
		glBufferData(GL_ARRAY_BUFFER, previewVertices.size() * sizeof(Vertex), &previewVertices[0], GL_STATIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		glGenBuffers(1, &segmentIBO);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, segmentIBO);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, previewIndices.size() * sizeof(Triangle), &previewIndices[0], GL_STATIC_DRAW);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

		glGenVertexArrays(1, &segmentVAO);
		glBindVertexArray(segmentVAO);
		glBindBuffer(GL_ARRAY_BUFFER, segmentVBO);
		glEnableVertexAttribArray(0);
		glEnableVertexAttribArray(1);
		glEnableVertexAttribArray(2);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(0));
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(sizeof(float) * 3));
		glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(sizeof(float) * 6));
	}

	bool SegmentationHelper::IsWallReady()
	{
		return pclProcessor.GetInlierIndices().size() > 0;
	}

	bool SegmentationHelper::InitializePreview()
	{
		if (GetSegmentationState() == IF_SEGSTATE_PLANE_SEGMENTATION)
		{
			ColorIF color = { 0.4f, 0.0f, 0.0f };
			meshHelper.RemoveAllHighlights();
			meshHelper.HighlightObjectsInOriginal(pclProcessor.GetInlierIndices(), color, true);
			isPreviewInitialized = true;
		}
		else if (GetSegmentationState() == IF_SEGSTATE_OBJECT_SEGMENTATION)
		{
			std::random_device rd;
			std::mt19937 gen(rd());
			std::uniform_real_distribution<> dis(0.3f, 0.7f);
			meshHelper.RemoveAllHighlights();
			//meshHelper.GenerateBuffers();
			for (int i = 0; i < pclProcessor.GetRegionClusterCount(); i++)
			{
				ColorIF color = { dis(gen), dis(gen), dis(gen) };
				meshHelper.HighlightObjectsInOriginal(pclProcessor.GetColoredCloudIndices(i), color, false);
			}
			isPreviewInitialized = true;
		}
		stateManager.SetWindowBusyState(IF_BUSYSTATE_DEFAULT);
		return true;
	}

	bool SegmentationHelper::InitializeMinCutPreview()
	{
		if (stateManager.GetWindowMode() == IF_MODE_MINCUT && pclProcessor.MinCutChanged())
		{
			std::random_device rd;
			std::mt19937 gen(rd());
			std::uniform_real_distribution<> dis(0.3f, 0.7f);
			meshHelper.RemoveAllHighlights();
			//meshHelper.GenerateBuffers();
			for (int i = 0; i < pclProcessor.GetRegionClusterCount(); i++)
			{
				ColorIF color = { dis(gen), dis(gen), dis(gen) };
				meshHelper.HighlightObjects(glSelector.selectedIndex, pclProcessor.GetColoredCloudIndices(i), color, false);
			}
			isPreviewInitialized = true;
			pclProcessor.SetMinCutChanged(false);

			DebugUtility::DbgOut(L"Initializing MinCut Preview");
			return true;
		}
		return false;
	}

	void SegmentationHelper::DetermineMinCutRadius(glm::vec3 upperPoint, glm::vec3 centerPoint)
	{
		params.minCutRadius = abs(glm::distance(centerPoint, upperPoint));
		appStatus.SetStatusBarMessage(L"Determined radius as " + std::to_wstring(params.minCutRadius));

		DebugUtility::DbgOut(L"Determined radius as ", params.minCutRadius);
	}

	bool SegmentationHelper::InitializePreview(std::vector<Vertex> vertices, std::vector<Triangle> triangles)
	{
		previewVertices.clear();
		previewIndices.clear();
		for (int i = 0; i < vertices.size(); i++)
		{
			previewVertices.push_back(vertices[i]);
		}
		for (int i = 0; i < triangles.size(); i++)
		{
			previewIndices.push_back(triangles[i]);
		}

		if (GetSegmentationState() == IF_SEGSTATE_PLANE_SEGMENTATION)
			appStatus.SetStatusBarMessage(L"Showing wall preview.");
		if (GetSegmentationState() == IF_SEGSTATE_OBJECT_SEGMENTATION)
		{
			appStatus.SetStatusBarMessage(L"Showing region growth segmentation preview with " + to_wstring(pclProcessor.GetClusterCount()) + L"clusters");
			pclProcessor.coloredCloudReady = false;
		}
		return true;
	}

/*	void SegmentationHelper::RenderPreview()
	{
		int w = glControl.GetViewportWidth();
		int h = glControl.GetViewportHeight();
		previewMesh->Draw();
		//glText.PrepareForRender();
		//glText.RenderText(L"Clusters: ", pclProcessor.GetClusterCount(), 20, -0.98f, 0.85f, 2.0f / w, 2.0f / h);
		glEnable(GL_DEPTH_TEST);
		return;
	}*/

	void SegmentationHelper::AddMinCutForegroundPoint(glm::vec3 fPoint)
	{
		pclProcessor.AddMinCutForegroundPoint(fPoint);
	}

	void SegmentationHelper::AddMinCutBackgroundPoint(glm::vec3 bPoint)
	{
		pclProcessor.AddMinCutBackgroundPoint(bPoint);
	}

	void SegmentationHelper::ResetMinCutValues()
	{
		pclProcessor.ResetMinCutValues();
	}

	void SegmentationHelper::SetSegmentationMesh(int index)
	{
		pclProcessor.ConvertToCloud(meshData[index]->GetVertices(), meshData[index]->GetTriangles());
	}

	glm::vec3 SegmentationHelper::GetPreviewCenterPoint()
	{
		glm::vec3 centerPoint = glm::vec3(0.0f, 0.0f, 0.0f);
		for (int i = 0; i < previewVertices.size(); i++)
		{
			centerPoint.x += previewVertices[i].x;
			centerPoint.y += previewVertices[i].y;
			centerPoint.z += previewVertices[i].z;
		}
		centerPoint = centerPoint / (float)previewVertices.size();
		return centerPoint;
	}

	SegmentationState SegmentationHelper::GetSegmentationState()
	{
		return segState;
	}

	/*void SegmentationHelper::SetInteractionWindow(InteractionWindow* _interactionWindow)
	{
		interactionWindow = _interactionWindow;
	}*/

	void SegmentationHelper::SetSegmentationState(SegmentationState state)
	{
		segState = state;

		//if (interactionWindow)
		//	interactionWindow->UpdateSegmentationUI();
	}

	SegmentationMode SegmentationHelper::GetSegmentationMode()
	{
		return segMode;
	}

	void SegmentationHelper::SetSegmentationMode(SegmentationMode mode)
	{
		segMode = mode;
	}


	void SegmentationHelper::ClearForResume()
	{
		for (vector <shared_ptr<MeshContainer>>::iterator mI = meshData_segTmp.begin(); mI != meshData_segTmp.end(); ++mI)
		{
			(*mI)->CleanUp();
		}
		meshData_segTmp.clear();
		SetSegmentationState(IF_SEGSTATE_NONE);
		pclProcessor.ClearAll();
		isPreviewInitialized = false;
	}

	void SegmentationHelper::CleanUp()
	{
		glDeleteBuffers(1, &segmentVBO);
		glDeleteVertexArrays(1, &segmentVAO);
		glDeleteBuffers(1, &segmentIBO);
		for (vector <shared_ptr<MeshContainer>>::iterator mI = meshData_segTmp.begin(); mI != meshData_segTmp.end(); ++mI)
		{
			(*mI)->CleanUp();
		}
		meshData_segTmp.clear();
	}
}