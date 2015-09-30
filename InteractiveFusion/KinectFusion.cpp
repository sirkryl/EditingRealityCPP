#include "KinectFusion.h"
#include "FusionResources.h"
#include "KinectFusionProcessorFrame.h"
#include "stdafx.h"
#include "CommonStructs.h"
#include "DebugUtility.h"
#include "MeshContainer.h"

using namespace std;

namespace InteractiveFusion {

	#define WM_FRAMEREADY           (WM_USER + 0)
	#define WM_UPDATESENSORSTATUS   (WM_USER + 1)

	HWND parentHandle;
	bool processorLocked = false;
	int fusionTotalGpuMemory = 0;

	float framesPerSecond = 0;
	std::wstring currentStatus;

	KinectFusion::KinectFusion() :
		m_pD2DFactory(nullptr),
		m_bSavingMesh(false),
		m_saveMeshFormat(Ply),
		m_bInitializeError(false),
		m_pSensorChooserUI(nullptr),
		m_bColorCaptured(false),
		m_bUIUpdated(false)
	{
	}


	KinectFusion::~KinectFusion()
	{
		// clean up sensor chooser UI
		SAFE_DELETE(m_pSensorChooserUI);

		// clean up Direct2D renderer
		//SAFE_DELETE(m_pDrawReconstruction);

		// clean up Direct2D renderer
		//SAFE_DELETE(m_pDrawTrackingResiduals);

		// clean up Direct2D renderer
		//SAFE_DELETE(m_pDrawDepth);

		// clean up Direct2D
		SafeRelease(m_pD2DFactory);
	}

	void KinectFusion::Initialize(HWND _parentHandle, HWND _depthView, HWND _reconstructionView, HWND _trackingResidualsView)
	{
		parentHandle = _parentHandle;


		D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pD2DFactory);

		int width = m_params.m_cDepthWidth;
		int height = m_params.m_cDepthHeight;

		// Create and initialize a new Direct2D image renderer (take a look at ImageRenderer.h)
		// We'll use this to draw the data we receive from the Kinect to the screen
		//m_pDrawDepth = new ImageRenderer();
		HRESULT hr = m_pDrawDepth.Initialize(
			_depthView,
			m_pD2DFactory,
			width,
			height,
			width * sizeof(long));

		if (FAILED(hr))
		{
			DebugUtility::DbgOut(L"KinectFusion::Initialize::ERROR::Failed to initialize the Direct2D draw device.");
			m_bInitializeError = true;
		}

		//m_pDrawReconstruction = new ImageRenderer();
		hr = m_pDrawReconstruction.Initialize(
			_reconstructionView,
			m_pD2DFactory,
			width,
			height,
			width * sizeof(long));

		if (FAILED(hr))
		{
			DebugUtility::DbgOut(L"KinectFusion::Initialize::ERROR::Failed to initialize the Direct2D draw device.");
			m_bInitializeError = true;
		}

		//m_pDrawTrackingResiduals = new ImageRenderer();
		hr = m_pDrawTrackingResiduals.Initialize(
			_trackingResidualsView,
			m_pD2DFactory,
			width,
			height,
			width * sizeof(long));

		if (FAILED(hr))
		{
			DebugUtility::DbgOut(L"KinectFusion::Initialize::ERROR::Failed to initialize the Direct2D draw device.");
			m_bInitializeError = true;
		}

		if (FAILED(m_processor.SetWindow(parentHandle, WM_FRAMEREADY, WM_UPDATESENSORSTATUS)) ||
			FAILED(m_processor.SetParams(m_params)) ||
			FAILED(m_processor.StartProcessing()))
		{
			m_bInitializeError = true;
		}

		m_saveMeshFormat = m_params.m_saveMeshType;
	}
	
	void KinectFusion::HandleCompletedFrame()
	{
		processorLocked = true;
		//DebugUtility::DbgOut(L"KinectFusion::HandleCompletedFrame()::Beginning");
		KinectFusionProcessorFrame const* pFrame = nullptr;

		// Flush any extra WM_FRAMEREADY messages from the queue
		MSG msg;
		while (PeekMessage(&msg, parentHandle, WM_FRAMEREADY, WM_FRAMEREADY, PM_REMOVE)) {}

		m_processor.LockFrame(&pFrame);

		if (!m_bSavingMesh && !pauseRendering) // don't render while a mesh is being saved
		{
			if (m_processor.IsVolumeInitialized())
			{
				m_pDrawDepth.Draw(pFrame->m_pDepthRGBX, pFrame->m_cbImageSize);
				m_pDrawReconstruction.Draw(pFrame->m_pReconstructionRGBX, pFrame->m_cbImageSize);
				m_pDrawTrackingResiduals.Draw(pFrame->m_pTrackingDataRGBX, pFrame->m_cbImageSize);
			}

			wstring status = pFrame->m_statusMessage;
			if (status.length() > 0)
				SetStatus(status);
			framesPerSecond = pFrame->m_fFramesPerSecond;
		}

		//if (pFrame->m_bIntegrationResumed)
		//{
		//	DebugUtility::DbgOut(L"Integration resumed!?");

		//	m_params.m_bPauseIntegration = false;
		//	//CheckDlgButton(m_hWnd, IDC_SDEBUG_CHECK_PAUSE_INTEGRATION, BST_UNCHECKED);
		//	m_processor.SetParams(m_params);
		//}
		//else if (m_processor.IsCameraPoseFinderAvailable() && !m_params.m_bPauseIntegration)
		//{
		//	m_params.m_bPauseIntegration = true;
		//	//CheckDlgButton(m_hWnd, IDC_SDEBUG_CHECK_PAUSE_INTEGRATION, BST_CHECKED);
		//	m_processor.SetParams(m_params);
		//}

		if (!m_bUIUpdated && m_processor.IsVolumeInitialized())
		{
			const int Mebi = 1024 * 1024;
			
			fusionTotalGpuMemory = (int)(pFrame->m_deviceMemory / 1024);
			DebugUtility::DbgOut(L"Detected GPU Memory:", (int)(pFrame->m_deviceMemory / 1024));
			// We now create both a color and depth volume, doubling the required memory, so we restrict
			// which resolution settings the user can choose when the graphics card is limited in memory.
			if (pFrame->m_deviceMemory <= 1 * Mebi)  // 1GB
			{
				/*
				// Disable 640 voxel resolution in all axes - cards with only 1GB cannot handle this
				HWND hButton = GetDlgItem(m_hWnd, IDC_SDEBUG_CHECK_VOXELS_X_640);
				EnableWindow(hButton, FALSE);
				hButton = GetDlgItem(m_hWnd, IDC_SDEBUG_CHECK_VOXELS_Y_640);
				EnableWindow(hButton, FALSE);
				hButton = GetDlgItem(m_hWnd, IDC_SDEBUG_CHECK_VOXELS_Z_640);
				EnableWindow(hButton, FALSE);
				if (Is64BitApp() == FALSE)
				{
					// Also disable 512 voxel resolution in one arbitrary axis on 32bit machines
					hButton = GetDlgItem(m_hWnd, IDC_SDEBUG_CHECK_VOXELS_Y_512);
					EnableWindow(hButton, FALSE);
				}*/
			}
			else if (pFrame->m_deviceMemory <= 2 * Mebi)  // 2GB
			{
				/*
				if (Is64BitApp() == FALSE)
				{
					// Disable 640 voxel resolution in one arbitrary axis on 32bit machines
					HWND hButton = GetDlgItem(m_hWnd, IDC_SDEBUG_CHECK_VOXELS_Y_640);
					EnableWindow(hButton, FALSE);
				}
				*/
				// True 64 bit apps seem to be more able to cope with large volume sizes.
			}

			m_bUIUpdated = true;
		}
		//DebugUtility::DbgOut(L"KinectFusion::HandleCompletedFrame()::Ending");
		m_bColorCaptured = pFrame->m_bColorCaptured;

		m_processor.UnlockFrame();
		
		processorLocked = false;
	}

	void KinectFusion::PrepareMeshSave()
	{
		m_bSavingMesh = true;
		m_params.m_bPauseIntegration = true;
		//m_processor.SetParams(m_params);
		m_processor.PauseProcessing(true);
	}
	bool KinectFusion::IsReconstructionReady()
	{
		return reconstructionReady;
	}
	void KinectFusion::FinishMeshSave()
	{
		DebugUtility::DbgOut(L"FinishMeshSave BEGIN");
		if (scannedVertices.size() > 0)
			scannedVertices.clear();
		if (scannedTriangles.size() > 0)
			scannedTriangles.clear();
		// Restore pause state of integration
		//m_processor.SetParams(m_params);
		m_processor.LockMutex();
		/*while (m_processor.IsProcessorLocked())
		{
		DebugUtility::DbgOut(L"Waiting for Processor Lock");
		}*/
		INuiFusionColorMesh *mesh = nullptr;
		HRESULT hr = m_processor.CalculateMesh(&mesh,2);

		if (mesh == nullptr)
		{
			DebugUtility::DbgOut(L"KinectFusion::FinishScan()::Mesh is nullptr");
			return;
		}

		const Vector3 *scanVertices = NULL;
		mesh->GetVertices(&scanVertices);
		DebugUtility::DbgOut(L"KinectFusion::FinishScan()::Scanned Vertices: ", (int)mesh->VertexCount());

		const Vector3 *scanNormals = NULL;
		mesh->GetVertices(&scanNormals);
		DebugUtility::DbgOut(L"KinectFusion::FinishScan()::Scanned Normals: ", (int)mesh->NormalCount());

		const int *scanColors = NULL;
		mesh->GetColors(&scanColors);
		DebugUtility::DbgOut(L"KinectFusion::FinishScan()::Scanned Colors: ", (int)mesh->ColorCount());
		//vector<Vertex> meshVertices;

		for (int i = 0; i < mesh->VertexCount(); i++)
		{
			Vertex newVertex;
			newVertex.x = scanVertices[i].x;
			newVertex.y = -scanVertices[i].y;
			newVertex.z = -scanVertices[i].z;
			newVertex.normal_x = scanNormals[i].x;
			newVertex.normal_y = -scanNormals[i].y;
			newVertex.normal_z = -scanNormals[i].z;
			newVertex.r = (float)((scanColors[i] >> 16) & 255) / 255.0f;
			newVertex.g = (float)((scanColors[i] >> 8) & 255) / 255.0f;
			newVertex.b = (float)(scanColors[i] & 255) / 255.0f;

			scannedVertices.push_back(newVertex);

		}
		DebugUtility::DbgOut(L"KinectFusion::FinishScan()::I've got some vertices: ", (int)scannedVertices.size());

		const int *scanTriangleIndices = NULL;
		mesh->GetTriangleIndices(&scanTriangleIndices);
		DebugUtility::DbgOut(L"KinectFusion::FinishScan()::Scanned Triangles: ", (int)mesh->TriangleVertexIndexCount());
		//vector<Triangle> meshTriangles;
		if (mesh->TriangleVertexIndexCount() <= 0)
		{
			DebugUtility::DbgOut(L"No vertices in scanned mesh");
			return;
		}
		for (int i = 0; i < mesh->TriangleVertexIndexCount() - 2; i += 3)
		{
			Triangle newTriangle;
			newTriangle.v1 = scanTriangleIndices[i];
			newTriangle.v2 = scanTriangleIndices[i + 1];
			newTriangle.v3 = scanTriangleIndices[i + 2];
			scannedTriangles.push_back(newTriangle);
		}
		if (SUCCEEDED(hr))
		{
			// Save mesh
			//std::wstring fileName = L"data\\models\\output.ply";
			//LPOLESTR fileString = W2OLE((wchar_t*)fileName.c_str());
			//hr = WriteAsciiPlyMeshFile(mesh, fileString, true, m_bColorCaptured);


			hr = S_OK;
			// Release the mesh
			DebugUtility::DbgOut(L"KinectFusion::FinishScan()::Before SafeRelease");
			SafeRelease(mesh);
			DebugUtility::DbgOut(L"KinectFusion::FinishScan()::After SafeRelease");
		}

		m_processor.UnlockMutex();


		//scannedMesh = MeshContainer(meshVertices, meshTriangles);

		m_bSavingMesh = false;
		reconstructionReady = true;
		DebugUtility::DbgOut(L"Setting ReconstructionReady");
		//m_processor.PauseProcessing(false);
	}

	std::vector<Vertex>& KinectFusion::GetScannedVertices()
	{
		return scannedVertices;
	}
	
	std::vector<Triangle>& KinectFusion::GetScannedTriangles()
	{
		return scannedTriangles;
	}

	void KinectFusion::UpdateSensorStatus(WPARAM _wParam)
	{
		if (m_pSensorChooserUI != nullptr)
		{
			m_pSensorChooserUI->UpdateSensorStatus(static_cast<DWORD>(_wParam));
		}
	}

	void KinectFusion::ResolvePotentialSensorConflict(LPARAM _lParam)
	{
		const NMHDR* pNMHeader = reinterpret_cast<const NMHDR*>(_lParam);
		if (pNMHeader->code == NSCN_REFRESH && pNMHeader->idFrom == IDC_SENSORCHOOSER)
		{
			m_processor.ResolveSensorConflict();
		}
	}

	void KinectFusion::ChangeVolumeSize(int _voxelsPerMeter)
	{
		m_params.m_reconstructionParams.voxelsPerMeter = (float)_voxelsPerMeter;
	}

	int KinectFusion::GetGpuMemory()
	{
		return fusionTotalGpuMemory;
	}

	float KinectFusion::GetFramesPerSecond()
	{
		return framesPerSecond;
	}

	glm::mat4 KinectFusion::GetTransformedCameraMatrix()
	{
		glm::mat4 glmLeftHandedCamTransform(1.0f);

		//correct transformations:
		/* 1 / -1 / -1 / 1 
		  -1 /  1 /  1 / 1
		  -1 /  1 /  1 / 1
		   1 / -1 / -1 / 1 */
		Matrix4 leftHandedCamTransform = m_processor.GetWorldToCameraTransform();
		glmLeftHandedCamTransform[0] = glm::vec4(leftHandedCamTransform.M11, -leftHandedCamTransform.M12, -leftHandedCamTransform.M13, leftHandedCamTransform.M14);
		glmLeftHandedCamTransform[1] = glm::vec4(-leftHandedCamTransform.M21, leftHandedCamTransform.M22, leftHandedCamTransform.M23, leftHandedCamTransform.M24);
		glmLeftHandedCamTransform[2] = glm::vec4(-leftHandedCamTransform.M31, leftHandedCamTransform.M32, leftHandedCamTransform.M33, leftHandedCamTransform.M34);
		glmLeftHandedCamTransform[3] = glm::vec4(leftHandedCamTransform.M41, -leftHandedCamTransform.M42, -leftHandedCamTransform.M43, leftHandedCamTransform.M44);

		return glmLeftHandedCamTransform;
	}

	void KinectFusion::ResetScan()
	{
		m_processor.ResetReconstruction();
		m_params.m_bPauseIntegration = false;
		m_processor.hardStopIntegration = false;
	}

	void KinectFusion::SetStatus(wstring _status)
	{
		currentStatus = _status;
	}

	wstring KinectFusion::GetAndResetStatus()
	{
		wstring outputStatus = currentStatus;
		currentStatus = L"";
		return outputStatus;
	}


	void KinectFusion::PauseProcessing(bool flag)
	{
		m_processor.PauseProcessing(flag);
	}

	void KinectFusion::PauseRendering()
	{
		pauseRendering = true;
		m_params.m_bPauseIntegration = true;
		DebugUtility::DbgOut(L"Pause Rendering");
	}

	void KinectFusion::PauseIntegration()
	{
		m_params.m_bPauseIntegration = true;
		m_processor.hardStopIntegration = true;
		DebugUtility::DbgOut(L"Pause Integration");

	}

	void KinectFusion::ResumeRendering()
	{
		pauseRendering = false;
		m_params.m_bPauseIntegration = false;
		DebugUtility::DbgOut(L"Resume Rendering");
	}

	void KinectFusion::CleanUp()
	{
		DebugUtility::DbgOut(L"KinectFusion::CleanUp()");
		m_processor.StopProcessing();

		m_pDrawReconstruction.CleanUp();
		m_pDrawTrackingResiduals.CleanUp();
		m_pDrawDepth.CleanUp();

		SAFE_DELETE(m_pSensorChooserUI);

		SafeRelease(m_pD2DFactory);
	}
}