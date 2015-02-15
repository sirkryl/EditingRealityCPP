#include "PlaneSegmenter.h"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/io.h>
#include <pcl/features/normal_3d.h>
#include <glm/gtc/matrix_transform.hpp>
#include "DebugUtility.h"
#include "ModelData.h"

namespace InteractiveFusion {



	PlaneSegmenter::PlaneSegmenter() :
		Segmenter()
	{
		allPlaneSegmentIndices = pcl::PointIndices::Ptr(new pcl::PointIndices);
		indicesUsedForNextPlaneSegmentation = pcl::PointIndices::Ptr(new pcl::PointIndices);
		axisUsedForSegmentation = Eigen::Vector3f(0, 1, 0);

		mainCloudWithoutNormals = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
		mainCloudNormals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
		allPlaneSegmentIndices = pcl::PointIndices::Ptr(new pcl::PointIndices);
		indicesUsedForNextPlaneSegmentation = pcl::PointIndices::Ptr(new pcl::PointIndices);
		indicesFromLastPlaneSegmentation = pcl::PointIndices::Ptr(new pcl::PointIndices);
	}


	PlaneSegmenter::~PlaneSegmenter()
	{
	}

	void PlaneSegmenter::SetSegmentationParameters(PlaneSegmentationParams* _segmentationParams)
	{
		DebugUtility::DbgOut(L"PlaneSegmenter::SetSegmentationParameters");

		PlaneSegmentationParams* temporaryParams = dynamic_cast<PlaneSegmentationParams*>(_segmentationParams);
		if (temporaryParams != nullptr)
			segmentationParameters = *temporaryParams;
		else
			segmentationParameters = PlaneSegmentationParams();
	}

	bool PlaneSegmenter::ConvertToPointCloud(MeshContainer &_mesh)
	{
		CleanUp();
		Segmenter::ConvertToPointCloud(_mesh);

		if (mainCloud->points.size() > 0)
		{
			copyPointCloud(*mainCloud, *mainCloudWithoutNormals);
			copyPointCloud(*mainCloud, *mainCloudNormals);
			return true;
		}
		return false;
	}

	bool PlaneSegmenter::Segment()
	{
		//temporarySegmentationClusterIndices.clear();

		DebugUtility::DbgOut(L"Before segmentation");
		DebugUtility::DbgOut(L"MainCloudWithoutNormals size: ", (int)mainCloudWithoutNormals->points.size());
		DebugUtility::DbgOut(L"mainCloudNormals size: ", (int)mainCloudNormals->points.size());

		//pcl::search::KdTree<pcl::PointXYZRGB>::Ptr	search(new pcl::search::KdTree<pcl::PointXYZRGB>);
		// Object for storing the plane model coefficients.
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		// Create the segmentation object.
		pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> segmentation;
		//search->setInputCloud(cloud_f);
		segmentation.setInputCloud(mainCloudWithoutNormals);
		segmentation.setInputNormals(mainCloudNormals);
		segmentation.setProbability(0.99);
		segmentation.setMaxIterations(300);

		if (indicesUsedForNextPlaneSegmentation->indices.size() > 0)
			segmentation.setIndices(indicesUsedForNextPlaneSegmentation);
		//Eigen::Vector3f segAxis(0, 1, 0);
		segmentation.setAxis(axisUsedForSegmentation);
		segmentation.setNormalDistanceWeight(segmentationParameters.planeSmoothness);
		segmentation.setEpsAngle(10.0f * (M_PI / 180.0f));
		// Configure the object to look for a plane.
		if (axisChangedCount <= 2)
			segmentation.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
		else
			segmentation.setModelType(pcl::SACMODEL_NORMAL_PLANE);// SACMODEL_NORMAL_PLANE);
		// Use RANSAC method.

		segmentation.setMethodType(pcl::SAC_RANSAC);
		// Set the maximum allowed distance to the model.
		segmentation.setDistanceThreshold(segmentationParameters.planeThickness);
		// Enable model coefficient refinement (optional).
		segmentation.setOptimizeCoefficients(true);
		pcl::PointIndices::Ptr inlierIndices(new pcl::PointIndices);
		//inlierIndices->indices.clear();

		segmentation.segment(*inlierIndices, *coefficients);

		if (inlierIndices->indices.size() <= mainCloud->points.size() / 15)
		{
			if (axisChangedCount == 0)
			{
				axisChangedCount++;
				axisUsedForSegmentation = Eigen::Vector3f(1, 0, 0);
				return Segment();
			}
			else if (axisChangedCount == 1)
			{
				axisChangedCount++;
				axisUsedForSegmentation = Eigen::Vector3f(0, 0, 1);
				return Segment();
			}
			else if (axisChangedCount == 2)
			{
				axisChangedCount++;
				return Segment();
			}
			temporarySegmentationClusterIndices.push_back(GetIndicesThatWereNotSegmented());
			return false;
		}

		temporarySegmentationClusterIndices.push_back(*inlierIndices);
		planeCoefficients.push_back(coefficients);
		return true;
	}

	void PlaneSegmenter::ConfirmLastSegment()
	{
		if (temporarySegmentationClusterIndices.size() == 0)
			return;

		UpdateIndicesUsedForNextSegmentation();
	}

	void PlaneSegmenter::RejectLastSegment()
	{
		if (temporarySegmentationClusterIndices.size() == 0)
			return;

		UpdateIndicesUsedForNextSegmentation();

		RemoveLastSegmentForNewSegmentation();
	}

	void PlaneSegmenter::RemoveLastSegmentForNewSegmentation()
	{
		if (temporarySegmentationClusterIndices.size() == 0)
			return;

		temporarySegmentationClusterIndices.pop_back();
		planeCoefficients.pop_back();
	}

	PlaneParameters PlaneSegmenter::GetModelCoefficients(int _clusterIndex)
	{
		PlaneParameters outputParameters;

		if (_clusterIndex < planeCoefficients.size())
		{
			outputParameters.x = planeCoefficients[_clusterIndex]->values[0];
			outputParameters.y = planeCoefficients[_clusterIndex]->values[1];
			outputParameters.z = planeCoefficients[_clusterIndex]->values[2];
			outputParameters.d = planeCoefficients[_clusterIndex]->values[3];
		}
		return outputParameters;
	}

	pcl::PointIndices PlaneSegmenter::GetIndicesThatWereNotSegmented()
	{
		pcl::PointIndices::Ptr notPlaneIndices(new pcl::PointIndices);
		pcl::PointIndices::Ptr allPlaneIndices(new pcl::PointIndices);
		for (auto &indexVector : temporarySegmentationClusterIndices)
		{
			for (auto &indices : indexVector.indices)
			{
				allPlaneIndices->indices.push_back(indices);
			}
		}

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr temporaryCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		pcl::ExtractIndices<pcl::PointXYZRGB> extract(true);
		extract.setInputCloud(mainCloudWithoutNormals);
		extract.setIndices(allPlaneIndices);

		extract.setNegative(false);
		extract.filter(*temporaryCloud);
		extract.getRemovedIndices(*notPlaneIndices);

		allPlaneIndices->indices.clear();

		return *notPlaneIndices;
	}

	void PlaneSegmenter::UpdateHighlights(ModelData* _modelData)
	{
		if (GetClusterCount() == 0)
			return;
		DebugUtility::DbgOut(L"UpdatePlaneSegmentationHighlights:: ", GetClusterCount() - 1);
		std::vector<int> trianglesToBeColored = GetClusterIndices(GetClusterCount() - 1);

		_modelData->TemporarilyColorTriangles(0, trianglesToBeColored, ColorIF{ 0.5f, 0.0f, 0.0f }, true);
	}

	void PlaneSegmenter::UpdateIndicesUsedForNextSegmentation()
	{
		if (temporarySegmentationClusterIndices.size() == 0)
			return;

		for (auto &indices : temporarySegmentationClusterIndices[temporarySegmentationClusterIndices.size() - 1].indices)
		{
			allPlaneSegmentIndices->indices.push_back(indices);
		}

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr temporaryCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		pcl::ExtractIndices<pcl::PointXYZRGB> extract(true);
		extract.setInputCloud(mainCloudWithoutNormals);
		extract.setIndices(allPlaneSegmentIndices);

		extract.setNegative(false);
		extract.filter(*temporaryCloud);
		extract.getRemovedIndices(*indicesUsedForNextPlaneSegmentation);
	}

	void PlaneSegmenter::FinishSegmentation(ModelData* _inputModelData, ModelData* _outputModelData)
	{
		for (int i = 0; i < GetClusterCount(); i++)
		{
			PlaneParameters planeParameters = GetModelCoefficients(i);

			DebugUtility::DbgOut(L"Finishing plane segmentation... index: ", i);
			if (planeParameters.IsInitialized())
			{
				//TODO: HOW TO DETERMINE GROUND PLANE (if it's not the first element?)
				if (i == 0)
				{
					glm::vec3 normalVector = glm::normalize(glm::vec3(planeParameters.x, planeParameters.y, planeParameters.z));

					glm::vec3 yAxis = glm::normalize(glm::vec3(0.0f, 1.0f, 0.0f));

					if (normalVector.y < 0)
						yAxis.y = -1.0f;

					glm::vec3 rotationAxis = glm::cross(normalVector, yAxis);

					//float xRotation = glm::acos(glm::dot(normalVector, glm::vec3(1.0f, 0.0f, 0.0f)));
					float rotY = glm::acos(glm::dot(normalVector, yAxis)) * 180.0f / M_PI;
					//float zRotation = glm::acos(glm::dot(normalVector, glm::vec3(0.0f, 0.0f, 1.0f)));
					//_temporaryModelData->SetGroundAlignmentRotation(glm::rotate(glm::mat4(1.0), rotY, rotationAxis));
					_outputModelData->SetGroundAlignmentRotation(rotY, rotationAxis);
				}
				_outputModelData->AddPlaneMeshToData(ConvertToMesh(i), planeParameters);
			}
			else
				_outputModelData->AddObjectMeshToData(ConvertToMesh(i));

		}
	}

	void PlaneSegmenter::CleanUp()
	{
		DebugUtility::DbgOut(L"PlaneSegmenter::CleanUp()");
		axisUsedForSegmentation = { 0, 1, 0 };

		mainCloudWithoutNormals->clear();
		mainCloudNormals->clear();

		allPlaneSegmentIndices->indices.clear();
		indicesUsedForNextPlaneSegmentation->indices.clear();
		indicesFromLastPlaneSegmentation->indices.clear();
		planeCoefficients.clear();
		Segmenter::CleanUp();
	}
}