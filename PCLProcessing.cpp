#include "common.h"
#include "PCLProcessing.h"
#include "openGLWin.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include "resource.h"
#include <pcl/kdtree/kdtree.h>
#include <unordered_set>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/io/vtk_lib_io.h>
#include <pcl/ros/conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/surface/poisson.h>
#include <pcl/features/normal_3d.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr mainCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusteredClouds;
std::vector<std::vector<int>> clusteredIndices;

unordered_map<int, vector<int>> indexMap;
unordered_map<float, vector<float>> vertexMap;


bool PCLProcessor::PlaneSegmentation() {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);

	// Create the segmentation object for the planar model and set all the parameters
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	//pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

	pcl::PCDWriter writer;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setRadiusLimits(0.22, 10000.0);
	seg.setDistanceThreshold(0.027);

	std::vector<std::vector<int>> indexVector;
	//std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> planarClusterVector;
	int numberOfElements = 0;
	int i = 0, nr_points = (int)mainCloud->points.size();
	while (mainCloud->points.size() > 0.3 * nr_points)
	{
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>());
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(mainCloud);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			// std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZRGB> extract;
		extract.setInputCloud(mainCloud);
		extract.setIndices(inliers);
		extract.setNegative(false);

		/*std::vector<int> tmp;

		for (std::vector<int>::const_iterator pit = inliers->indices.begin(); pit != inliers->indices.end(); pit++) {
		tmp.push_back(*pit);
		}*/


		//indexVector.push_back(tmp);
		// Get the points associated with the planar surface
		extract.filter(*cloud_plane);
		if (numberOfElements == 0)
			clusteredIndices.push_back(inliers->indices);
		else
			CalculateIndicesForCluster(cloud_plane);
		clusteredClouds.push_back(cloud_plane);
		//numberOfElements = tmp.size();
		// std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
		numberOfElements++;
		// Remove the planar inliers, extract the rest
		extract.setNegative(true);
		extract.filter(*cloud_f);
		*mainCloud = *cloud_f;
	}

	cDebug::DbgOut(L"Number of planes: ", numberOfElements);

	return true;
}

void PCLProcessor::KeyDown(const pcl::visualization::KeyboardEvent &keyEvent, void* viewer_void) {
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
	if (keyEvent.keyDown()){
		if (keyEvent.getKeySym() == "k") {
			openGLWin.kSearchValue += 1;
			//cDebug::DbgOut(L"kSearch value up to: ", kSearchValue);
		}
		else if (keyEvent.getKeySym() == "j") {
			openGLWin.kSearchValue -= 1;
			//cDebug::DbgOut(L"kSearch value down to: ", kSearchValue);
		}
		if (keyEvent.getKeySym() == "n") {
			openGLWin.numberOfNeighbors += 1;
			//cDebug::DbgOut(L"numberOfNeighbors value up to: ", numberOfNeighbors);
		}
		else if (keyEvent.getKeySym() == "b") {
			openGLWin.numberOfNeighbors -= 1;
			//cDebug::DbgOut(L"numberOfNeighbors value down to: ", numberOfNeighbors);
		}
		if (keyEvent.getKeySym() == "z") {
			openGLWin.smoothnessThreshold += 0.1;
			//cDebug::DbgOut(L"smoothnessThreshold value up to: ", smoothnessThreshold);
		}
		else if (keyEvent.getKeySym() == "t") {
			openGLWin.smoothnessThreshold -= 0.1;
			//cDebug::DbgOut(L"smoothnessThreshold value down to: ", smoothnessThreshold);
		}
		if (keyEvent.getKeySym() == "Return") {
			cDebug::DbgOut(L"ENTER: ", 0);
			//newSegmentation = true;
		}
		cDebug::DbgOut(L"Key code: ", keyEvent.getKeyCode());
		wstring st(keyEvent.getKeySym().begin(), keyEvent.getKeySym().end());
		cDebug::DbgOut(st);
	}
}

void ViewerInit(pcl::visualization::PCLVisualizer& viewer)
{
	cDebug::DbgOut(L"init ", 0);
}

void ViewerCallback(pcl::visualization::PCLVisualizer& viewer)
{
	std::stringstream ss_kSearch;
	ss_kSearch << "kSearchValue: " << openGLWin.kSearchValue;
	viewer.removeShape("kSearch", 0);
	viewer.addText(ss_kSearch.str(), 20, 30, "kSearch", 0);
	std::stringstream ss_noN;
	ss_noN << "numberOfNeighbors: " << openGLWin.numberOfNeighbors;
	viewer.removeShape("noN", 0);
	viewer.addText(ss_noN.str(), 20, 50, "noN", 0);
	std::stringstream ss_sT;
	ss_sT << "smoothnessThreshold: " << openGLWin.smoothnessThreshold;
	viewer.removeShape("sT", 0);
	viewer.addText(ss_sT.str(), 20, 70, "sT", 0);
	std::stringstream ss_cT;
	ss_cT << "curvatureThreshold: " << openGLWin.curvatureThreshold;
	viewer.removeShape("cT", 0);
	viewer.addText(ss_cT.str(), 20, 90, "cT", 0);
	/*static unsigned count = 0;
	std::stringstream ss;
	ss << "Once per viewer loop: " << count++;
	viewer.removeShape("text", 0);
	viewer.addText(ss.str(), 200, 300, "text", 0);*/

	/*if (segmentationFinished)
	{
	segmentationFinished = false;
	viewer.updatePointCloud(coloredSegmentedCloud);
	}*/

}

void PCLProcessor::ShowViewer() {
	/*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	//pcl::visualization::PCLVisualizer viewer("3D Viewer");
	//viewer = new pcl::visualization::PCLVisualizer("3D Viewer");
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(coloredSegmentedCloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(coloredSegmentedCloud, rgb, "Cluster viewer");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Cluster viewer");
	//viewer.addCoordinateSystem(1.0, 0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(
	0.0f, 0.0f, 0.0f,
	0.0f, 0.0f, -1.0f,
	0.0f, 1.0f, 0.0f,
	0);
	//camPosition = glm::vec3(0.0f, 0.0f, 0.0f);
	//camLookAt = glm::vec3(0.0f, 0.0, -1.0f);
	//camUpDirection = glm::vec3(0.0f, 1.0f, 0.0f);
	while (!viewer->wasStopped())
	{
	viewer->spinOnce(100);
	Sleep(100);
	//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	//HWND hWnd = (HWND)viewer->getRenderWindow()->GetGenericWindowId();
	//delete viewer;
	//DestroyWindow(hWnd);
	//viewer->close();
	//boost::this_thread::yield();
	//viewer = NULL;
	*/

	pcl::visualization::CloudViewer viewer("Cluster viewer");
	viewer.showCloud(coloredSegmentedCloud);
	viewer.runOnVisualizationThreadOnce(ViewerInit);
	viewer.runOnVisualizationThread(ViewerCallback);

	while (!viewer.wasStopped())
	{

	}
	IndexEstimation();
}

void PCLProcessor::IndexEstimation()
{
	LARGE_INTEGER frequency;        // ticks per second
	LARGE_INTEGER t1, t2;           // ticks
	double elapsedTime;
	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&t1);
	QueryPerformanceCounter(&t2);

	int j = 0;

	clusterCount = segmentedClusterIndices.size();

	clusteredClouds.clear();
	//clusteredclouds = new pcl::PointCloud<pcl::PointXYZRGB>::Ptr[clusterCount];
	//clusteredcloudsindices.resize(clusterCount);


	for (std::vector<pcl::PointIndices>::const_iterator it = segmentedClusterIndices.begin(); it != segmentedClusterIndices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
		std::vector<int> *tmpClusterIndices(new vector<int>);

		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
			cloud_cluster->points.push_back(mainCloud->points[*pit]); //*
			//clusteredcloudsindices[j].push_back(*pit);
			if (j == 0)
				tmpClusterIndices->push_back(*pit);
		}
		//clusteredIndices.push_back(*tmpClusterIndices);
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		// std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
		if (j == 0)
			clusteredIndices.push_back(*tmpClusterIndices);
		else
			CalculateIndicesForCluster(cloud_cluster);
		clusteredClouds.push_back(cloud_cluster);
		//clusteredclouds[j] = cloud_cluster;

		j++;
	}
	QueryPerformanceCounter(&t2);
	elapsedTime = (t2.QuadPart - t1.QuadPart) * 1000.0 / frequency.QuadPart;
	cDebug::DbgOut(L"Index estimation in ", elapsedTime);
}

bool PCLProcessor::RegionGrowingSegmentation() {
	pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> >(new pcl::search::KdTree<pcl::PointXYZRGB>);

	LARGE_INTEGER frequency;        // ticks per second
	LARGE_INTEGER t1, t2;           // ticks
	double elapsedTime;
	pcl::PointCloud <pcl::Normal>::Ptr estNormals(new pcl::PointCloud <pcl::Normal>);
	if (openGLWin.estimateNormals)
	{
		std::wstringstream ws;
		ws << L"Estimating normals... ";
		wstring statusMsg(ws.str());
		const TCHAR *c_str = statusMsg.c_str();
		SetDlgItemText(openGLWin.glWindowParent, IDC_IM_STATUS, c_str);

		QueryPerformanceFrequency(&frequency);
		QueryPerformanceCounter(&t1);
		QueryPerformanceCounter(&t2);
		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
		normal_estimator.setSearchMethod(tree);
		normal_estimator.setInputCloud(mainCloud);
		normal_estimator.setKSearch(openGLWin.kSearchValue);
		//normal_estimator.setRadiusSearch(0.01);
		normal_estimator.compute(*estNormals);

		QueryPerformanceCounter(&t2);
		elapsedTime = (t2.QuadPart - t1.QuadPart) * 1000.0 / frequency.QuadPart;
		cDebug::DbgOut(L"Normal estimation in ", elapsedTime);
	}
	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&t1);
	QueryPerformanceCounter(&t2);
	//std::vector<pcl::PointIndices> cluster_indices;
	segmentedClusterIndices.clear();

	std::wstringstream ws;
	ws << L"Processing segmentation... ";
	wstring statusMsg(ws.str());
	const TCHAR *c_str = statusMsg.c_str();
	SetDlgItemText(openGLWin.glWindowParent, IDC_IM_STATUS, c_str);

	pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
	reg.setMinClusterSize(openGLWin.minClusterSize);
	reg.setMaxClusterSize(openGLWin.maxClusterSize * 1000);
	reg.setSearchMethod(tree);
	reg.setResidualTestFlag(true);
	reg.setNumberOfNeighbours(openGLWin.numberOfNeighbors);
	reg.setInputCloud(mainCloud);

	if (openGLWin.estimateNormals)
		reg.setInputNormals(estNormals);
	else
		reg.setInputNormals(normals);
	reg.setSmoothnessThreshold((openGLWin.smoothnessThreshold / 10.0) / 180.0 * M_PI);
	reg.setCurvatureThreshold((openGLWin.curvatureThreshold / 10.0));

	reg.extract(segmentedClusterIndices);
	clusterCount = segmentedClusterIndices.size();
	QueryPerformanceCounter(&t2);
	elapsedTime = (t2.QuadPart - t1.QuadPart) * 1000.0 / frequency.QuadPart;
	cDebug::DbgOut(L"Region based clustering in ", elapsedTime);

	//pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
	coloredSegmentedCloud = reg.getColoredCloud();
	if (clusterCount > 0)
		coloredCloudReady = true;
	//ShowViewer();
	return true;

}

bool PCLProcessor::EuclideanSegmentation() {
	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud(mainCloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	ec.setClusterTolerance(0.05); // 2cm
	ec.setMinClusterSize(300);
	ec.setMaxClusterSize(5000000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(mainCloud);
	ec.extract(cluster_indices);
	int j = 0;

	clusterCount = cluster_indices.size();
	//clusteredclouds = new pcl::PointCloud<pcl::PointXYZRGB>::Ptr[clusterCount];
	//clusteredcloudsindices.resize(clusterCount);

	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		std::vector<int> *tmpClusterIndices(new vector<int>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);


		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
			cloud_cluster->points.push_back(mainCloud->points[*pit]); //*
			//clusteredcloudsindices[j].push_back(*pit);
			//tmpClusterIndices->push_back(*pit);
		}
		//clusteredIndices.push_back(*tmpClusterIndices);
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		// std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

		CalculateIndicesForCluster(cloud_cluster);
		clusteredClouds.push_back(cloud_cluster);
		//clusteredclouds[j] = cloud_cluster;

		j++;
	}

	return true;
}

void PCLProcessor::PoissonReconstruction()
{
	/*pcl::Poisson<pcl::PointNormal> poisson;
	poisson.setDepth(9);
	poisson.setInputCloud(cloud_smoothed_normals);
	pcl::PolygonMesh mesh;
	poisson.reconstruct(mesh);*/
}

void PCLProcessor::CalculateIndicesForCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	std::vector<int> newIndices;
	for (int i = 0; i < cloud->points.size(); i++)
	{
		size_t index = -1;
		vector<float> tmp = vertexMap[cloud->points[i].x];
		for (int j = 0; j < tmp.size(); j += 3)
		{
			if (tmp[j + 1] == cloud->points[i].y &&
				tmp[j + 2] == cloud->points[i].z)
			{
				index = (size_t)tmp[j];
				break;
			}
		}
		newIndices.push_back(index);
	}
	clusteredIndices.push_back(newIndices);
}

bool PCLProcessor::ConvertToCloud(std::vector<float> startingVertices, std::vector<GLuint> startingIndices, std::vector<float> startingNormals)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	indexMap.clear();
	clusterIndexCount = 0;
	mainCloud->clear();
	cloud->width = startingVertices.size() / 6;
	cloud->height = 1;
	cloud->is_dense = true;
	cloud->points.resize(cloud->width * cloud->height);

	normals->clear();
	normals->width = startingNormals.size() / 3;
	normals->height = 1;
	normals->is_dense = true;
	normals->points.resize(normals->width * normals->height);
	int index = 0;
	for (int i = 0; i < startingVertices.size(); i += 6)
	{
		cloud->points[index].x = startingVertices[i];
		cloud->points[index].y = startingVertices[i + 1];
		cloud->points[index].z = startingVertices[i + 2];
		cloud->points[index].r = startingVertices[i + 3] * 255;
		cloud->points[index].g = startingVertices[i + 4] * 255;
		cloud->points[index].b = startingVertices[i + 5] * 255;
		//cloud->points[index].a = startingVertices[i + 6];
		index++;

		vertexMap[startingVertices[i]].push_back(i / 6);
		vertexMap[startingVertices[i]].push_back(startingVertices[i + 1]);
		vertexMap[startingVertices[i]].push_back(startingVertices[i + 2]);
	}
	

	index = 0;
	for (int i = 0; i < startingNormals.size(); i += 3)
	{
		normals->points[index].normal_x = startingNormals[i];
		normals->points[index].normal_y = startingNormals[i + 1];
		normals->points[index].normal_z = startingNormals[i + 2];
		index++;
	}

	if (indexMap.empty())
	{
		for (int i = 0; i < startingIndices.size(); i += 3)
		{
			indexMap[startingIndices[i]].push_back(startingIndices[i]);
			indexMap[startingIndices[i]].push_back(startingIndices[i + 1]);
			indexMap[startingIndices[i]].push_back(startingIndices[i + 2]);
			indexMap[startingIndices[i + 1]].push_back(startingIndices[i]);
			indexMap[startingIndices[i + 1]].push_back(startingIndices[i + 1]);
			indexMap[startingIndices[i + 1]].push_back(startingIndices[i + 2]);
			indexMap[startingIndices[i + 2]].push_back(startingIndices[i]);
			indexMap[startingIndices[i + 2]].push_back(startingIndices[i + 1]);
			indexMap[startingIndices[i + 2]].push_back(startingIndices[i + 2]);

			if (i % 100000 == 0)
			{
				int percent = (((float)i / (float)startingIndices.size()) * 100.0f);
				std::wstringstream ws;
				ws << L"Converting mesh to point cloud... ";
				ws << percent;
				ws << L"%";
				wstring statusMsg(ws.str());
				const TCHAR *c_str = statusMsg.c_str();
				SetDlgItemText(openGLWin.glWindowParent, IDC_IM_STATUS, c_str);
			}

		}
	}
	pcl::copyPointCloud(*cloud, *mainCloud);

	return true;
}

bool PCLProcessor::ConvertToTriangleMesh(int clusterIndex, std::vector<float> allVertices, std::vector<float> &vertices, std::vector<GLuint> &indices)
{
	//get clustered cloud at clusterIndex
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster = clusteredClouds[clusterIndex];

	if (cluster->empty()) {
		return false;
	}

	//init clusterCount for progress display
	if (clusterIndexCount == 0)
	{
		procIndexCount = 0;
		for (int i = 0; i < clusteredIndices.size(); i++)
		{
			clusterIndexCount += clusteredIndices[i].size();
		}
	}

	//measure performance
	LARGE_INTEGER frequency;        // ticks per second
	LARGE_INTEGER t1, t2;           // ticks
	double elapsedTime;

	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&t1);

	//vertices are straight-forward and are mapped 1:1
	for (size_t i = 0; i < cluster->points.size(); i++)
	{
		vertices.push_back(cluster->points[i].x);
		vertices.push_back(cluster->points[i].y);
		vertices.push_back(cluster->points[i].z);
		vertices.push_back(cluster->points[i].r / 255.0f);
		vertices.push_back(cluster->points[i].g / 255.0f);
		vertices.push_back(cluster->points[i].b / 255.0f);
	}

	QueryPerformanceCounter(&t2);
	elapsedTime = (t2.QuadPart - t1.QuadPart) * 1000.0 / frequency.QuadPart;
	cDebug::DbgOut(L"Time for vertex copy ", elapsedTime);


	//for indices, we use a vector for iteration AND an unordered_map for retrieval for the best possible performance
	std::vector<int> cloudIndices = clusteredIndices[clusterIndex];
	std::unordered_map<int, int> clusterIndexMap;

	//iterator for find-operations
	std::unordered_map<int, int>::const_iterator mapIterator;

	//fill indexMap with indices
	for (int i = 0; i < clusteredIndices[clusterIndex].size(); i++)
	{
		clusterIndexMap[clusteredIndices[clusterIndex][i]] = i;
	}

	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&t1);

	//in this loop, we try to recreate the appropriate index values for the mesh cluster by using the data collected at ConvertToCloud()
	for (int i = 0; i < cloudIndices.size(); i++)
	{
		vector<int> tmp = indexMap[cloudIndices[i]];
		for (int j = 0; j < tmp.size(); j += 3)
		{
			/*if the index is the first value in the current face, look for 2nd and 3rd in the current cluster 
			and either add them to the new face or add new vertices to fill the face */
			if (tmp[j] == cloudIndices[i])
			{
				//first index of face
				indices.push_back(i);

				//second index of face

				//way faster than vector::find or unordered_set::find for that matter
				mapIterator = clusterIndexMap.find(tmp[j + 1]);

				
				if (mapIterator != clusterIndexMap.end())
				{
					size_t index = clusterIndexMap[tmp[j + 1]];

					indices.push_back(index);
				}
				else
				{
					size_t index = vertices.size() / 6;
					indices.push_back(index);
					vertices.push_back(allVertices[tmp[j + 1] * 6]);
					vertices.push_back(allVertices[(tmp[j + 1] * 6) + 1]);
					vertices.push_back(allVertices[(tmp[j + 1] * 6) + 2]);
					vertices.push_back(allVertices[(tmp[j + 1] * 6) + 3]);
					vertices.push_back(allVertices[(tmp[j + 1] * 6) + 4]);
					vertices.push_back(allVertices[(tmp[j + 1] * 6) + 5]);
				}
				
				//third index of face
				mapIterator = clusterIndexMap.find(tmp[j + 2]);

				if (mapIterator != clusterIndexMap.end())
				{
					size_t index = clusterIndexMap[tmp[j + 2]];

					indices.push_back(index);
				}
				else
				{
					size_t index = vertices.size() / 6;

					indices.push_back(index);

					vertices.push_back(allVertices[tmp[j + 2] * 6]);
					vertices.push_back(allVertices[(tmp[j + 2] * 6) + 1]);
					vertices.push_back(allVertices[(tmp[j + 2] * 6) + 2]);
					vertices.push_back(allVertices[(tmp[j + 2] * 6) + 3]);
					vertices.push_back(allVertices[(tmp[j + 2] * 6) + 4]);
					vertices.push_back(allVertices[(tmp[j + 2] * 6) + 5]);
				}
			}
			else if (tmp[j + 1] == cloudIndices[i])
			{
				//first index of face
				mapIterator = clusterIndexMap.find(tmp[j]);

				if (mapIterator != clusterIndexMap.end())
				{
					size_t index = clusterIndexMap[tmp[j]];
					indices.push_back(index);
				}
				else
				{
					size_t index = vertices.size() / 6;
					indices.push_back(index);
					vertices.push_back(allVertices[tmp[j] * 6]);
					vertices.push_back(allVertices[(tmp[j] * 6) + 1]);
					vertices.push_back(allVertices[(tmp[j] * 6) + 2]);
					vertices.push_back(allVertices[(tmp[j] * 6) + 3]);
					vertices.push_back(allVertices[(tmp[j] * 6) + 4]);
					vertices.push_back(allVertices[(tmp[j] * 6) + 5]);
				}

				//second index of face
				indices.push_back(i);

				//third index of face
				mapIterator = clusterIndexMap.find(tmp[j + 2]);

				if (mapIterator != clusterIndexMap.end())
				{
					size_t index = clusterIndexMap[tmp[j + 2]];
					indices.push_back(index);
				}
				else
				{
					size_t index = vertices.size() / 6;
					indices.push_back(index);
					vertices.push_back(allVertices[tmp[j + 2] * 6]);
					vertices.push_back(allVertices[(tmp[j + 2] * 6) + 1]);
					vertices.push_back(allVertices[(tmp[j + 2] * 6) + 2]);
					vertices.push_back(allVertices[(tmp[j + 2] * 6) + 3]);
					vertices.push_back(allVertices[(tmp[j + 2] * 6) + 4]);
					vertices.push_back(allVertices[(tmp[j + 2] * 6) + 5]);
				}
			}
			else if (tmp[j + 2] == cloudIndices[i])
			{
				//first index of face
				mapIterator = clusterIndexMap.find(tmp[j]);

				if (mapIterator != clusterIndexMap.end())
				{
					size_t index = clusterIndexMap[tmp[j]];
					indices.push_back(index);
				}
				else
				{
					size_t index = vertices.size() / 6;
					indices.push_back(index);
					vertices.push_back(allVertices[tmp[j] * 6]);
					vertices.push_back(allVertices[(tmp[j] * 6) + 1]);
					vertices.push_back(allVertices[(tmp[j] * 6) + 2]);
					vertices.push_back(allVertices[(tmp[j] * 6) + 3]);
					vertices.push_back(allVertices[(tmp[j] * 6) + 4]);
					vertices.push_back(allVertices[(tmp[j] * 6) + 5]);
				}

				//second index of face
				mapIterator = clusterIndexMap.find(tmp[j + 1]);

				if (mapIterator != clusterIndexMap.end())
				{
					size_t index = clusterIndexMap[tmp[j + 1]];
					indices.push_back(index);
				}
				else
				{
					size_t index = vertices.size() / 6;
					indices.push_back(index);
					vertices.push_back(allVertices[tmp[j + 1] * 6]);
					vertices.push_back(allVertices[(tmp[j + 1] * 6) + 1]);
					vertices.push_back(allVertices[(tmp[j + 1] * 6) + 2]);
					vertices.push_back(allVertices[(tmp[j + 1] * 6) + 3]);
					vertices.push_back(allVertices[(tmp[j + 1] * 6) + 4]);
					vertices.push_back(allVertices[(tmp[j + 1] * 6) + 5]);
				}

				//third index of face
				indices.push_back(i);
			}
		}

		//display progress in UI
		procIndexCount++;
		if (procIndexCount % 2000 == 0)
		{
			int percent = (((float)procIndexCount / (float)clusterIndexCount) * 100.0f);
			std::wstringstream ws;
			ws << L"Converting cloud to triangle mesh... ";
			ws << percent;
			ws << L"% (";
			ws << clusterIndex + 1;
			ws << L" of ";
			ws << clusteredClouds.size();
			ws << L")";
			wstring statusMsg(ws.str());
			const TCHAR *c_str = statusMsg.c_str();
			SetDlgItemText(openGLWin.glWindowParent, IDC_IM_STATUS, c_str);
		}

	}

	QueryPerformanceCounter(&t2);
	elapsedTime = (t2.QuadPart - t1.QuadPart) * 1000.0 / frequency.QuadPart;
	cDebug::DbgOut(L"Cluster to mesh with indexSet in ", elapsedTime);

	return true;
}

int PCLProcessor::GetClusterCount() {
	return clusterCount;
}

int PCLProcessor::GetCloudSize() {
	return mainCloud->points.size();
}

bool PCLProcessor::CompareTwoPoints(pcl::PointXYZRGB &ptA, pcl::PointXYZRGB &ptB)
{
	if (ptA.x == ptB.x && ptA.y == ptB.y && ptA.z == ptB.z)
		return true;
	return false;
}