#include "common.h"
#include "PCLProcessing.h"
#include "openGLWin.h"
#include <pcl/filters/extract_indices.h>
//#include <pcl/filters/voxel_grid.h>
#include "resource.h"
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/surface/mls.h>
#include <unordered_set>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/surface/gp3.h>
#include <pcl/ModelCoefficients.h>
//#include <pcl/io/vtk_lib_io.h>
//#include <pcl/ros/conversions.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/region_growing.h>
//#include <pcl/surface/poisson.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/surface/marching_cubes_rbf.h>
//#include <pcl/surface/marching_cubes_hoppe.h>
//#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_windowed_sinc.h>
//#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>


pcl::PointCloud<pcl::PointXYZRGB>::Ptr mainCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudWithoutPlane(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::Normal>::Ptr cloudWithoutPlaneNormals(new pcl::PointCloud <pcl::Normal>);
std::vector<pcl::PointIndices::Ptr > cloudWithoutPlaneIndices;

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusteredClouds;
std::vector<std::vector<int>> clusteredIndices;

unordered_map<int, vector<int>> indexMap;
unordered_map<float, vector<float>> vertexMap;


bool PCLProcessor::PlaneSegmentation() {
	cDebug::DbgOut(L"PLane segmentation! ", 0);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inlierPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*mainCloud, *cloud_f);

	// Object for storing the plane model coefficients.
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	// Create the segmentation object.
	pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> segmentation;
	segmentation.setInputCloud(cloud_f);
	segmentation.setInputNormals(normals);
	segmentation.setNormalDistanceWeight(0.02);
	segmentation.setEpsAngle(0.09);
	// Configure the object to look for a plane.
	segmentation.setModelType(pcl::SACMODEL_NORMAL_PLANE);
	// Use RANSAC method.
	
	segmentation.setMethodType(pcl::SAC_RANSAC);
	// Set the maximum allowed distance to the model.
	segmentation.setDistanceThreshold(0.05);
	// Enable model coefficient refinement (optional).
	segmentation.setOptimizeCoefficients(true);

	pcl::PointIndices::Ptr inlierIndices(new pcl::PointIndices);
	segmentation.segment(*inlierIndices, *coefficients);



	if (inlierIndices->indices.size() == 0)
		cDebug::DbgOut(L"Could not find any points that fitted the plane model.");
	else
	{
		std::cerr << "Model coefficients: " << coefficients->values[0] << " "
			<< coefficients->values[1] << " "
			<< coefficients->values[2] << " "
			<< coefficients->values[3] << std::endl;

		// Copy all inliers of the model to another cloud.
		//pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud_f, *inlierIndices, *inlierPoints);
	}
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::ExtractIndices<pcl::PointXYZRGB> extract(true);
	extract.setInputCloud(cloud_f);
	extract.setIndices(inlierIndices);
	extract.setNegative(false);
	extract.filter(*inlierPoints);
	pcl::PointIndices::Ptr testIndices(new pcl::PointIndices);
	extract.getRemovedIndices(*testIndices);
	extract.setIndices(testIndices);
	extract.filter(*cloud_f);
	//extract.setNegative(true);
	//extract.filter(*cloud_f);
	
	pcl::copyPointCloud(*cloud_f, *cloudWithoutPlane);

	cloudWithoutPlaneIndices.push_back(testIndices);

	//extract.setNegative(true);
	
	planeCloudIndices.push_back(*inlierIndices);

	pcl::ExtractIndices<pcl::Normal> extractNormals;
	extractNormals.setInputCloud(normals);
	extractNormals.setIndices(inlierIndices);
	extractNormals.setNegative(true);
	extractNormals.filter(*cloudWithoutPlaneNormals);
	/*boost::shared_ptr<pcl::visualization::PCLVisualizer> PV(new pcl::visualization::PCLVisualizer("Plane Viewer"));
	PV->setBackgroundColor(0, 0, 0);
	PV->addPointCloud(cloudWithoutPlane, "Plane Windows");
	while (!PV->wasStopped())
	{
		PV->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	boost::shared_ptr<pcl::visualization::PCLVisualizer> PV2(new pcl::visualization::PCLVisualizer("Plane Viewer 2"));
	PV2->setBackgroundColor(0, 0, 0);
	PV2->addPointCloud(inlierPoints, "Plane Windows 2");
	while (!PV2->wasStopped())
	{
		PV2->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}*/

	isPlaneSegmented = true;


	/*
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

	cDebug::DbgOut(L"Number of planes: ", numberOfElements);*/

	return true;
}

void PCLProcessor::PlaneIndexEstimation()
{
	clusterCount = planeCloudIndices.size();
	clusteredClouds.clear();
	clusteredIndices.clear();
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = planeCloudIndices.begin(); it != planeCloudIndices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
		std::vector<int> *tmpClusterIndices(new vector<int>);

		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
			cloud_cluster->points.push_back(mainCloud->points[*pit]); 

			if (j == 0)
				tmpClusterIndices->push_back(*pit);
		}
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		// std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
		if (j == 0)
			clusteredIndices.push_back(*tmpClusterIndices);
		//else
			clusteredIndices.push_back(CalculateIndicesForCluster(cloud_cluster));
		
			
		clusteredClouds.push_back(cloud_cluster);

		j++;
	}
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

/*
void PCLProcessor::ShowViewer() {
	pcl::visualization::CloudViewer viewer("Cluster viewer");
	viewer.showCloud(coloredSegmentedCloud);
	viewer.runOnVisualizationThreadOnce(ViewerInit);
	viewer.runOnVisualizationThread(ViewerCallback);

	while (!viewer.wasStopped())
	{

	}
}
*/

void PCLProcessor::IndexEstimation()
{
	LARGE_INTEGER frequency;        // ticks per second
	LARGE_INTEGER t1, t2;           // ticks
	double elapsedTime;
	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&t1);
	QueryPerformanceCounter(&t2);

	int j = 0;

	clusterCount = segmentedClusterIndices.size() + planeCloudIndices.size();

	clusteredClouds.clear();
	clusteredIndices.clear();
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
			clusteredIndices.push_back(CalculateIndicesForCluster(cloud_cluster));
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
		//if (cloudWithoutPlane->empty())
			normal_estimator.setInputCloud(mainCloud);
		//else
		//	normal_estimator.setInputCloud(cloudWithoutPlane);
			if (!cloudWithoutPlane->empty())
				normal_estimator.setIndices(cloudWithoutPlaneIndices[0]);
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
	if (!cloudWithoutPlane->empty())
		reg.setIndices(cloudWithoutPlaneIndices[0]);
	if (!cloudWithoutPlane->empty())
		cDebug::DbgOut(L"NOT EMPTY ", (int)cloudWithoutPlaneIndices.size());
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> PV(new pcl::visualization::PCLVisualizer("Normals Viewer"));
	//PV->setBackgroundColor(0, 0, 0);
	//PV->addPointCloud(mainCloud, "Normals Windows");

	if (openGLWin.estimateNormals)
	{
		reg.setInputNormals(estNormals);
		//PV->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(mainCloud, estNormals, 10, 0.05, "Normals");
	}
	else
	{
		//if (cloudWithoutPlane->empty())
			reg.setInputNormals(normals);
		//else
		//	reg.setInputNormals(cloudWithoutPlaneNormals);
		//PV->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(mainCloud, normals, 10, 0.05, "Normals");
	}

	//while (!PV->wasStopped())
	//{
	//	PV->spinOnce(100);
	//	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	//}
	reg.setSmoothnessThreshold((openGLWin.smoothnessThreshold / 10.0) / 180.0 * M_PI);
	
	reg.setCurvatureThreshold((openGLWin.curvatureThreshold / 10.0));
	cDebug::DbgOut(L"Smoothness: " + std::to_wstring(reg.getSmoothnessThreshold()));
	cDebug::DbgOut(L"Curvature: " + std::to_wstring(reg.getCurvatureThreshold()));
	reg.extract(segmentedClusterIndices);
	clusterCount = segmentedClusterIndices.size() + planeCloudIndices.size();
	QueryPerformanceCounter(&t2);
	elapsedTime = (t2.QuadPart - t1.QuadPart) * 1000.0 / frequency.QuadPart;
	cDebug::DbgOut(L"Region based clustering in ", elapsedTime);

	//pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
	coloredSegmentedCloud = reg.getColoredCloud();
	cDebug::DbgOut(L"COLORED CLOUD ", (int)coloredSegmentedCloud->points.size());
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

std::vector<int> PCLProcessor::CalculateIndicesForCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
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
	return newIndices;
	
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
	//mainCloud->clear();
	return true;
}



bool PCLProcessor::IsMainCloudInitialized()
{
	return !mainCloud->empty();
}

bool PCLProcessor::IsPlaneSegmented()
{
	return isPlaneSegmented;
}

/*void PCLProcessor::MovingLeastSquares(std::vector<float> vertices, std::vector<float> normals, std::vector<float> &outputVertices, std::vector<GLuint> &outputIndices, std::vector<float> &outputNormals)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud->width = vertices.size() / 6;
	cloud->height = 1;
	cloud->is_dense = true;
	cloud->points.resize(cloud->width * cloud->height);

	int index = 0;
	for (int i = 0; i < vertices.size(); i += 6)
	{
		cloud->points[index].x = vertices[i];
		cloud->points[index].y = vertices[i + 1];
		cloud->points[index].z = vertices[i + 2];
		cloud->points[index].r = vertices[i + 3] * 255;
		cloud->points[index].g = vertices[i + 4] * 255;
		cloud->points[index].b = vertices[i + 5] * 255;
		//cloud->points[index].normal_x = normals[i];
		//cloud->points[index].normal_y = normals[i+1];
		//cloud->points[index].normal_z = normals[i+2];
		//cloud->points[index].a = startingVertices[i + 6];
		index++;
	}

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGBNormal> mls_points;
	pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> mls;

	mls.setComputeNormals(true);
	mls.setInputCloud(cloud);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(0.05);
	mls.setPolynomialOrder(2);
	mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal>::SAMPLE_LOCAL_PLANE);
	mls.setUpsamplingRadius(0.005);
	mls.setUpsamplingStepSize(0.003);
	mls.process(mls_points);

	pcl::visualization::CloudViewer viewer("Cloud Viewer");

	//blocks until the cloud is actually rendered
	

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mlsCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr viewCloud (new pcl::PointCloud<pcl::PointXYZRGB>);


	pcl::copyPointCloud(mls_points, *mlsCloud);
	pcl::copyPointCloud(mls_points, *viewCloud);
	viewer.showCloud(viewCloud);

	//use the following functions to get access to the underlying more advanced/powerful
	//PCLVisualizer

	//This will only get called once

	//This will get called once per visualization iteration
	while (!viewer.wasStopped())
	{
		//you can also do cool processing here
		//FIXME: Note that this is running in a separate thread from viewerPsycho
		//and you should guard against race conditions yourself...
	}
	//greedy triangulation
	// Create search tree*
	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
	tree2->setInputCloud(mlsCloud);

	//pcl::MarchingCubesRBF<pcl::PointXYZRGBNormal> mc;
	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
	pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);


	pcl::MarchingCubesRBF<pcl::PointXYZRGBNormal> rbf;
	rbf.setIsoLevel(0);
	rbf.setGridResolution(20, 20, 20);
	rbf.setPercentageExtendGrid(0.1f);
	rbf.setInputCloud(mlsCloud);
	rbf.setOffSurfaceDisplacement(0.02f);
	rbf.reconstruct(*triangles);

	

	//mc.setIsoLevel(0.001);
	//mc.setInputCloud(mlsCloud);
	//mc.setSearchMethod(tree2);
	//mc.reconstruct(*triangles);

	pcl::PointCloud<pcl::PointXYZRGBNormal> newCloud;
	pcl::fromROSMsg(triangles->cloud, newCloud);
	for (int i = 0; i < newCloud.points.size(); i++)
	{
		outputVertices.push_back(newCloud.points[i].x);
		outputVertices.push_back(newCloud.points[i].y);
		outputVertices.push_back(newCloud.points[i].z);
		outputVertices.push_back(newCloud.points[i].r / 255.0f);
		outputVertices.push_back(newCloud.points[i].g / 255.0f);
		outputVertices.push_back(newCloud.points[i].b / 255.0f);
		outputNormals.push_back(newCloud.points[i].normal_x);
		outputNormals.push_back(newCloud.points[i].normal_y);
		outputNormals.push_back(newCloud.points[i].normal_z);
	}

	for (int i = 0; i < triangles->polygons.size(); i++)
	{
		outputIndices.push_back((GLuint)(triangles->polygons[i].vertices[0]));
		outputIndices.push_back((GLuint)(triangles->polygons[i].vertices[1]));
		outputIndices.push_back((GLuint)(triangles->polygons[i].vertices[2]));
	}

	
	//pcl::copyPointCloud(*cloud_with_normals, *mainCloud);
}*/

int PCLProcessor::GetClusterCount() {
	//return clusterCount;
	return segmentedClusterIndices.size() + planeCloudIndices.size();
}

int PCLProcessor::GetRegionClusterCount() {
	return segmentedClusterIndices.size();
}

int PCLProcessor::GetPlaneClusterCount() {
	return planeCloudIndices.size();
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