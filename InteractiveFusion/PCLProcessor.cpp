#include "common.h"
#include "PCLProcessor.h"
#include "InteractiveFusion.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include "MeshHelper.h"
#include "SegmentationHelper.h"
//#include <pcl/filters/voxel_grid.h>
//#include "resource.h"
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/surface/mls.h>
#include <unordered_set>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/surface/gp3.h>
#include <pcl/ModelCoefficients.h>
//#include <pcl/io/vtk_lib_io.h>
//#include <pcl/ros/conversions.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/region_growing.h>
//#include <pcl/surface/poisson.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
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
pcl::PointIndices::Ptr planeCloudIndicesConfirmed(new pcl::PointIndices);
pcl::PointIndices::Ptr planeInlierIndices(new pcl::PointIndices);
glm::vec3 minPlane(999.0f, 999.0f, 999.0f);
glm::vec3 maxPlane(-999.0f, -999.0f, -999.0f);

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusteredClouds;
std::vector<std::vector<int>> clusteredIndices;

unordered_map<int, vector<int>> indexMap;
unordered_map<float, vector<float>> vertexMap;


bool PCLProcessor::PlaneSegmentation() {

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inlierPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*mainCloud, *cloud_f);
	pcl::copyPointCloud(*normals, *cloudWithoutPlaneNormals);
	pcl::copyPointCloud(*mainCloud, *cloudWithoutPlane);

	pcl::PointIndices::Ptr allWallSegmentIndices (new pcl::PointIndices);
	pcl::PointIndices::Ptr confirmedWallSegmentIndices(new pcl::PointIndices);
	pcl::PointIndices::Ptr planeSegmentationIndices(new pcl::PointIndices);
	Eigen::Vector3f segAxis(1, 0, 0);
	
	bool redoingSegmentation = false;
	int axisCnt = 0;
	while (true)
	{
		
		//pcl::search::KdTree<pcl::PointXYZRGB>::Ptr	search(new pcl::search::KdTree<pcl::PointXYZRGB>);
		// Object for storing the plane model coefficients.
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		// Create the segmentation object.
		pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> segmentation;
		//search->setInputCloud(cloud_f);
		segmentation.setInputCloud(cloud_f);
		segmentation.setInputNormals(normals);
		segmentation.setProbability(0.99);
		segmentation.setMaxIterations(300);
		if (planeSegmentationIndices->indices.size() > 0)
			segmentation.setIndices(planeSegmentationIndices);
		//segmentation.setSamplesMaxDist(0.005, search);
		segmentation.setAxis(segAxis);
		segmentation.setNormalDistanceWeight(openGLWin.wallSmoothness);
		segmentation.setEpsAngle(10.0f * (M_PI / 180.0f));
		// Configure the object to look for a plane.
		segmentation.setModelType(pcl::SACMODEL_NORMAL_PLANE);// SACMODEL_NORMAL_PLANE);
		// Use RANSAC method.
	
		segmentation.setMethodType(pcl::SAC_RANSAC);
		// Set the maximum allowed distance to the model.
		segmentation.setDistanceThreshold(openGLWin.wallThickness);
		// Enable model coefficient refinement (optional).
		segmentation.setOptimizeCoefficients(true);
		pcl::PointIndices::Ptr inlierIndices(new pcl::PointIndices);
		//inlierIndices->indices.clear();
		segmentation.segment(*inlierIndices, *coefficients);
		
		statusMsg = L"Trying to detect planes";
		if (inlierIndices->indices.size() <= mainCloud->points.size()/20 && !redoingSegmentation)
		{
			if (axisCnt == 0)
			{
				segAxis = Eigen::Vector3f(0, 1, 0);
				axisCnt++;
				continue;
			}
			else if (axisCnt == 1)
			{
				segAxis = Eigen::Vector3f(0, 0, 1);
				axisCnt++;
				continue;
			}
			else if (axisCnt == 2)
			{
			
				cDebug::DbgOut(L"break", 1);
				break;
			}
		}
			


		if (inlierIndices->indices.size() == 0)
			cDebug::DbgOut(L"Could not find any points that fitted the plane model.");
		else
		{
			cDebug::DbgOut(L"coefficient 0 ", coefficients->values[0]);
			cDebug::DbgOut(L"coefficient 0 ", coefficients->values[1]);
			cDebug::DbgOut(L"coefficient 0 ", coefficients->values[2]);
			cDebug::DbgOut(L"coefficient 0 ", coefficients->values[3]);
			
		}
		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>());
		pcl::ExtractIndices<pcl::PointXYZRGB> extract(true);
		extract.setInputCloud(cloud_f);
		extract.setIndices(inlierIndices);
		extract.setNegative(false);
		extract.filter(*inlierPoints);
		pcl::PointIndices::Ptr testIndices(new pcl::PointIndices);
		

		/*boost::shared_ptr<pcl::visualization::PCLVisualizer> PV2(new pcl::visualization::PCLVisualizer("Plane Viewer 2"));
		PV2->setBackgroundColor(0, 0, 0);
		PV2->addPointCloud(inlierPoints, "Plane Windows 2");
		PV2->registerKeyboardCallback(&PCLProcessor::KeyDown, *this);
		PV2->addText("Is this (part of) a floor/wall? (Y/N)", 250, 30, 20,1,1,1);*/
		
		wallSegmentCloud = inlierPoints;
		planeInlierIndices->indices.clear();
		for (int i = 0; i < inlierIndices->indices.size(); i++)
		{
			planeInlierIndices->indices.push_back(inlierIndices->indices[i]);
		}
		float storedThickness = openGLWin.wallThickness;
		float storedSmoothness = openGLWin.wallSmoothness;
		openGLWin.SetWindowState(WALL_SELECTION);
		if (redoingSegmentation)
		{ 
			glSegmentation.ResetInitializedStatus();
			redoingSegmentation = false;
		}
		while (openGLWin.GetWindowState() == WALL_SELECTION)
		{
			if (storedThickness != openGLWin.wallThickness || storedSmoothness != openGLWin.wallSmoothness)
			{
				cDebug::DbgOut(L"stored not like wall");
				//openGLWin.SetWindowState(SEGMENTATION);
				redoingSegmentation = true;
				break;
			}
			//if (!openGLWin.wallSelection)
			//	break;
			//PV2->spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
		
		wallSegmentCloud->clear();
		if (redoingSegmentation)
			continue;
		//closeViewer = false;

		for (int i = 0; i < inlierIndices->indices.size(); i++)
		{
			allWallSegmentIndices->indices.push_back(inlierIndices->indices[i]);
		}
		extract.setIndices(allWallSegmentIndices);
		extract.setNegative(false);
		extract.filter(*cloudWithoutPlane);
		extract.getRemovedIndices(*testIndices);
		extract.setNegative(true);
		extract.filter(*cloudWithoutPlane);
		
		//extract.setNegative(true);
		//extract.filter(*cloud_f);
	
		//pcl::copyPointCloud(*cloud_f, *cloudWithoutPlane);
		planeSegmentationIndices = testIndices;

		if (openGLWin.isWall)
		{
			pcl::PointIndices::Ptr newIndices(new pcl::PointIndices);
			for (int i = 0; i < inlierIndices->indices.size(); i++)
			{
				confirmedWallSegmentIndices->indices.push_back(inlierIndices->indices[i]);
			}

			extract.setIndices(confirmedWallSegmentIndices);
			extract.setNegative(false);
			extract.filter(*cloudWithoutPlane);
			extract.getRemovedIndices(*newIndices);
			extract.setNegative(true);
			extract.filter(*cloudWithoutPlane);

			

			//experimental 
			//delete outliers (vertices behind wall)
			Eigen::Vector4f centroid;
			pcl::compute3DCentroid(*inlierPoints, centroid);
			pcl::PointXYZRGB min;
			pcl::PointXYZRGB max;
			if (axisCnt == 0)
			{
				pcl::getMinMax3D(*inlierPoints, min, max);
				if (centroid.x() < 0)
					minPlane.x = min.x;
				else
					maxPlane.x = max.x;
			}
			else if (axisCnt == 1)
			{
				pcl::getMinMax3D(*inlierPoints, min, max);
				if (centroid.y() < 0)
				{
					minPlane.y = min.y;
				}
				else
					maxPlane.y = max.y;
			}
			else if (axisCnt == 2)
			{
				pcl::getMinMax3D(*inlierPoints, min, max);
				if (centroid.z() < 0)
					minPlane.z = min.z;
				else
					maxPlane.z = max.z;
			}
			cDebug::DbgOut(L"axisCnt", axisCnt);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr testCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::PassThrough<pcl::PointXYZRGB> pass;
			pass.setInputCloud(cloudWithoutPlane);
			pass.setFilterFieldName("y");
			cDebug::DbgOut(L"minPlane y", minPlane.y);
			pass.setFilterLimits(minPlane.y, 999.0);
			//pass.setFilterLimitsNegative (true);
			pass.filter(*testCloud);
			//experimental end

			

			if (cloudWithoutPlaneIndices.size() == 0)
				cloudWithoutPlaneIndices.push_back(newIndices);
			else
				cloudWithoutPlaneIndices[0] = newIndices;

			planeCoefficients.push_back(coefficients);
			planeCloudIndices.push_back(inlierIndices);

			
		}


		//extract.setNegative(true);
	
		

		pcl::ExtractIndices<pcl::Normal> extractNormals;
		extractNormals.setInputCloud(cloudWithoutPlaneNormals);
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
		}*/
		

	}
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
	for (std::vector<pcl::PointIndices::Ptr>::const_iterator it = planeCloudIndices.begin(); it != planeCloudIndices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
		//std::vector<int> *tmpClusterIndices(new vector<int>);

		for (std::vector<int>::const_iterator pit = (*it)->indices.begin(); pit != (*it)->indices.end(); pit++) {
			cloud_cluster->points.push_back(mainCloud->points[*pit]); 

		//	if (j == 0)
		//		tmpClusterIndices->push_back(*pit);
		}
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		// std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
		
		//if (j == 0)
		//	clusteredIndices.push_back(*tmpClusterIndices);
		//else
			clusteredIndices.push_back(CalculateIndicesForCluster(cloud_cluster));
		
			
		clusteredClouds.push_back(cloud_cluster);

		j++;
	}
}

void PCLProcessor::ConfirmPlaneIndices(int index)
{
	for (int i = 0; i < planeCloudIndices[index]->indices.size(); i++)
	{
		planeCloudIndicesConfirmed->indices.push_back(planeCloudIndices[index]->indices[i]);
	}
}

void PCLProcessor::PrepareForObjectSegmentation()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr tempNormals(new pcl::PointCloud <pcl::Normal>);

	pcl::copyPointCloud(*normals, *tempNormals);

	pcl::ExtractIndices<pcl::PointXYZRGB> extract(true);
	extract.setInputCloud(mainCloud);
	extract.setIndices(planeCloudIndicesConfirmed);
	extract.setNegative(false);
	extract.filter(*tempCloud);
	extract.getRemovedIndices(*cloudWithoutPlaneIndices[0]);
	
	cDebug::DbgOut(L"size: ", (int)planeCloudIndicesConfirmed->indices.size());
	cDebug::DbgOut(L"size: ", (int)mainCloud->points.size());
	/*boost::shared_ptr<pcl::visualization::PCLVisualizer> PV(new pcl::visualization::PCLVisualizer("Plane Viewer"));
	PV->setBackgroundColor(0, 0, 0);
	PV->addPointCloud(tempCloud, "Plane Windows");
	while (!PV->wasStopped())
	{
		PV->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}*/

	cloudWithoutPlaneNormals->clear();
	cloudWithoutPlaneNormals->points.resize(cloudWithoutPlaneIndices[0]->indices.size());
	for (int i = 0; i < cloudWithoutPlaneIndices[0]->indices.size(); i++)
	{
		cloudWithoutPlaneNormals->points[i] = tempNormals->points[cloudWithoutPlaneIndices[0]->indices[i]];
	}
	planeCloudIndicesConfirmed->indices.clear();
	
}
void PCLProcessor::KeyDown(const pcl::visualization::KeyboardEvent &keyEvent, void* viewer_void) {
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
	if (keyEvent.keyDown()){
		if (keyEvent.getKeySym() == "y") {
			isWall = true;
			closeViewer = true;
		}
		else if (keyEvent.getKeySym() == "n") {
			closeViewer = true;
			isWall = false;
			//cDebug::DbgOut(L"kSearch value down to: ", kSearchValue);
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
		//std::vector<int> *tmpClusterIndices(new vector<int>);

		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
			cloud_cluster->points.push_back(mainCloud->points[*pit]); //*
			//clusteredcloudsindices[j].push_back(*pit);
			//if (j == 0)
			//	tmpClusterIndices->push_back(*pit);
		}
		//clusteredIndices.push_back(*tmpClusterIndices);
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		// std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
		//if (j == 0)
		//	clusteredIndices.push_back(*tmpClusterIndices);
		//else
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
		openGLWin.ShowStatusBarMessage(L"Estimating normals..");
		SetViewportStatusMessage(L"Estimating normals");

		QueryPerformanceFrequency(&frequency);
		QueryPerformanceCounter(&t1);
		QueryPerformanceCounter(&t2);
		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
		normal_estimator.setSearchMethod(tree);
		//if (cloudWithoutPlane->empty())
			normal_estimator.setInputCloud(mainCloud);
		//else
		//	normal_estimator.setInputCloud(cloudWithoutPlane);
			if (cloudWithoutPlaneIndices.size() > 0)
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

	openGLWin.ShowStatusBarMessage(L"Segmenting mesh...");
	SetViewportStatusMessage(L"Segmenting mesh");

	pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
	reg.setMinClusterSize(openGLWin.minClusterSize);
	reg.setMaxClusterSize(openGLWin.maxClusterSize * 1000);
	reg.setSearchMethod(tree);
	reg.setResidualTestFlag(true);
	reg.setNumberOfNeighbours(openGLWin.numberOfNeighbors);
	reg.setInputCloud(mainCloud);
	if (cloudWithoutPlaneIndices.size() > 0)
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

bool
EnforceCurvatureOrColor(const pcl::PointXYZRGBNormal& point_a, const pcl::PointXYZRGBNormal& point_b, float squared_distance)
{
	Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.normal, point_b_normal = point_b.normal;
	if ((abs(point_a.r - point_b.r) < 10 && abs(point_a.g - point_b.g) < 10 && abs(point_a.b - point_b.b) < 10)
		|| fabs(point_a_normal.dot(point_b_normal)) < 0.05)
		return (true);
	return (false);
}

bool PCLProcessor::EuclideanSegmentation() {
	pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> >(new pcl::search::KdTree<pcl::PointXYZRGB>);

	LARGE_INTEGER frequency;        // ticks per second
	LARGE_INTEGER t1, t2;           // ticks
	double elapsedTime;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mainCloudWithNormals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::copyPointCloud(*mainCloud, *mainCloudWithNormals);
	for (int i = 0; i < mainCloudWithNormals->points.size(); i++)
	{
		mainCloudWithNormals->points[i].normal_x = normals->points[i].normal_x;
		mainCloudWithNormals->points[i].normal_y = normals->points[i].normal_y;
		mainCloudWithNormals->points[i].normal_z = normals->points[i].normal_z;
	}
	pcl::PointCloud <pcl::Normal>::Ptr estNormals(new pcl::PointCloud <pcl::Normal>);
	if (openGLWin.estimateNormals)
	{
		openGLWin.ShowStatusBarMessage(L"Estimating normals...");
		SetViewportStatusMessage(L"Estimating normals");

		QueryPerformanceFrequency(&frequency);
		QueryPerformanceCounter(&t1);
		QueryPerformanceCounter(&t2);
		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
		normal_estimator.setSearchMethod(tree);
		//if (cloudWithoutPlane->empty())
		normal_estimator.setInputCloud(mainCloud);
		//else
		//	normal_estimator.setInputCloud(cloudWithoutPlane);
		if (cloudWithoutPlaneIndices.size() > 0)
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

	SetViewportStatusMessage(L"Segmenting mesh");
	openGLWin.ShowStatusBarMessage(L"Processing segmentation...");

	//pcl::ConditionalEuclideanClustering<pcl::PointXYZRGBNormal> ec;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	//ec.setConditionFunction(&EnforceCurvatureOrColor);
	
	
	if (openGLWin.meshQuality == QUALITY_MEDIUM || openGLWin.meshQuality == QUALITY_HIGH || openGLWin.meshQuality == QUALITY_VERYHIGH)
		ec.setClusterTolerance(0.02); // 2cm
	if (openGLWin.meshQuality == QUALITY_LOW || openGLWin.meshQuality == QUALITY_VERYLOW)
		ec.setClusterTolerance(0.04); // 2cm

	ec.setClusterTolerance(openGLWin.clusterTolerance);

	if (openGLWin.meshQuality == QUALITY_HIGH || openGLWin.meshQuality == QUALITY_VERYHIGH)
		ec.setMinClusterSize(1000);
	else if (openGLWin.meshQuality == QUALITY_MEDIUM || openGLWin.meshQuality == QUALITY_LOW)
		ec.setMinClusterSize(100);
	else
		ec.setMinClusterSize(50);

	ec.setMaxClusterSize(9000000);
	//ec.setSearchMethod(tree);
	ec.setInputCloud(mainCloud);
	if (cloudWithoutPlaneIndices.size() > 0)
		ec.setIndices(cloudWithoutPlaneIndices[0]);
	ec.extract(segmentedClusterIndices);

	clusterCount = segmentedClusterIndices.size() + planeCloudIndices.size();
	QueryPerformanceCounter(&t2);
	elapsedTime = (t2.QuadPart - t1.QuadPart) * 1000.0 / frequency.QuadPart;
	cDebug::DbgOut(L"Euclidean clustering in ", elapsedTime);
	
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

bool PCLProcessor::ConvertToCloud(std::vector<Vertex> startingVertices, std::vector<Triangle> startingIndices)
{
	SetViewportStatusMessage(L"Converting mesh to point cloud");

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	indexMap.clear();
	clusterIndexCount = 0;
	mainCloud->clear();
	cloud->width = startingVertices.size();
	cloud->height = 1;
	cloud->is_dense = true;
	cloud->points.resize(cloud->width * cloud->height);

	normals->clear();
	normals->width = startingVertices.size();
	normals->height = 1;
	normals->is_dense = true;
	normals->points.resize(normals->width * normals->height);
	for (int i = 0; i < startingVertices.size(); i += 1)
	{
		cloud->points[i].x = startingVertices[i].x;
		cloud->points[i].y = startingVertices[i].y;
		cloud->points[i].z = startingVertices[i].z;
		cloud->points[i].r = startingVertices[i].r * 255;
		cloud->points[i].g = startingVertices[i].g * 255;
		cloud->points[i].b = startingVertices[i].b * 255;
		normals->points[i].normal_x = startingVertices[i].normal_x;
		normals->points[i].normal_y = startingVertices[i].normal_y;
		normals->points[i].normal_z = startingVertices[i].normal_z;
		//cloud->points[index].a = startingVertices[i + 6];

		vertexMap[startingVertices[i].x].push_back(i);
		vertexMap[startingVertices[i].x].push_back(startingVertices[i].y);
		vertexMap[startingVertices[i].x].push_back(startingVertices[i].z);
	}

	if (indexMap.empty())
	{
		for (int i = 0; i < startingIndices.size(); i += 1)
		{
			indexMap[startingIndices[i].v1].push_back(startingIndices[i].v1);
			indexMap[startingIndices[i].v1].push_back(startingIndices[i].v2);
			indexMap[startingIndices[i].v1].push_back(startingIndices[i].v3);
			indexMap[startingIndices[i].v2].push_back(startingIndices[i].v1);
			indexMap[startingIndices[i].v2].push_back(startingIndices[i].v2);
			indexMap[startingIndices[i].v2].push_back(startingIndices[i].v3);
			indexMap[startingIndices[i].v3].push_back(startingIndices[i].v1);
			indexMap[startingIndices[i].v3].push_back(startingIndices[i].v2);
			indexMap[startingIndices[i].v3].push_back(startingIndices[i].v3);

			if (i % 100000 == 0)
			{
				int percent = (((float)i / (float)startingIndices.size()) * 100.0f);
				//openGLWin.ShowStatusBarMessage(L"Converting mesh to point cloud... " + to_wstring(percent) + L"%");
				SetViewportPercentMsg(to_wstring(percent) + L"% (1 of 1)");
			}

		}
	}
	pcl::copyPointCloud(*cloud, *mainCloud);

	SetViewportPercentMsg(L"");
	return true;
}

bool PCLProcessor::ConvertToTriangleMesh(int clusterIndex, std::vector<Vertex> allVertices, std::vector<Vertex> &vertices, std::vector<Triangle> &indices, bool isPlane)
{
	//get clustered cloud at clusterIndex
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster = clusteredClouds[clusterIndex];

	if (cluster->empty()) {
		return false;
	}
	clusterIndexCount = 0;
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

	Vertex centerVertex{ 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	//vertices are straight-forward and are mapped 1:1
	for (size_t i = 0; i < cluster->points.size(); i++)
	{
		Vertex vertex;
		vertex.x = cluster->points[i].x;
		vertex.y = cluster->points[i].y;
		vertex.z = cluster->points[i].z;
		vertex.r = cluster->points[i].r / 255.0f;
		vertex.g = cluster->points[i].g / 255.0f;
		vertex.b = cluster->points[i].b / 255.0f;
		vertices.push_back(vertex);

		centerVertex.x += vertex.x;
		centerVertex.y += vertex.y;
		centerVertex.z += vertex.z;
	}
	centerVertex.x = centerVertex.x / (float)cluster->points.size();
	centerVertex.y = centerVertex.y / (float)cluster->points.size();
	centerVertex.z = centerVertex.z / (float)cluster->points.size();

	// THIS IS FOR WALL & FLOOR CROPPING
	cDebug::DbgOut(L"clusterIndex:", clusterIndex);
	cDebug::DbgOut(L"planeCoefficients size:", (int)planeCoefficients.size());
	/*if (!isPlane)
	{
		glm::vec3 overallCenterPoint = meshHelper.GetCombinedCenterPoint();
		for (int i = 0; i < planeCoefficients.size(); i++)
		{
			float iO = centerVertex.x*planeCoefficients[0]->values[0] + centerVertex.y*planeCoefficients[0]->values[1] + centerVertex.z * planeCoefficients[0]->values[2] + planeCoefficients[0]->values[3];
			cDebug::DbgOut(L"mesh #" + to_wstring(clusterIndex) + L" iO: ", iO);

			float max = -99999.0f;
			int pIndex = -1;
			for (int j = 0; j < 3; j++)
			{
				if (abs(planeCoefficients[i]->values[j]) > max)
				{
					max = abs(planeCoefficients[i]->values[j]);
					pIndex = j;
				}
			}
			cDebug::DbgOut(L"max dimension: ", pIndex);
			int dimension;
			cDebug::DbgOut(L"clusterIndex maybe false?", clusterIndex);
			if (planeCoefficients[i]->values[pIndex] > 0)
			{
				if (iO < 0.0f)
					return false;
			}
			else if (iO > 0.0f)
				return false;
				
		}
		
	}*/
	//END FOR CROPPING

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
	SetViewportStatusMessage(L"Converting cloud to triangle mesh");

	//in this loop, we try to recreate the appropriate index values for the mesh cluster by using the data collected at ConvertToCloud()
	for (int i = 0; i < cloudIndices.size(); i++)
	{
		vector<int> tmp = indexMap[cloudIndices[i]];
		for (int j = 0; j < tmp.size(); j += 3)
		{
			Triangle triangle;
			/*if the index is the first value in the current face, look for 2nd and 3rd in the current cluster 
			and either add them to the new face or add new vertices to fill the face */
			if (tmp[j] == cloudIndices[i])
			{
				//first index of face
				triangle.v1 = i;
				
				//indices.push_back(i);

				//second index of face

				//way faster than vector::find or unordered_set::find for that matter
				mapIterator = clusterIndexMap.find(tmp[j + 1]);

				
				if (mapIterator != clusterIndexMap.end())
				{
					size_t index = clusterIndexMap[tmp[j + 1]];

					triangle.v2 = index;
					//indices.push_back(index);
				}
				else
				{
					size_t index = vertices.size();
					triangle.v2 = index;
					//indices.push_back(index);
					Vertex vertex;
					vertex.x = allVertices[tmp[j + 1]].x;
					vertex.y = allVertices[tmp[j + 1]].y;
					vertex.z = allVertices[tmp[j + 1]].z;
					vertex.r = allVertices[tmp[j + 1]].r;
					vertex.g = allVertices[tmp[j + 1]].g;
					vertex.b = allVertices[tmp[j + 1]].b;
					vertices.push_back(vertex);
				}
				
				//third index of face
				mapIterator = clusterIndexMap.find(tmp[j + 2]);

				if (mapIterator != clusterIndexMap.end())
				{
					size_t index = clusterIndexMap[tmp[j + 2]];

					triangle.v3 = index;
					//indices.push_back(index);
				}
				else
				{
					size_t index = vertices.size();

					//indices.push_back(index);
					triangle.v3 = index;
					Vertex vertex;
					vertex.x = allVertices[tmp[j + 2]].x;
					vertex.y = allVertices[tmp[j + 2]].y;
					vertex.z = allVertices[tmp[j + 2]].z;
					vertex.r = allVertices[tmp[j + 2]].r;
					vertex.g = allVertices[tmp[j + 2]].g;
					vertex.b = allVertices[tmp[j + 2]].b;
					vertices.push_back(vertex);
				}
				indices.push_back(triangle);
			}
			else if (tmp[j + 1] == cloudIndices[i])
			{
				//first index of face
				mapIterator = clusterIndexMap.find(tmp[j]);

				if (mapIterator != clusterIndexMap.end())
				{
					size_t index = clusterIndexMap[tmp[j]];
					triangle.v1 = index;
					//indices.push_back(index);
				}
				else
				{
					size_t index = vertices.size();
					triangle.v1 = index;
					//indices.push_back(index);
					Vertex vertex;
					vertex.x = allVertices[tmp[j]].x;
					vertex.y = allVertices[tmp[j]].y;
					vertex.z = allVertices[tmp[j]].z;
					vertex.r = allVertices[tmp[j]].r;
					vertex.g = allVertices[tmp[j]].g;
					vertex.b = allVertices[tmp[j]].b;
					vertices.push_back(vertex);
				}

				//second index of face
				//indices.push_back(i);
				triangle.v2 = i;

				//third index of face
				mapIterator = clusterIndexMap.find(tmp[j + 2]);

				if (mapIterator != clusterIndexMap.end())
				{
					size_t index = clusterIndexMap[tmp[j + 2]];
					triangle.v3 = index;
					//indices.push_back(index);
				}
				else
				{
					size_t index = vertices.size();
					triangle.v3 = index;
					//indices.push_back(index);
					Vertex vertex;
					vertex.x = allVertices[tmp[j + 2]].x;
					vertex.y = allVertices[tmp[j + 2]].y;
					vertex.z = allVertices[tmp[j + 2]].z;
					vertex.r = allVertices[tmp[j + 2]].r;
					vertex.g = allVertices[tmp[j + 2]].g;
					vertex.b = allVertices[tmp[j + 2]].b;
					vertices.push_back(vertex);
				}
				indices.push_back(triangle);
			}
			else if (tmp[j + 2] == cloudIndices[i])
			{
				//first index of face
				mapIterator = clusterIndexMap.find(tmp[j]);

				if (mapIterator != clusterIndexMap.end())
				{
					size_t index = clusterIndexMap[tmp[j]];
					triangle.v1 = index;
					//indices.push_back(index);
				}
				else
				{
					size_t index = vertices.size();
					triangle.v1 = index;
					//indices.push_back(index);
					Vertex vertex;
					vertex.x = allVertices[tmp[j]].x;
					vertex.y = allVertices[tmp[j]].y;
					vertex.z = allVertices[tmp[j]].z;
					vertex.r = allVertices[tmp[j]].r;
					vertex.g = allVertices[tmp[j]].g;
					vertex.b = allVertices[tmp[j]].b;
					vertices.push_back(vertex);
				}

				//second index of face
				mapIterator = clusterIndexMap.find(tmp[j + 1]);

				if (mapIterator != clusterIndexMap.end())
				{
					size_t index = clusterIndexMap[tmp[j + 1]];
					triangle.v2 = index;
					//indices.push_back(index);
				}
				else
				{
					size_t index = vertices.size();
					//indices.push_back(index);
					triangle.v2 = index;
					Vertex vertex;
					vertex.x = allVertices[tmp[j + 1]].x;
					vertex.y = allVertices[tmp[j + 1]].y;
					vertex.z = allVertices[tmp[j + 1]].z;
					vertex.r = allVertices[tmp[j + 1]].r;
					vertex.g = allVertices[tmp[j + 1]].g;
					vertex.b = allVertices[tmp[j + 1]].b;
					vertices.push_back(vertex);
				}

				//third index of face
				triangle.v3 = i;
				//indices.push_back(i);
				indices.push_back(triangle);
			}
		}

		//display progress in UI
		procIndexCount++;
		if (procIndexCount % 2000 == 0)
		{
			int percent = (((float)procIndexCount / (float)clusterIndexCount) * 100.0f);
			
			openGLWin.ShowStatusBarMessage(L"Converting cloud to triangle mesh ( " + to_wstring(percent) + L"% " + to_wstring(clusterIndex + 1) + L" of " + to_wstring(clusteredClouds.size()) + L")");

			//SetViewportStatusMessage(L"Converting cloud to triangle mesh ( " + to_wstring(percent) + L"% " + to_wstring(clusterIndex + 1) + L" of " + to_wstring(clusteredClouds.size()) + L")");
			SetViewportPercentMsg(to_wstring(percent) + L"% (" + to_wstring(clusterIndex + 1) + L" of " + to_wstring(clusteredClouds.size()) + L")");
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

std::vector<int> PCLProcessor::GetInlierIndices()
{
	std::vector<int> triangles;
	for (int i = 0; i < planeInlierIndices->indices.size(); i++)
	{
		triangles.push_back(planeInlierIndices->indices[i]);
	}
	return triangles;
}

std::vector<int> PCLProcessor::GetPlaneCloudIndices(int index)
{
	std::vector<int> triangles;
	for (int i = 0; i < planeCloudIndices[index]->indices.size(); i++)
	{
		triangles.push_back(planeCloudIndices[index]->indices[i]);
	}
	return triangles;
}

std::vector<int> PCLProcessor::GetColoredCloudIndices(int index)
{
	std::vector<int> triangles;
	for (int i = 0; i < segmentedClusterIndices[index].indices.size(); i ++)
	{
		triangles.push_back(segmentedClusterIndices[index].indices[i]);
	}
	return triangles;
}

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

void PCLProcessor::ClearAll() {
	
	mainCloud->clear();
	normals->clear();
	cloudWithoutPlane->clear();
	cloudWithoutPlaneNormals->clear();
	cloudWithoutPlaneIndices.clear();
	clusteredClouds.clear();
	clusteredIndices.clear();
	indexMap.clear();
	vertexMap.clear();
	minPlane = glm::vec3(999.0f, 999.0f, 999.0f);
	maxPlane = glm::vec3(-999.0f, -999.0f, -999.0f);

	clusterCount = 0;
	clusterIndexCount = 0;
	procIndexCount = 0;
	segmentedClusterIndices.clear();
	planeCoefficients.clear();
	planeCloudIndices.clear();

	isPlaneSegmented = false;
	coloredCloudReady = false;
	isWall = false;
}

bool PCLProcessor::CompareTwoPoints(pcl::PointXYZRGB &ptA, pcl::PointXYZRGB &ptB)
{
	if (ptA.x == ptB.x && ptA.y == ptB.y && ptA.z == ptB.z)
		return true;
	return false;
}