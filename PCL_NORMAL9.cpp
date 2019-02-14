#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string.h>


int main(int argc, char** argv)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if ( pcl::io::loadPCDFile <pcl::PointXYZ> ("filename.pcd", *cloud) == -1 )
    {
        PCL_ERROR("Couldn't read file mypointcloud.pcd\n");
        return -1;
    }
    std::cerr << "cloud ok" << std::endl;


    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(100);
    n.compute(*normals);
    std::cerr << "normals ok" << std::endl;



    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    gp3.setSearchRadius(10000000);
    gp3.setMu(10000000);
    gp3.setMaximumNearestNeighbors(165);
    gp3.setMaximumSurfaceAngle(M_PI / 4);
    gp3.setMinimumAngle(M_PI / 180);
    gp3.setMaximumAngle(2 * M_PI);
    gp3.setNormalConsistency(false);
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);
    std::cerr << "triangles ok" << std::endl;

//    gp3.setSearchRadius(200);
//    gp3.setMu(200);
//    gp3.setMaximumNearestNeighbors(165);
//    gp3.setMaximumSurfaceAngle(M_PI);
//    gp3.setMinimumAngle(M_PI / 18);
//    gp3.setMaximumAngle(2 * M_PI);
//    gp3.setNormalConsistency(false);
//    gp3.setInputCloud(cloud_with_normals);
//    gp3.setSearchMethod(tree2);
//    gp3.reconstruct(triangles);
//    std::cerr << "triangles ok" << std::endl;

    pcl::io::saveVTKFile("mymesh.vtk", triangles);
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();
    fstream fs;
    fs.open("partsID.txt", ios::out);
    if (!fs)
    {
        return -2;
    }
    fs << "cloud number:" << parts.size() << "\n";
    for (int i = 0; i < parts.size(); i++)
    {
        if (parts[i] != 0)
        {
            fs << parts[i] << "\n";
        }
    }

    std::cerr << "viewer show ........" << std::endl;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0.4,0.4,0.88);
    viewer->addPolygonMesh(triangles, "my");
    viewer->setRepresentationToSurfaceForAllActors(); //panel
//    viewer->setRepresentationToPointsForAllActors(); //cloud
//    viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
//    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 1, 0.03, "normals");//normals
//    viewer->setRepresentationToWireframeForAllActors();  //line
    viewer->addCoordinateSystem(10.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    // Finish
    return 0;
}
