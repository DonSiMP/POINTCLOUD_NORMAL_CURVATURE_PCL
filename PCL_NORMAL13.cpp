#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h> //重构
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h> //可视化
#include <pcl/visualization/pcl_visualizer.h> //多线程
#include <boost/thread/thread.hpp>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <string>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if ( pcl::io::loadPCDFile <pcl::PointXYZ> ("filename.pcd", *cloud) == -1 )
    {
        PCL_ERROR("Couldn't read file mypointcloud.pcd\n");
        return -1;
    }
    std::cerr << "cloud ok" << std::endl;


    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::NormalEstimation<pcl::PointXYZ , pcl::Normal> n ;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>) ;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>) ;
    tree->setInputCloud(cloud) ;
    n.setInputCloud(cloud) ;
    n.setSearchMethod(tree) ;
    n.setKSearch(50);
    n.compute(*normals);

    pcl::concatenateFields(*cloud , *normals , *cloud_with_normals) ;

    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>) ;
    tree2->setInputCloud(cloud_with_normals) ;

    pcl::Poisson<pcl::PointNormal> pn ;
    pn.setConfidence(false);
    pn.setDegree(2);
    pn.setDepth(8);
    pn.setIsoDivide(8);
    pn.setManifold(false);
    pn.setOutputPolygons(false);
    pn.setSamplesPerNode(3.0);
    pn.setScale(1.25);
    pn.setSolverDivide(8);
    pn.setSearchMethod(tree2);
    pn.setInputCloud(cloud_with_normals);
    pcl::PolygonMesh mesh ;
    pn.performReconstruction(mesh);

    pcl::io::savePLYFile("result.ply", mesh);


    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer")) ;
    viewer->addPolylineFromPolygonMesh(mesh);
    viewer->setBackgroundColor(0 , 0 , 0) ;
    viewer->addPolygonMesh(mesh , "my") ;
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters() ;
    while (!viewer->wasStopped()){
        viewer->spinOnce(100) ;
        boost::this_thread::sleep(boost::posix_time::microseconds(100000)) ;
    }
}



