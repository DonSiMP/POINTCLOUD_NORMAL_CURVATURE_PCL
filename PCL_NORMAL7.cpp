#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include<pcl/features/normal_3d.h>
#include<pcl/features/principal_curvatures.h>
#include <pcl/features/integral_image_normal.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
//fa xian

int
main (int argc, char** argv)
{
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::io::loadPCDFile ("test3.pcd", *cloud);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    //ne.setRadiusSearch(0.05);
    ne.setKSearch(5);
    ne.compute(*cloud_normals);

    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures>pc;
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloud_curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
    pc.setInputCloud(cloud);
    pc.setInputNormals(cloud_normals);
    pc.setSearchMethod(tree);
    //pc.setRadiusSearch(0.05);
    pc.setKSearch(5);
    pc.compute(*cloud_curvatures);



    for (size_t i = 0; i < cloud_curvatures->size(); i++ )
    {
        double  normal_x=cloud_curvatures->points[i].principal_curvature_x;
        double  normal_y=cloud_curvatures->points[i].principal_curvature_y;
        double  normal_z=cloud_curvatures->points[i].principal_curvature_z;

        std::cout << "normal_x: " << normal_x  << "normal_y: " << normal_y  << "normal_z: " << normal_z << std::endl;

    }

    // Save output
    //pcl::io::savePCDFile ("bun0-mls.pcd", cloud_curvatures);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");

    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals, 1, 0.03, "cloud_normals");
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    std::cout << "cloud_curvatures->size()" <<cloud_curvatures->size()<< std::endl;
    std::cout << "Normals good" << std::endl;
}