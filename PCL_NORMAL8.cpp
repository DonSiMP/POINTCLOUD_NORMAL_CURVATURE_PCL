#include <stdio.h>
#include<boost/thread.hpp>
#include<boost/timer.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/integral_image_normal.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>


#define random(x1,x2) ((rand()%x2) - x1/2.0)

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>); //PointXYZ 数据结构
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_medium(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile ("bin_Laser-00147_-00847.pcd", *cloud2);
//    pcl::PCDReader reader;
//    reader.read("pcdData//lader3.PCD",*cloud2);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud2);
    pass.setFilterFieldName("x");
    pass.setFilterLimitsNegative(false);
    pass.setFilterLimits(5, 10);
    pass.filter(*cloud_medium);



    int v1(0), v2(0);
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud2);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    //ne.setRadiusSearch(0.05);
    ne.setKSearch(5);
    ne.compute(*cloud_normals);

    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures>pc;
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloud_curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
    pc.setInputCloud(cloud2);
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


    viewer->addPointCloud<pcl::PointXYZ>(cloud2, "cloud");

    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);//(Xmin,Ymin,Xmax,Ymax)设置窗口坐标
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addText("original", 10, 10, "v1 text", v1);//设置视口名称
    viewer->addPointCloud(cloud2, "sample cloud1", v1);//添加点云

    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud2, cloud_normals, 1, 0.03, "cloud_normals");
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }


    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
    viewer->addText("after filtered", 10, 10, "v2 text", v2);
    viewer->addPointCloud(cloud_medium, "sample cloud2", v2);

    viewer->addCoordinateSystem(1.0,"sample cloud1");

    while (!viewer->wasStopped())
    {

        viewer->spinOnce();
    }



    return 0;
}
