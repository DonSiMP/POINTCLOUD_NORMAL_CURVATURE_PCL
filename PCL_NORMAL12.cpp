#include <iostream>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>


using namespace pcl;
using namespace std;

int
main (int argc, char** argv)
{

    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
    if(io::loadPCDFile <pcl::PointXYZ> ("test_one.pcd", *cloud) == -1){
        cout << "cloud no" << endl;

        return 1;
    }
    cout << "cloud ok" << endl;

    PointCloud<PointXYZ>::Ptr filtered(new PointCloud<PointXYZ>());
    PassThrough<PointXYZ> filter;
    filter.setInputCloud(cloud);
    filter.filter(*filtered);
    cout << "filter ok" << endl;

    // MovingLeastSquares<PointXYZ, PointXYZ> mls;
    // mls.setInputCloud(filtered);
    // mls.setSearchRadius(0.01);
    // mls.setPolynomialFit(true);
    // mls.setPolynomialOrder(2);
    // mls.setUpsamplingMethod(MovingLeastSquares<PointXYZ, PointXYZ>::SAMPLE_LOCAL_PLANE);
    // mls.setUpsamplingRadius(0.005);
    // mls.setUpsamplingStepSize(0.003);

    // PointCloud<PointXYZ>::Ptr cloud_smoothed (new PointCloud<PointXYZ>());
    // mls.process(*cloud_smoothed);
    // cout << "移动最小二乘平面滤波完成" << endl;


    NormalEstimationOMP<PointXYZ, Normal> ne;
//    ne.setNumberOfThreads(8);
    ne.setInputCloud(filtered);
//    ne.setRadiusSearch(2.5);
    ne.setKSearch(100);
    Eigen::Vector4f centroid;
    compute3DCentroid(*filtered, centroid);
    ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

    PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>());
    ne.compute(*cloud_normals);

    for(size_t i = 0; i < cloud_normals->size(); ++i){
        cloud_normals->points[i].normal_x *= -1;
        cloud_normals->points[i].normal_y *= -1;
        cloud_normals->points[i].normal_z *= -1;
    }

    PointCloud<PointNormal>::Ptr cloud_smoothed_normals(new PointCloud<PointNormal>());
    concatenateFields(*filtered, *cloud_normals, *cloud_smoothed_normals);
    cout << "cloud_smoothed_normals ok" << endl;


    Poisson<PointNormal> poisson;
    poisson.setDepth(9);
    poisson.setInputCloud(cloud_smoothed_normals);

    PolygonMesh mesh;
    poisson.reconstruct(mesh);
    io::savePLYFile(argv[2], mesh);
    cout << "mesh ok" << endl;

    cout << "show......" << endl;
    boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("my viewer"));

    viewer->setBackgroundColor(0,0,7);
    viewer->addPolygonMesh(mesh, "my");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while(!viewer->wasStopped()){

        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }


    return (0);
}