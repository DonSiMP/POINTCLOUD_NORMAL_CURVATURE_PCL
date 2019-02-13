#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
    //定义一个点云cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile ("test2.pcd", *cloud);


    // Normal estimation*
    //法向计算
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    //建立kdtree来进行近邻点集搜索
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    //为kdtree添加点云数据
    tree->setInputCloud(cloud);

    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    //点云法向计算时，需要搜索的近邻点大小
    n.setKSearch(100);
    //开始进行法向计算
    n.compute(*normals);
    //* normals should not contain the point normals + surface curvatures

    //显示类
    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

    //设置背景色
    viewer.setBackgroundColor(0, 0, 0);

    //按照z值进行渲染点的颜色
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "x");

    //添加需要显示的点云数据
    viewer.addPointCloud<pcl::PointXYZ>(cloud, fildColor, "sample cloud");

    //设置点显示大小
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");

    //添加需要显示的点云法向。cloud为原始点云模型，normal为法向信息，1表示需要显示法向的点云间隔，即每1个点显示一次法向，0.01表示法向长度。
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 1, 0.01, "normals");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

}
