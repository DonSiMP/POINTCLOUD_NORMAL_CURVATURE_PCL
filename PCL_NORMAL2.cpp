//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/surface/mls.h>
//#include<pcl/features/normal_3d.h>
//#include<pcl/features/principal_curvatures.h>
//#include <pcl/io/pcd_io.h>
//#include <iostream>
//#include <pcl/visualization/cloud_viewer.h>
//
//int user_data;
//
//void
//viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
//{
//    viewer.setBackgroundColor (0.1,0.1,0.1);
//    pcl::PointXYZ o;
//    o.x = 1.0;
//    o.y = 0;
//    o.z = 0;
//    viewer.addSphere (o, 0.25, "sphere", 0);
//    std::cout << "i only run once" << std::endl;
//
//}
//
//void
//viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
//{
//    static unsigned count = 0;
//    std::stringstream ss;
//    ss << "Once per viewer loop: " << count++;
//    viewer.removeShape ("text", 0);
//    viewer.addText (ss.str(), 200, 300, "text", 0);
//
//    //FIXME: possible race condition here:
//    user_data++;
//}
//
//int
//main (int argc, char** argv)
//{
//    // Load input file into a PointCloud<T> with an appropriate type
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
//    // Load bun0.pcd -- should be available with the PCL archive in test
//    pcl::io::loadPCDFile ("test.pcd", *cloud);//读取pcd文件
//
//    pcl::visualization::CloudViewer viewer("Cloud Viewer");
//    viewer.showCloud(cloud);
//
//    viewer.runOnVisualizationThreadOnce(viewerOneOff);
//
//    viewer.runOnVisualizationThread(viewerPsycho);
//
//    while (!viewer.wasStopped ())
//    {
//        //you can also do cool processing here
//        //FIXME: Note that this is running in a separate thread from viewerPsycho
//        //and you should guard against race conditions yourself...
//        user_data++;
//    }
//    return 0;


    //----------------------------------------------------------
//    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//    ne.setInputCloud(cloud);//创建法线估计的对象
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
//    ne.setSearchMethod(tree); //设置搜索方法(近邻搜索方式)
//    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
//    //ne.setRadiusSearch(0.05); //设置半径邻域搜索，半径为0.05米->5cm
//    ne.setKSearch(5);
//    ne.compute(*cloud_normals); //计算法向量
//
//    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures>pc;
//    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloud_curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
//    pc.setInputCloud(cloud);
//    pc.setInputNormals(cloud_normals);
//    pc.setSearchMethod(tree);
//    //pc.setRadiusSearch(0.05);
//    pc.setKSearch(5);
//    pc.compute(*cloud_curvatures);
//
//    for (size_t i = 0; i < cloud_curvatures->size(); i++ )
//    {
//        double  normal_x=cloud_curvatures->points[i].principal_curvature_x;
//        double  normal_y=cloud_curvatures->points[i].principal_curvature_y;
//        double  normal_z=cloud_curvatures->points[i].principal_curvature_z;
//
//        //std::cout << "normal_x: " << normal_x  << "normal_y " << normal_y  << "normal_z " << normal_z << std::endl;
//
//    }
//
//    // Save output
//    //pcl::io::savePCDFile ("bun0-mls.pcd", cloud_curvatures);
//
//
//
//    //std::cout << "cloud_curvatures->size()" <<cloud_curvatures->size()<< std::endl;
//    std::cout << "Normals good" << std::endl;
//}


#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);

#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_clusters.h>

int user_data;

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
//    viewer.setBackgroundColor(1.0f, 0.5f, 1.0f);
    viewer.setBackgroundColor(1.0f, 1.0f, 1.0f);
    pcl::PointXYZ o;
    o.x = 0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere(o, 0.25, "Sphere", 0);
    std::cout << "I only run once" << std::endl;
}


void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape("text", 0);
    viewer.addText(ss.str(), 200, 300, "text", 0);//this is to set the coordination of text "Once per viewer loop:"
    //FIXME : possible race condition here
    user_data++;
}

void keepLanePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_left_lane_threshold = 0.5,
                    float in_right_lane_threshold = 0.5)
{
    pcl::PointIndices::Ptr far_indices(new pcl::PointIndices);
    for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
    {
        pcl::PointXYZ current_point;
        current_point.x = in_cloud_ptr->points[i].x;
        current_point.y = in_cloud_ptr->points[i].y;
        current_point.z = in_cloud_ptr->points[i].z;

        if (current_point.y > (in_left_lane_threshold) || current_point.y < -0.1 * in_right_lane_threshold)
        {
            far_indices->indices.push_back(i);
        }
    }
    out_cloud_ptr->points.clear();
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(in_cloud_ptr);
    extract.setIndices(far_indices);
    extract.setNegative(true);  // true removes the indices, false leaves only the indices
    extract.filter(*out_cloud_ptr);

}

int
main()
{
    static double _keep_lane_left_distance;
    static double _keep_lane_right_distance;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPCDFile("test2.pcd", *cloud);
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    //showCloud 函数是同步的，在此处等待直到渲染显示为止
    viewer.showCloud(cloud);
    //该注册函数在可视化时只调用一次
    viewer.runOnVisualizationThreadOnce(viewerOneOff);
    //该注册函数在渲染输出是每次都调用
    viewer.runOnVisualizationThread(viewerPsycho);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clipped_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inlanes_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    keepLanePoints(clipped_cloud_ptr, inlanes_cloud_ptr, _keep_lane_left_distance, _keep_lane_right_distance);
//    private_nh.param("keep_lane_left_distance", _keep_lane_left_distance, 5.0);
//    ROS_INFO("keep_lane_left_distance: %f", _keep_lane_left_distance);
//    private_nh.param("keep_lane_right_distance", _keep_lane_right_distance, 5.0);
//    ROS_INFO("keep_lane_right_distance: %f", _keep_lane_right_distance);
    while (!viewer.wasStopped())
    {
        //在此处可以添加其他处理
        user_data++;
    }
    return 0;

}