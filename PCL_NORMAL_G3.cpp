#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/visualization/cloud_viewer.h>


int user_data;

void
viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (1.0, 0.5, 1.0);
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.25, "sphere", 0);
    std::cout << "i only run once" << std::endl;

}

void
viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);

    //FIXME: possible race condition here:
    user_data++;
}


int
main (int argc, char** argv)
{
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    // Load bun0.pcd -- should be available with the PCL archive in test
    pcl::io::loadPCDFile ("bin_Laser-00152_-00851.pcd", *cloud);

    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::Normal>  ::Ptr Normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointNormal> ::Ptr PointWithNormals (new pcl::PointCloud<pcl::PointNormal>);

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setComputeNormals (true);

    // Set parameters
    mls.setInputCloud (cloud);
    mls.setPolynomialOrder (2);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (0.03);

    // Reconstruct
    mls.process (*PointWithNormals);



//    Normals=PointWithNormals;



    size_t size;
    size_t i;
    size = PointWithNormals->points.size();
    Normals->points.resize(PointWithNormals->size());
    for (i = 0; i < size; i++ )
    {
        Normals->points[i].normal_x = PointWithNormals->points[i].normal_x;
        Normals->points[i].normal_y = PointWithNormals->points[i].normal_y;
        Normals->points[i].normal_z = PointWithNormals->points[i].normal_z;
    }



    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> principalCurvaturesEstimation;

// Provide the original point cloud (without normals)
    principalCurvaturesEstimation.setInputCloud (cloud);

// Provide the point cloud with normals
    principalCurvaturesEstimation.setInputNormals(Normals);

// Use the same KdTree from the normal estimation
    principalCurvaturesEstimation.setSearchMethod (tree);
    principalCurvaturesEstimation.setKSearch(10);

// Actually compute the principal curvatures
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principalCurvatures (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
    principalCurvaturesEstimation.compute (*principalCurvatures); // only provide with pc1,pc2 and normals


    // Save output

    pcl::io::savePCDFile ("cloud.pcd", *cloud);
//    pcl::io::savePCDFile ("Normals.pcd", *Normals);
    pcl::io::savePCDFile ("PointWithNormals.pcd", *PointWithNormals);
//    pcl::io::savePCDFile ("principalCurvatures.pcd", *principalCurvatures);




    cout << "Normals " << Normals->size() << endl;
    cout << "principalCurvatures " << principalCurvatures->size() << endl;



    //visualization
    //visualization
    //visualization
    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    //blocks until the cloud is actually rendered
    viewer.showCloud(cloud);

    //use the following functions to get access to the underlying more advanced/powerful
    //PCLVisualizer

    //This will only get called once
    viewer.runOnVisualizationThreadOnce (viewerOneOff);

    //This will get called once per visualization iteration
    viewer.runOnVisualizationThread (viewerPsycho);
    while (!viewer.wasStopped ())
    {
        //you can also do cool processing here
        //FIXME: Note that this is running in a separate thread from viewerPsycho
        //and you should guard against race conditions yourself...
        user_data++;
    }
    return 0;












}