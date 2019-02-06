#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/features/principal_curvatures.h>


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
//    pcl::io::savePCDFile ("Normals.pcd", principalCurvatures);
}