#include <pcl/filters/bilateral.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>

typedef pcl::PointXYZ PointT;

int main(int argc, char *argv[]) {
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);

    const std::string PATH = "../pointclouds/";

    /* read the point cloud data */
    pcl::io::loadPCDFile(PATH + "table_scene_lms400.pcd", *cloud);

    /* display cloud data before filtering (debug) */
    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    /* set up the kdtree */
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    const double SPATIAL_SIGMA = 15.0f;
    const double RANGE_SIGMA = 0.03f;

    /* create the filtered object */
    pcl::BilateralFilter<PointT> bf;
    bf.setInputCloud(cloud);
    bf.setSearchMethod(tree);
    bf.setHalfSize(SPATIAL_SIGMA);
    bf.setStdDev(RANGE_SIGMA);
    bf.filter(*cloud_filtered);

    /* display cloud data after filtering (debug) */
    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << cloud_filtered << std::endl;

    /* save the filtered point cloud to file */
    pcl::io::savePCDFile(PATH + "table_scene_lms400_filtered.pcd", *cloud_filtered);

    return 0;
}
