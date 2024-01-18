#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>//ROS点云话题sensor_msgs::PointCloud2转pcl::PointCloud
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization//pcl_visualizer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>

void CreatCylinder(pcl::ModelCoefficients::Ptr& coefficients_cylinder, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,bool valization);
void cloud_cb2 (const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

ros::Publisher pub,pub2,pub3;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  // Do data processing here...
  output = *input;

  // Publish the data.
  pub.publish (output);

}

void cloud_cb2 (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

//    // Container for original & filtered data
//    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
//    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
//    pcl::PCLPointCloud2 cloud_filtered;
//
//    // Convert to PCL data type
//    pcl_conversions::toPCL(*cloud_msg, *cloud);
//
//    // Perform the actual filtering
//    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//    sor.setInputCloud (cloudPtr);
//    sor.setLeafSize (0.1, 0.1, 0.1);
//    sor.filter (cloud_filtered);

//实现圆柱分割、拟合
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);    //https://blog.csdn.net/luyuhangGray/article/details/122634340
    //--------passthrough filter---------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(cloud);
    pass_y.setFilterFieldName("x");//限制y轴范围  https://www.jianshu.com/p/a1c6cd7d411b
    pass_y.setFilterLimits(-0.6,0.6);
    pass_y.filter(*cloud_filtered);
    cout<<"Cloud_filtered has: "<<cloud_filtered->points.size()<<" data points."<<endl;

    //限制x轴范围  https://www.jianshu.com/p/a1c6cd7d411b
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(cloud_filtered);
    pass_x.setFilterFieldName("y");
    pass_x.setFilterLimits(-0.2,10);
    pass_x.filter(*cloud_filtered);
    cout<<"Cloud_filtered has: "<<cloud_filtered->points.size()<<" data points."<<endl;

    //限制z轴范围  https://www.jianshu.com/p/a1c6cd7d411b
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud_filtered);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(-0.2,1);
    pass_z.filter(*cloud_filtered);
    cout<<"Cloud_filtered has: "<<cloud_filtered->points.size()<<" data points."<<endl;

    //-----compute normal 点云法向量估计-----
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);//定义输出数据集
    //创建一个空的kdtree对象，并把它传递给法线估计对象。基于给出的输入数据集，kdtree将被建立
    pcl::search::KdTree<pcl::PointXYZ >::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ >);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_est;//估计法线
    normal_est.setSearchMethod(tree);
    //normal_est.setInputCloud(cloud);//用所有点
    normal_est.setInputCloud(cloud_filtered);//用过滤后的点
    normal_est.setKSearch(50);
    normal_est.compute(*cloud_normals);//计算特征值

    //-------extract_cylinder---------
    pcl::SACSegmentationFromNormals<pcl::PointXYZ,pcl::Normal> seg;//创建一个分割器，用于分割点云
    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);//用于存储圆柱的系数的指针
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);//用于存储圆柱体内的点索引的指针

    seg.setOptimizeCoefficients(true);        //设置对估计的模型系数需要进行优化
    seg.setModelType(pcl::SACMODEL_CYLINDER); //设置分割模型为圆柱型
    seg.setMethodType(pcl::SAC_RANSAC);       //设置采用RANSAC作为算法的参数估计方法
    seg.setNormalDistanceWeight(0.1);         //设置表面法线权重系数
    seg.setMaxIterations(5000);               //设置迭代的最大次数
    seg.setDistanceThreshold(0.05);           //设置内点到模型的距离允许最大值
    seg.setRadiusLimits(0, 0.5);    //设置估计出圆柱模型的半径范围
    //seg.setInputCloud(cloud);
    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(cloud_normals);
    //获取圆柱模型系数和圆柱上的点
    seg.segment(*inliers_cylinder, *coefficients_cylinder);     //最终获取内点以及模型系数
    cout << "Cylinder coefficients: " << *coefficients_cylinder << endl;

    //-----------------------------存储点云到输出文件----------------------------
    pcl::ExtractIndices<pcl::PointXYZ> extract;//创建一个用于提取圆柱体的点云提取对象
    pcl::PCDWriter writer;//用于写入点云数据到文件
    //extract.setInputCloud(cloud);
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);//设置提取模式为正向提取，即提取指定索引的点云
    pcl::PointCloud<pcl::PointXYZ >::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ >);//创建一个存储圆柱体点云的指针
    extract.filter(*cloud_cylinder);//通过点云提取对象提取圆柱体点云，并存储到cloud_cylinder中

    if (cloud_cylinder->points.empty())
        cout << "Can't find the cylindrical component." << endl;
    else
    {
        cout << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size() << " data points." << endl;
        //writer.write("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
    }

    //-------visualization-----------
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("cloud_cylinder Viewer"));
    //viewer->removeAllPointClouds();  // 移除当前所有点云
//    viewer->addCoordinateSystem(1.0);
//    viewer->addLine(pcl::PointXYZ(0,0,0),pcl::PointXYZ(10,0,0),"x");
//    viewer->addLine(pcl::PointXYZ(0,0,0),pcl::PointXYZ(0,5,0),"y");
//    viewer->addLine(pcl::PointXYZ(0,0,0),pcl::PointXYZ(0,0,1),"z");
//    //viewer->addPointCloud(cloud,"cloud");
//    viewer->addPointCloud(cloud_cylinder,"cloud_cylinder");
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud_cylinder");
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_cylinder");
//    viewer->addCylinder(*coefficients_cylinder);    //PCL可视化圆柱体 https://blog.csdn.net/stanshi/article/details/124773320
//    //viewer->spin();
//    viewer->spinOnce(1);//https://blog.csdn.net/jqw11/article/details/90757077

    cout<<"PointCloud has: "<<cloud->points.size()<<" data points."<<endl;

    // Convert to ROS data type
//    sensor_msgs::PointCloud2 output;
//    pcl::toROSMsg(*cloud_cylinder,output);//这边转成ros类型后可以publish出去，然后在rviz中就可以看了（https://blog.csdn.net/acquirement/article/details/119384896）
//    // Publish the data
//    pub.publish (output);


    //画圆柱
    pcl::ModelCoefficients::Ptr cylinder(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_virtual(new pcl::PointCloud<pcl::PointXYZ>);//保存虚拟圆柱点云
    cylinder->values.resize(7);
    //设置参数 参考https://blog.csdn.net/qq_45006390/article/details/119102420
    //自己实现限制，让画图更好看
    double x0 = coefficients_cylinder->values[0];
    double y0 = coefficients_cylinder->values[1];
    double z0 = coefficients_cylinder->values[2];
    double a = coefficients_cylinder->values[3];
    double b = coefficients_cylinder->values[4];
    double c = coefficients_cylinder->values[5];
    double R = coefficients_cylinder->values[6];
    if(coefficients_cylinder->values[4]<0)//保证朝向,使拟合的圆柱朝前
    {
        cylinder->values[3] = -1*a;
        cylinder->values[4] = -1*b;
        cylinder->values[5] = -1*c;
    }
    else
    {
        cylinder->values[3] = a;
        cylinder->values[4] = b;
        cylinder->values[5] = c;
    }
//    //z坐标为0
//    cylinder->values[0] = x0-a*z0/c;
//    cylinder->values[1] = y0-b*z0/c;
//    cylinder->values[2] = 0;
    //x坐标为0
    cylinder->values[0] = 0;
    cylinder->values[1] = y0-b*x0/a;
    cylinder->values[2] = z0-c*x0/a;
    cylinder->values[6] = R;
    //values[0]、values[1]、values[2]：圆柱轴线上一点的xyz坐标
    //values[3]、values[4]、values[5]：圆柱轴线的方向向量
    //values[6]：圆柱半径

    CreatCylinder(cylinder, cloud_virtual,false);
    //点云写入磁盘
    //pcl::io::savePCDFile("/home/hl/project/pcl_ws/cylinder.pcd", *cloud_virtual);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_virtual,output);//这边转成ros类型后可以publish出去，然后在rviz中就可以看了（https://blog.csdn.net/acquirement/article/details/119384896）
    output.header.frame_id = "livox_frame";//rviz一直显示有问题，加了这句设置output话题的frame_id就好了 参考https://blog.csdn.net/qq_43176116/article/details/88020003
    // Publish the data
    pub.publish (output);

    sensor_msgs::PointCloud2 output2;
    pcl::toROSMsg(*cloud_filtered,output2);//这边转成ros类型后可以publish出去，然后在rviz中就可以看了（https://blog.csdn.net/acquirement/article/details/119384896）
    output2.header.frame_id = "livox_frame";
    pub2.publish (output2);

    sensor_msgs::PointCloud2 output_all;
    pcl::toROSMsg(*cloud,output_all);//这边转成ros类型后可以publish出去，然后在rviz中就可以看了（https://blog.csdn.net/acquirement/article/details/119384896）
    output_all.header.frame_id = "livox_frame";
    pub3.publish (output_all);

}

//画圆柱 https://blog.csdn.net/Dbojuedzw/article/details/127472475
//根据圆柱面参数创建空间任意圆柱面点云
//param[in]  pcl::ModelCoefficients::Ptr& coefficients_cylinder://圆柱面参数：系数0、1、2代表圆柱轴线上的原点，3、4、5代表这条轴线的方向向量，系数6就是圆柱的半径。
//param[in] valization：默认可视化
//param[out] cloud：PCD格式的点云文件
void CreatCylinder(pcl::ModelCoefficients::Ptr& coefficients_cylinder, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool valization) {
    //检查参数
    if (coefficients_cylinder->values.size() != 7) {
        std::cerr << "参数数目错误。。。" << std::endl;
        system("pause");
    }
    if(coefficients_cylinder->values[6]<=0){
        std::cerr << "圆柱面半径不能小于等于0" << std::endl;
        system("pause");
    }
    //先构建轴线为Z轴的圆柱面点云
    int Num = 1200;
    float inter = 2.0 * M_PI / Num;
    Eigen::RowVectorXd vectorx(Num), vectory(Num), vectorz(Num);
    Eigen::RowVector3d axis(coefficients_cylinder->values[3], coefficients_cylinder->values[4], coefficients_cylinder->values[5]);
    float length = axis.norm();
    vectorx.setLinSpaced(Num, 0, Num - 1);
    vectory = vectorx;
    float x0, y0, z0,r0;
    x0 = coefficients_cylinder->values[0];
    y0 = coefficients_cylinder->values[1];
    z0 = coefficients_cylinder->values[2];
    r0 = coefficients_cylinder->values[6];

    pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder(new pcl::PointCloud<pcl::PointXYZ>);

    for (float z(0.0); z <= length; z += 0.005)//设置截面圆间距
    {
        for (auto i = 0; i < Num; ++i) {
            pcl::PointXYZ point;
            point.x = r0 * cos(vectorx[i] * inter);
            point.y = r0 * sin(vectory[i] * inter);
            point.z = z;
            cylinder->points.push_back(point);
        }
    }

    cylinder->width = (int)cylinder->size();
    cylinder->height = 1;
    cylinder->is_dense = false;

    //点云旋转 Z轴转到axis
    Eigen::RowVector3d  Z(0.0, 0.0, 0.1), T0(0, 0, 0), T(coefficients_cylinder->values[0], coefficients_cylinder->values[1], coefficients_cylinder->values[2]);
    Eigen::Matrix3d R;
    Eigen::Matrix3d E = Eigen::MatrixXd::Identity(3, 3);
    Eigen::Matrix4d Rotate,Translation;
    R = Eigen::Quaterniond::FromTwoVectors(Z, axis).toRotationMatrix();
    Rotate.setIdentity();
    Translation.setIdentity();

    //旋转
    Rotate.block<3, 3>(0, 0) = R;
    Rotate.block<3, 1>(0, 3) = T;
    pcl::transformPointCloud(*cylinder, *cloud, Rotate);


    if (valization) {
        //--------------------------------------可视化--------------------------
        pcl::visualization::PCLVisualizer viewer;
        //创建的点云和直接addcylinder函数创建的圆柱面面片进行必对
        viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud1");
        viewer.addCylinder(*coefficients_cylinder, "cylinder");
        viewer.addCoordinateSystem();
        while (!viewer.wasStopped())
        {
            viewer.spinOnce(100);
        }
    }
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "PclProj");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  //ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);
  ros::Subscriber sub = nh.subscribe ("/livox/lidar", 1, cloud_cb2);//queue_size改小可以更快更新拟合点

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  pub2 = nh.advertise<sensor_msgs::PointCloud2> ("output2", 1);
  pub3 = nh.advertise<sensor_msgs::PointCloud2> ("output_all", 1);

  // Spin
  ros::spin();
}

/* 输出内容解释 https://zhuanlan.zhihu.com/p/461614904
 values[0]:   0.023054              x0
  values[1]:   -0.0171193           y0
  values[2]:   0.209944             z0
  values[3]:   0.601884             a
  values[4]:   0.7541               b
  values[5]:   0.262809             c
  values[6]:   0.26476
values[0]、values[1]、values[2]：圆柱轴线上一点的xyz坐标
values[3]、values[4]、values[5]：圆柱轴线的方向向量
values[6]：圆柱半径
 */