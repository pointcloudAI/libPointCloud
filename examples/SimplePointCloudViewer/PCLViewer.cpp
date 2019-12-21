/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */

#include "PCLViewer.h"
#include "PCLGrabber.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/marching_cubes.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/io/vtk_lib_io.h>


#define PCL_NO_PRECOMPILE
#include <pcl/visualization/pcl_visualizer.h>
#undef PCL_NO_PRECOMPILE

namespace PointCloud
{
  
#define CLOUD_NAME "cloud"

PCLViewer::PCLViewer()
{
}

void PCLViewer::_renderLoop()
{
  if(!_viewer)
  {
    _viewer =  Ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("PCL PointCloud Viewer"));
    _viewer->setBackgroundColor(0, 0, 0);
    //_viewer->createInteractor();
    _viewer->addCoordinateSystem(1.0);
    _viewer->initCameraParameters();
    _viewer->setShowFPS(true);
    _viewer->resetCameraViewpoint();
    //_viewer->setCameraPosition(0, 0, 0, 0, 0, 5, 0, 0, 1);
    _viewer->setCameraPosition(15, 15, 20, 0, 0, 5, 0, 0, 1);
  }
  
  if(_viewer->wasStopped())
    _viewer->resetStoppedFlag();
  
  bool firstTime = false;
  int updateCount = 0;
  
  while(!_stopLoop && !_viewer->wasStopped())
  {
    {
      Lock<Mutex> _(_cloudUpdateMutex);
      
      if(_cloud && _handler)
      {
          
        //---------------------------------------------------------------------------
          // 估计法向量
          //pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> n;
          // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
          
          // pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
          // //pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
          // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
          // tree->setInputCloud(_cloud);
          // n.setInputCloud(_cloud);
          // n.setSearchMethod(tree);
          // n.setKSearch(20);
          // n.compute (*normals); //计算法线，结果存储在normals中
          // normals 不能同时包含点的法向量和表面的曲率
          
          //将点云和法线放到一起
          //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>) ;
          // pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
          // pcl::concatenateFields(*_cloud, *normals, *cloud_with_normals);
          // cloud_with_normals = _cloud + normals
 
          // //创建搜索树
          // pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
          // tree2->setInputCloud (cloud_with_normals);
          
          // //初始化MarchingCubes对象，并设置参数
          // pcl::MarchingCubes<pcl::PointNormal> *mc;
          // mc = new pcl::MarchingCubesHoppe<pcl::PointNormal> ();
         
          // //创建多变形网格，用于存储结果
          // pcl::PolygonMesh mesh;
          
          // //设置MarchingCubes对象的参数
          // mc->setIsoLevel (0.0f);
          // mc->setGridResolution (50, 50, 50);
          // mc->setPercentageExtendGrid (0.0f);
          
          // //设置搜索方法
          // mc->setInputCloud (cloud_with_normals);
          
          // //执行重构，结果保存在mesh中
          // mc->reconstruct (mesh);
          
          //保存网格图
          // pcl::io::savePolygonFileSTL ("/Users/hejie/Documents/PointcloudAI/workspace/result.stl", mesh);
          // 显示结果图
        //---------------------------------------------------------------------------
        double psize = 1.0, opacity = 1.0, linesize =1.0;
          
        _viewer->getPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, linesize, CLOUD_NAME);
        _viewer->getPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, CLOUD_NAME);
        _viewer->getPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize, CLOUD_NAME);
       // _viewer->addPolygonMesh(mesh,"my"); //设置显示的网格
        
        if(!_viewer->updatePointCloud(_cloud, *_handler, CLOUD_NAME))
        {
          _viewer->addPointCloud(_cloud, *_handler, CLOUD_NAME);
          _viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, CLOUD_NAME);
          
          // Get the cloud mapper and set it not scale intensity to false color range dynamically
          auto cloudActorMap = _viewer->getCloudActorMap();
          auto actor = cloudActorMap->find(CLOUD_NAME);
          vtkPolyDataMapper *polyDataMapper = reinterpret_cast<vtkPolyDataMapper*>(actor->second.actor->GetMapper());
          polyDataMapper->UseLookupTableScalarRangeOn();
        }
        else
        {
          _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize, CLOUD_NAME);
          _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, linesize, CLOUD_NAME);
          _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, CLOUD_NAME);
        }
      }
    }
    
    _viewer->spinOnce(10);
    std::this_thread::sleep_for(std::chrono::microseconds(10000));
    
    updateCount++;
    if(firstTime)
    {
      _viewer->setCameraPosition(15, 15, 20, 0, 0, 5, 0, 0, 1);
      firstTime = false;
    }
  }
  
  //_viewer->close();
  //_viewer = nullptr;
  _stopLoop = true;
}



void PCLViewer::setDepthCamera(DepthCameraPtr depthCamera)
{
  if(isRunning())
  {
    stop();
    _depthCamera = depthCamera;
    start();
  }
  else
    _depthCamera = depthCamera;
}

void PCLViewer::removeDepthCamera()
{
  if(isRunning())
  {
    stop();
  }
  
  _depthCamera = nullptr;
}


//void PCLViewer::_cloudRenderCallback(const pcl::PointCloud<pcl::PointXYZI> &cloud)
void PCLViewer::_cloudRenderCallback(const pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    
  if(_viewer && !_viewer->wasStopped())
  {
    Lock<Mutex> _(_cloudUpdateMutex);

    //_cloud = boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>>(new pcl::PointCloud<pcl::PointXYZI>(cloud));
      _cloud = boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>(cloud));
      //static int i= 0;
      //i++;
      //std::cout << i << "_cloudRenderCallback:" <<_cloud->points.size() << std::endl;
    //_handler = Ptr<pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>>(
   //   new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>(_cloud, "intensity"));
      _handler = Ptr<pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ>>(
        new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ>(_cloud, "z"));
      
      
  }
}

void PCLViewer::start()
{
  if(!_depthCamera)
    return;
  
  _stopLoop = true;
  
  //if(_renderThread.joinable())
  //  _renderThread.join();

#ifdef WINDOWS
  _viewer = nullptr;
#endif

  _stopLoop = false;
  
  //_renderThread = std::thread(&PCLViewer::_renderLoop, this);
    
  _grabber = Ptr<pcl::Grabber>(new PointCloud::PCLGrabber(*_depthCamera));
  
  //boost::function<void (const pcl::PointCloud<pcl::PointXYZI> &)> f = boost::bind(&PCLViewer::_cloudRenderCallback, this, _1);
    boost::function<void (const pcl::PointCloud<pcl::PointXYZ> &)> f = boost::bind(&PCLViewer::_cloudRenderCallback, this, _1);
    
  _grabber->registerCallback(f);
  _grabber->start();
}

bool PCLViewer::isRunning()
{
  return _grabber && _grabber->isRunning() && !_stopLoop && !viewerStopped();
}

bool PCLViewer::viewerStopped()
{
  return !_viewer || _viewer->wasStopped();
}

void PCLViewer::stop()
{
  _stopLoop = true;
  
  //if(_renderThread.joinable()) _renderThread.join();
  
  //if (_viewer)
  //{
  //  Lock<Mutex> _(_cloudUpdateMutex);
  //  _viewer->removePointCloud(CLOUD_NAME);
  //}

#ifdef WINDOWS
  _viewer = nullptr;
#endif
  
  _grabber = nullptr;
  
  
}

}
