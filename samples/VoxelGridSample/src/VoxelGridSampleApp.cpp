#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/gl/Vbo.h"
#include "cinder/gl/Texture.h"
#include "cinder/ImageIo.h"
#include "Resources.h"

#include <pcl/point_types.h>

#define nil Boost_nil
#define Nil Boost_Nil
#include "cinder/Rand.h"
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ros/conversions.h>
using namespace ci;
using namespace ci::app;
using namespace std;

class VoxelGridSampleApp : public AppBasic {
  public:
	void setup();
	void mouseDown( MouseEvent event );
    void keyDown(KeyEvent event);
	void update();
	void draw();
private:
    void setupCloud();
    void createPointVbo();
    int32_t xSize, ySize, size, newXSize, newYSize;
    float leafSize;
    bool isUpdated;
    
    sensor_msgs::PointCloud2::Ptr cloud_filtered;
    sensor_msgs::PointCloud2 cloud2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud;
    
    pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
    
    // VBO AND SHADER
	gl::VboMesh		mVboMesh;
	gl::GlslProg	mShader;
    std::vector<Vec3f> mPositions;
    
    gl::Texture				mDepthTex;
};

void VoxelGridSampleApp ::setupCloud()
{
    filteredCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>);
    cloud->height = xSize;
    cloud->width = ySize;
    cloud->is_dense = false;
    cloud->points.resize (cloud->height * cloud->width);
    register int depth_idx = 0;
    for (int v = -240; v < 240; ++v)
    {
        for (register int u = -320; u < 320; ++u, ++depth_idx)
        {
            pcl::PointXYZ& pt = cloud->points[depth_idx];
            float constant = 1.0f;
            pt.z = static_cast<float> (u) * Rand::randFloat( 0.0f, 20.0f ) * constant;
            pt.x = static_cast<float> (u) * Rand::randFloat( 0.0f, 20.0f ) * constant;
            pt.y = static_cast<float> (u) * Rand::randFloat( 0.0f, 20.0f ) * constant;
        }
    }
    
    sensor_msgs::PointCloud2 cloud2;

    pcl::toROSMsg(*cloud, cloud2);

    sensor_msgs::PointCloud2ConstPtr cloud2Ptr (new sensor_msgs::PointCloud2(cloud2));
    
    std::cerr << "PointCloud before filtering: " << cloud2Ptr->width * cloud2Ptr->height
    << " data points (" << pcl::getFieldsList (*cloud2Ptr) << ").";
    
    cloud_filtered = sensor_msgs::PointCloud2::Ptr (new sensor_msgs::PointCloud2 ());
    sor.setInputCloud (cloud2Ptr);
}

void VoxelGridSampleApp::keyDown(KeyEvent event){
    if (event.getCode() == KeyEvent::KEY_UP){
        leafSize += 0.1f;
        isUpdated = true;
    } else if (event.getCode() == KeyEvent::KEY_DOWN){
        leafSize -= 0.1f;
        isUpdated = true;
    } else {
        isUpdated = false;        
    }
}

void VoxelGridSampleApp::setup()
{
    leafSize = 10.0f;
    isUpdated = true;
    xSize = newXSize = 480;
    ySize = newYSize = 640;
    
    mDepthTex = gl::Texture( xSize, ySize );
    
    setupCloud();
    
    createPointVbo();
    try {
        //mShader	= gl::GlslProg( loadResource( RES_VERT_ID ), loadResource( RES_FRAG_ID ) );
    } catch (cinder::Exception e) {
        //trace(e.what());
    }
    
    mDepthTex = gl::Texture( loadImage( loadResource( RES_IMAGE ) ) );
}

void VoxelGridSampleApp::createPointVbo()
{
	gl::VboMesh::Layout layout;
	
	layout.setStaticPositions();
	layout.setStaticTexCoords2d();
	layout.setStaticIndices();
    
	std::vector<Vec2f> texCoords;
	std::vector<uint32_t> indices;
	
	int numVertices = newXSize * newYSize;
	int numShapes	= ( newXSize - 1 ) * ( newYSize - 1 );
    
	mVboMesh		= gl::VboMesh( numVertices, numShapes, layout, GL_POINTS );
	mPositions.clear();
	for( int x=0; x<newXSize; ++x ){
		for( int y=0; y<newYSize; ++y ){
			indices.push_back( x * ySize + y );
            
			float xPer	= x / (float)(newXSize-1);
			float yPer	= y / (float)(newYSize-1);
			mPositions.push_back( Vec3f( ( xPer * 2.0f - 1.0f ) * newXSize, ( yPer * 2.0f - 1.0f ) * newYSize, 0.0f ) );
			texCoords.push_back( Vec2f( xPer, yPer ) );
		}
	}
	
	mVboMesh.bufferPositions( mPositions );
	mVboMesh.bufferIndices( indices );
	mVboMesh.bufferTexCoords2d(0, texCoords );
}

void VoxelGridSampleApp::mouseDown( MouseEvent event )
{
}

void VoxelGridSampleApp::update()
{
    if (!isUpdated) return;
    
    sor.setLeafSize (leafSize, leafSize, leafSize);
    sor.filter (*cloud_filtered);
    std::cerr << leafSize<<" PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
    << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")."<<std::endl;
    pcl::fromROSMsg (*cloud_filtered, *filteredCloud);
    
    newXSize = cloud_filtered->width;
    newYSize = cloud_filtered->height;
    createPointVbo();
    isUpdated = false;
}

void VoxelGridSampleApp::draw()
{
	// clear out the window with black
	gl::clear( Color( 0, 0, 0 ) );
    
    gl::pushMatrices();
    gl::scale( Vec3f( -1.0f, -1.0f, 1.0f ) );
//    gl::rotate( mSceneRotation );
    Matrix44f transform;
//    transform.rotate( Vec3f(toRadians(rotate.x), toRadians(rotate.y), toRadians(rotate.z)) );
//    transform.translate( translate );
    
    gl::pushModelView();
//    gl::multModelView( transform );
    
    mDepthTex.enableAndBind();
    
//    mShader.bind();
//    mShader.uniform("depthTex", 0 );
    
    gl::draw( mVboMesh );
    //mShader.unbind();
    
	gl::popMatrices();
    gl::popModelView();
}

CINDER_APP_BASIC( VoxelGridSampleApp, RendererGl )
