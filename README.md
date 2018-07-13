# SFM算法实现

代码实现了基本的SfM流程。该代码实现了Richard Hartley和Andrew Zisserman的Multiple View Geometry教科书中描述的理论。

## 依赖关系：
- cmake 
- OpenCV（测试版本为2.4.10和3.0）


## 编译说明
	cd StructurefromMotion
	mkdir build 
	cd build
	cmake 
	make

## 生成点云文件
    转到构建目录内的应用程序文件夹（cd apps）
    ./reconstruction将输出保存到文件
    ./reconstruction> output.xyz

## 中间结果
中间结果在output文件夹中，可使用MeshLab来查看.xyz3D点云文件。