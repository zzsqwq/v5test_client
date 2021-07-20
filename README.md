# v5test_client

## 使用方式	

​	创建catkin工作空间并将依赖包放入，在工作空间根目录下执行

```shell
$ catkin_make
$ source devel/setup.bash 				#若使用zsh，则替换为 devel/setup.zsh
$ rosrun v5test_client image_publisher
```

​	可以看到屏幕上有图像窗口出现并伴随终端输出，提示您是否已经成功与服务器通信以及答案是否正确。

​	图像中包含多种随机颜色的多边形。其中多边形**颜色随机，个数随机，个别多边形大小随机**。

​	同时，`image_publisher` 启动时可以附带参数 `type`，用来指定背景形式。

```shell
$ rosrun v5test_client image_publisher
or
$ rosrun v5test_client image_publisher 1
```

​	通过上述指令发布的图片背景为纯黑色。

```shell
$ rosrun v5test_client image_publisher 2
```

​	通过上述指令发布的图片背景为随机图片。
