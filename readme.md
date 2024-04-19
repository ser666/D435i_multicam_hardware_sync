Environment：ubuntu20.04,librealsenseSDK,opencv,C++

Build:

```
mkdir build

cd build

cmake ..

Make
```



run:

```
cd build

./hwsyn
```



使用步骤：
1、下位机数据脉冲开始发送。

2、确定数据保存目录位置，确定路径下没有DATA文件夹。

3、通过USB3.0接口连接相机。

4、运行主机端程序后，然后需要等待三秒检测链接各个相机。

5、程序开始采集并保存数据。（由于三个相机启动顺序和时间不一致，而且脉冲事先已经在发送，所以先启动的相机会先采几组数据，即多几个更小帧序号数据，第一组完整数据查看最后一个相机的IMU中的第一个帧序号）

程序会自动创建DATA文件夹以及其中的子文件夹，每次采集数据前需要删掉整个DATA文件夹！！！
程序使用的是绝对路径，注意修改。

```c++
std::string data_path = "/home/merlincs/hwsyn/Data";//"/home/merlincs/hwsyn/Data"根据自己程序位置进行修改
```


IMU针对第一个图片由于要创建txt文件导致数据保存失败，之后IMU数据采集不影响，这是一个小BUG。
