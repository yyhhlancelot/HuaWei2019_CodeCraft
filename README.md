## 华为2019年软件精英挑战赛C++方案

### 官网(可能发生改变)
https://codecraft.huawei.com/

### 主要数据结构及程序逻辑
graph.h中构建了Graph类以及它的派生类Graphmtx类，用于构建邻接矩阵，为了使用最短路径算法。

method.h中构建了Car类、Cross类以及Road类。关键数据结构为Road类中构建的Car对象指针矩阵，通过对指针矩阵的改变达到行驶车辆的目的，同时矩阵中的每个元素代表了每一台车，将车移动时将原位置的指针置零，同时在新位置上赋予对象地址。
在Cross类中设立了Road指针指向该每个cross的各条road。Car类中设置了car的各种状态作为成员变量。
method.cpp中对各个类的成员函数进行实现。

process_2.h包含了process_2.cpp的各个函数的声明，process_2.cpp中实现了判别器规则(交通系统，任务书中指定的各项交通规则)。
CodeCraft-2019.cpp包含了整个流程的脚本。首先通过dijkstra算法找到最短路，再将规划的路径放入判别器中进行判别看是否未发生死锁(dead lock)。

read_write.h包含了对txt文件以及官方数据格式进行读写的函数。

### 不足之处
该程序未根据官方逻辑进行书写，官方逻辑为在判别器中发现死锁后改变路径。因为调试判别器花了不少时间，后面再写死锁机制感觉时间不够了，
而那个时候还没有上分，于是就放弃了去继续码判别器。同时程序中又很多可以优化的地方，包括循环的判断等，将这些地方进行优化能大大提高代码效率，但是还是心急上分，所以并没有再继续花时间在这上面。最终成绩不够理想，在成渝赛区只获得了115名，未能进入复赛。不过好处是获得了不少的项目经验，毕竟从10号开始正式写代码到30号结束，尝试了两套方案，差不多码了五六千行左右的代码，平时科研生活的强度和效率也赶不上这么高。
