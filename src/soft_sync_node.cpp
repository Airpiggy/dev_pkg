// ros头文件
#include <ros/ros.h>

// 消息类型
#include <sensor_msgs/Image.h>
#include <livox_ros_driver2/CustomMsg.h>

// message_filters相关组件
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class SyncNode
{
private:
    // 私有变量
    // 相机和雷达的订阅和发布
    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    message_filters::Subscriber<livox_ros_driver2::CustomMsg> lidar_sub;
    ros::Publisher image_pub;
    ros::Publisher lidar_pub;
    // 定义包含同步规则的变量
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, livox_ros_driver2::CustomMsg> MySyncPolicy;
    // 定义一个同步器
    message_filters::Synchronizer<MySyncPolicy> sync;
    // timer和存储信息的变量及标志位
    ros::Timer timer;
    sensor_msgs::Image last_image_msg;
    livox_ros_driver2::CustomMsg last_lidar_msg;
    bool new_image_received = false;
    bool new_lidar_received = false;

    // 回调函数
    void callback(const sensor_msgs::ImageConstPtr& img_msg, const livox_ros_driver2::CustomMsgConstPtr& lidar_msg)
    {
        ROS_INFO("Received syncronized messages.");
        last_image_msg = *img_msg;
        last_lidar_msg = *lidar_msg;
        new_image_received = true;
        new_lidar_received = true;
    }

    // 定时器回调函数，定时发送同步数据
    void timerCallback(const ros::TimerEvent& event)
    {
        // 如果相机和雷达数据均接收到，则发送信息并重置标志位
        if (new_image_received && new_lidar_received)
        {
            image_pub.publish(last_image_msg);
            lidar_pub.publish(last_lidar_msg);

            new_image_received = false;
            new_lidar_received = false;
        }
    }

public:
    // 构造函数
    SyncNode(ros::NodeHandle& nh) : sync(MySyncPolicy(10), image_sub, lidar_sub) // 初始化同步器
    {
        // 从参数服务器获取配置信息
        std::string image_topic, lidar_topic, synced_image_topic, synced_lidar_topic;
        nh.getParam("image_topic", image_topic);
        nh.getParam("lidar_topic", lidar_topic);
        nh.getParam("synced_image_topic", synced_image_topic);
        nh.getParam("synced_lidar_topic", synced_lidar_topic);

        // 订阅对应的topic
        image_sub.subscribe(nh, "/camera/color/image_raw", 10);
        lidar_sub.subscribe(nh, "/livox/lidar", 10);

        // 发布对应的topic
        image_pub = nh.advertise<sensor_msgs::Image>(synced_image_topic, 10);
        lidar_pub = nh.advertise<livox_ros_driver2::CustomMsg>(synced_lidar_topic, 10);
        
        // 发布频率
        double publish_rate;
        nh.getParam("publish_rate", publish_rate);
        
        // 注册回调函数
        sync.registerCallback(boost::bind(&SyncNode::callback, this, _1, _2));

        // 设置一个定时器，以制定的频率调用timerCallback函数
        ros::Duration period(1.0 / publish_rate);
        timer = nh.createTimer(period, &SyncNode::timerCallback, this);
    }
};

int main(int argc, char** argv)
{
    // 初始化ros节点
    ros::init(argc, argv, "soft_sync_node");

    // 创建节点句柄
    ros::NodeHandle nh("~");

    // 创建一个SyncNode类的实例
    SyncNode node(nh);

    // 输出节点启动成功信息
    ROS_INFO("soft_sync_node is up.");

    // 进入循环
    ros::spin();

    return 0;
}

