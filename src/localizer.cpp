#include "ymsm_midterm2024_localizer/localizer/node.h"

int main(int argc, char **argv)
{
  //ノードの初期化
  ros::init(argc, argv, "localizer");
  ymsm_midterm2024_localizer::localizer::Node node;
  ros::spin();
  return 0;
}