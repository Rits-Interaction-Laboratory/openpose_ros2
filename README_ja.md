# openpose_ros2
[English](./README.md) | 日本語

ROS2からOpenPoseを呼び出すROS2パッケージ

## 動作要件
* Python 3.6 以上
* ROS2 dashing
* cv_bridge
* [OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose)
    * Python Wrapper

## 動作確認環境
* Python 3.6
* Ubuntu 18.04 LTS
* ROS2 dashing
* OpenPose v1.7.0

## インストール
1. [OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose) をインストールします。このROS2パッケージは Openpose の　PythonWrapperが必要です。
1. このリポジトリをros2ワークスペースにクローンします(大体は `~/ros2_ws/src`)。
1. ros2ワークスペースのルートに移動します(大体は `~/ros2_ws`)。移動後 `colcon build --base-paths src/openpose_ros2` を実行しパッケージをビルドします。
1. `. install/setup.bash` を実行してパッケージをアップデートします。

## params.ymlでノードを起動する (**推奨**)
1. Ros2ワークスペースのルートに移動します。
1. `cp ./src/openpose_ros2/openpose_ros2/params.yml.sample ./src/openpose_ros2/openpose_ros2/params.yml` を実行してファイルをコピーします。
1. `./src/openpose_ros2/openpose_ros2/params.yml` を開き編集します。パラメータについては「ROS2パラメータの一覧」を参照してください。
1. `ros2 run openpose_ros2 openpose_ros2 __params:=src/openpose_ros2/openpose_ros2/params.yml` を実行します。

## launch コマンドでノードを起動する
1. Ros2ワークスペースのルートに移動します。
1. `./src/openpose_ros2/openpose_ros2/launch/openpose_ros2.py` を開き編集します。パラメータについては「ROS2パラメータの一覧」を参照してください。
1. `colcon build --base-paths src/openpose_ros2` と `. install/setup.bash` を実行し、パッケージをビルドします。
1. `ros2 launch openpose_ros2 openpose_ros2.py` を実行します。

## ノードの一覧
* /openpose_node

## トピックの一覧
* /openpose/pose_key_points \[openpose_ros2_msgs/msg/PoseKeyPointsList\]
* /openpose/preview \[sensor_msgs/msg/Image\] (デバッグモードのみ)
* /openpose/preview/compressed \[sensor_msgs/msg/CompressedImage\] (デバッグモードのみ)

## ROS2パラメータの一覧
* is_debug_mode : trueの場合、デバッグモードで実行します。 
* openpose_root : OpenPoseのルートディレクトリを指定します。
* is_image_compressed : trueの場合、入力される画像はCompressedImage型を期待します。falseの場合は、入力される画像はImage型を期待します。
* image_node : 入力画像のノードを指定します。

## ライセンス
MIT License
