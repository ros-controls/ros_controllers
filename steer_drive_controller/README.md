## Steer Drive Controller ##

Controller for a steer drive mobile base.

## 仕様
- Subscribe
  - `steer_drive_controller/cmd_vel` にTwist型のメッセージを投げて下さい．
- Publish
  - `steer_drive_controller/odom` を出すべきなのですが，未完成です．

- 例：SteerDriveController + SteerBot(RobotHWSim) で起動して，以下のメッセージで動く
```bash
rostopic pub -1 steer_drive_controller/cmd_vel geometry_msgs/Twist -- '[1.0, 0.0, 0.0]' '[0.0, 0.0, 0.5]'
```

## third_robotのlinkメモ
### ベース
- base_link

### ステアリング
- steer

### ホイール
- left_front_wheel
- left_rear_wheel
- right_front_wheel
- right_rear_wheel

### センサ
- camera_link
- front_bottom_lrf
- front_top_lrf
- rear_bottom_lrf

## third_robotのjointメモ
### ホイール
- base_to_left_front_wheel
- base_to_left_rear_wheel
- base_to_right_front_wheel
- base_to_right_rear_wheel

### ステアリング
- base_to_steer
- base_to_steer_right: gazebo用のvirtual.
- base_to_steer_left: gazebo用のvirtual.

## デバッグ開発の手順
### 準備
- パスを通す

```bash
source path.bash
```

- gazeboを実行する
```bash
roslaunch third_robot_gazebo third_robot_world.launch 
```

- コントローラ動作用の最低限の設定を行うlaunchファイルを実行する
```bash
roslaunch steer_drive_controller steer_drive_test_setting.launch 
```

### デバッガ
- パスを通したコンソールでQtCreatorを起動する
```bash
qtcreator
```

- [QtCreatorでROSのパッケージをビルド&デバッグ実行する](http://qiita.com/MoriKen/items/ea41e485929e0724d15e)にしたがってビルドする．実行オプションで`steer_drive_test`を選択する．

- `third_robot_control/steer_drive_controller/include/steer_drive_controller.h`内で`#define GUI_DEBUG`をアンコメントアウトする．
 - 53行目：`#define GUI_DEBUG // uncommentout when you use qtcreator for debugging` ってとこ．
 - デバッグ用のモジュールに名前空間がついてないので，無理やり付与する設定．

- `third_robot_control/steer_drive_controller/test/steer_drive_test.cpp` のmain関数内でブレイクポイントを置いてデバッグ実行．
  - SteerRobotはフェイクで作ってある．
  - `bool rt_code = rb_controller.init(steer_drive_bot, nh_main, nh_control);` でコントローラの初期化
  
