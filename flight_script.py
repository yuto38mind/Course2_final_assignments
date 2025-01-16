from pymavlink import mavutil
# 機体への接続
master = mavutil.mavfile = mavutil.mavlink_connection("tcp:127.0.0.1:5762", source_system=1, source_component=90)
#master = mavutil.mavfile = mavutil.mavlink_connection("tcp:127.0.0.1:14550", source_system=1, source_component=90)
try:
    print("SITLに接続中...")
    master.wait_heartbeat(timeout=10)  # タイムアウトを10秒に設定
    print("MAVLink接続成功！")
except mavutil.mavlink.MAVError as e:
    print("ハートビート受信失敗。SITLが動作していない可能性があります。")
    exit(1)

import time
import math
import numpy as np
#定義

Tick = 2.5
distance = 20
bearing = 0
target_altitude = 10

def request_message_interval(message_id: int, frequency_hz: float):
    """
    Request MAVLink message in a desired frequency,
    documentation for SET_MESSAGE_INTERVAL:
        https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

    Args:
        message_id (int): MAVLink message ID
        frequency_hz (float): Desired frequency in Hz
    """
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, # The MAVLink message ID
        1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
        0, 0, 0, 0, # Unused parameters
        0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
    )
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 1)

#離陸関数
def takeoff_operation():
    print("離陸します")
    # GUIDEDモードを設定
    print("mode")
    # set the arm
    master.set_mode(4)
    time.sleep(2)
    master.arducopter_arm()
    print("arm")
    master.motors_armed_wait()
    print("armed")
    time.sleep(4)
    #set the altitude
    target_altitude = 10
    # 離陸
    print("start to take off comand")
    master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0,
    0, 0, target_altitude   # 緯度、経度、高度
    )
    print("離陸中...")
    time.sleep(20)  # 離陸中の待機
    print("離陸完了！")

def land():
    master.mav.command_long_send(
    master.target_system,  # 対象システムID
    master.target_component,  # 対象コンポーネントID
    mavutil.mavlink.MAV_CMD_NAV_LAND,  # LAND コマンド
    0,  # Confirmation (通常は0)
    0, 0, 0, 0,  # パラメータ1〜4: 未使用
    0, 0, 0  # 緯度、経度、高度
    )
    print("着陸コマンドを送信しました")

    # Disarmコマンドを送信
    master.mav.command_long_send(
    master.target_system,  # 対象システムID
    master.target_component,  # 対象コンポーネントID
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # ARM/DISARMコマンド
    0,  # Confirmation (通常は0)
    0,  # パラメータ1: 0はDisarm、1はArm
    0, 0, 0, 0, 0, 0  # パラメータ2〜7: 未使用
    )
    print("ドローンをDisarm状態に設定しました")
    master.set_mode(0)
print("着陸完了")


#現在位置を取得する関数
def get_current_position(master,):

    print("get_current")
#    time.sleep(2)
    # GLOBAL_POSITION_INTメッセージを取得
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    #msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    if msg:
        cur_lat = msg.lat
        cur_lon = msg.lon
        cur_alt = msg.relative_alt
        cur_latitude = cur_lat / 1e7  # 緯度（1e7でスケール変換）
        cur_longitude = cur_lon / 1e7  # 経度（1e7でスケール変換）
        cur_altitude = cur_alt / 1000.0  # 相対高度（mmからmに変換）
        print(f"現在位置: 緯度={cur_latitude}, 経度={cur_longitude}, 高度={cur_altitude}m")
        return cur_lat, cur_lon, cur_alt
    else:
        print("位置情報を取得できませんでした")
        return None, None, None


#次の地点を算出する関数
def calculate_new_position(cur_lat, cur_lon, distance, bearing):
    """
    現在地(lat, lon)から指定距離(distance)と方位角(bearing)で新しい緯度と経度を計算する
    """
    R = 6378137.0  # 地球の半径 (メートル)
    bearing = math.radians(bearing)  # 方位角をラジアンに変換(0（北）、90（東）、180（南）、270（西）)

    new_lat_bk = cur_lat + ((distance / R) * math.cos(bearing) * (180 / math.pi) * 1e7)
    new_lon_bk = cur_lon + ((distance / (R * math.cos(math.radians(cur_lat)))) * math.sin(bearing) * (180 / math.pi) * 1e7)
#    print(f"計算結果: 緯度={new_lat_bk}, 経度={new_lon_bk}m")
    new_lat = int(round(new_lat_bk, 7))
    new_lon = int(round(new_lon_bk, 7))
#    print(f"最終計算結果: 緯度={new_lat}, 経度={new_lon}m")
    cur_lat = new_lat
    cur_lon = new_lon
    new_latitude = new_lat / 1e7  # 緯度（1e7でスケール変換）
    new_longitude = new_lon / 1e7  # 経度（1e7でスケール変換）
    print(f"次の位置: 緯度={new_latitude}, 経度={new_longitude}m")
    return new_lat, new_lon

#次の地点へ移動する関数
def move_next_location(master, new_lat, new_lon, target_altitude):
    print ("次の地点へ移動する関数よばれました")
    """
    指定した緯度、経度、高度に移動する
    """

    print(f"送信する値 (整数形式): 緯度={new_lat}, 経度={new_lon}, 高度={target_altitude}")

    master.mav.set_position_target_global_int_send(
        0,  # 時間
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # フレーム
        0b0000111111111000,  # 位置のみ制御
        int(new_lat),  # 緯度（整数形式で送信）
        int(new_lon),  # 経度（整数形式で送信）
        target_altitude,  # 高度（m）
        0, 0, 0,  # 速度（未使用）
        0, 0, 0,  # 加速度（未使用）
        0, 0  # 未使用
    )
    print(f"移動中...")
    time.sleep(3)
#    cur_lat, cur_lon, cur_alt = get_current_position(master)
#    print(f"現在地: 緯度={cur_lat / 1e7}, 経度={cur_lon / 1e7}, 高度={cur_alt / 1000}m")

#「S」のポイントを生成する関数
def S_make_points(S_upper_base_point, S_lower_base_point, Tick):
    #「S」のポイントを計算
#    upper_points = []
#    lower_points = []
#    num_points = 10  # 固定のポイント数
    #上部のポイント
#    S_upper_start_angle = 0
#    S_upper_end_angle = 270
#    S_upper_angle_step = (S_upper_start_angle + S_upper_end_angle) / (num_points - 1)
#    
#    for i in range(num_points):
#        S_upper_angle = S_upper_start_angle + i * S_upper_angle_step
 #       new_lat, new_lon = calculate_new_position(*S_upper_base_point, Tick, S_upper_angle)
  #      print(f"{S_upper_base_point},{S_upper_angle},{i}")
 #       upper_points.append((new_lat, new_lon))
    #「S」のポイントを計算_v2
    upper_points = []
    lower_points = []
    num_points = 10  # 固定のポイント数
    #上部のポイント
    #S_upper_start_angle = 450
    S_upper_start_angle = 270
    S_upper_end_angle = 180
    #S_upper_angle_step = (S_upper_start_angle - S_upper_end_angle) / (num_points - 1)
    S_upper_angle_step = 270 / (num_points - 1)

    for i in range(num_points):
        S_upper_angle = S_upper_start_angle + i * S_upper_angle_step
#        S_upper_base_point = S_lower_base_point #
        new_lat, new_lon = calculate_new_position(*S_upper_base_point, Tick, S_upper_angle)
        print(f"{S_upper_angle},{S_upper_base_point},{S_upper_angle},{i}")
        upper_points.append((new_lat, new_lon))
    
    #下部のポイント
    S_lower_start_angle = 360
    S_lower_end_angle = 270
    #S_lower_angle_step = (S_lower_end_angle + S_lower_start_angle) / (num_points - 1)
    #S_lower_angle_step = (S_lower_start_angle + S_lower_end_angle) / (num_points - 1)
    S_lower_angle_step = 270 / (num_points - 1)

    for i in range(num_points):
        S_lower_angle = S_lower_start_angle + i * S_lower_angle_step
        new_lat, new_lon = calculate_new_position(*S_lower_base_point, Tick, S_lower_angle)
        print(f"{S_lower_angle},{S_lower_base_point},{S_lower_angle},{i}")
        lower_points.append((new_lat, new_lon))
    
    points = upper_points + lower_points
    return points

#S文字を書く関数
def S_draw(master, cur_lat, cur_lon, target_altitude):
    #ポイントの起点の生成
    print(f"ポイント生成前: 緯度={cur_lat}, 経度={cur_lon}")
    #斜辺を計算
    S_upper_distance = math.sqrt(Tick**2 + (2 * Tick)**2)
    #角度を計算
    S_upper_bearing = 90 + math.degrees(math.atan((2 * Tick) / Tick**2))
    #S_upper_bearing = ?, S_upper_distance = ?
    print(f"角度={S_upper_bearing}, 斜辺={S_upper_distance}m")
    S_upper_bearing = 128
    S_upper_distance = 5
    #上の起点の計算
    S_upper_base_point = calculate_new_position(cur_lat, cur_lon, S_upper_distance, S_upper_bearing)
    #下の起点の計算
    #S_lower_base_point = calculate_new_position(cur_lat, cur_lon, (2 * Tick), 180)
    S_upper_new_lat, S_upper_new_lon = S_upper_base_point
    S_lower_base_point = ((S_upper_new_lat - 449), S_upper_new_lon)
    print(f"上部の支点={S_upper_base_point}, 下部の支点={S_lower_base_point}m")
    #「S」のポイントを生成
    S_points = S_make_points(S_upper_base_point, S_lower_base_point, Tick)
    #「S」のポイントの確認と実行
    for i, (new_lat, new_lon) in enumerate(S_points):
        #「S」のポイントの確認
        new_latitude = new_lat / 1e7  # 緯度（1e7でスケール変換）
        new_longitude = new_lon / 1e7  # 経度（1e7でスケール変換）
        print(f"ポイント{i+1}: 緯度={new_latitude}, 経度={new_longitude}, 高度={target_altitude}m に移動中...")
        #「S」のポイントの実行
        #time.sleep(4)
        time.sleep(2)
        move_next_location(master, new_lat, new_lon, target_altitude)
        #一定時間待機（到達時間を調整）
        print("next point")
    print("すべてのポイントを飛行完了しました")

def O_make_points(O_base_point, Tick):
    """
    楕円上の9つのポイントを計算する関数。
    
    Parameters:
        O_base_point (tuple): 基準点 (lat, lon)
        Tick (float): 楕円の短軸半径

    Returns:
        list: 楕円上の9つのポイント (lat, lon)
    """
    print(f"支点={O_base_point}")

    O_points = []  # 結果を格納するリスト
    num_points = 13  # ポイントの数（固定）

    # 楕円のパラメータ
    a = Tick  # 短軸半径
    b = 2 * Tick  # 長軸半径
    R = 6378137.0
    O_start_angle = 0
    O_angle_step = 360 / (num_points - 1)

    # 各点の角度（ラジアン）を計算
    for i in range(num_points):
        #theta = (2 * np.pi / (num_points - 1)) * i  # 角度（ラジアン）
        O_angle = math.radians(O_start_angle + i * O_angle_step) 

        # 楕円上の座標を計算
        #delta_lat = b * np.cos(O_angle)
        #delta_lon = a * np.sin(O_angle)
        delta_lat = ((b / R) * math.cos(O_angle) * (180 / math.pi) * 1e7)
        delta_lon = ((a / R) * math.sin(O_angle) * (180 / math.pi) * 1e7)


        print(f"楕円のある点の座標={delta_lat}, {delta_lon}")

        # 基準点に対する新しい座標
        new_lat_bk = O_base_point[0] + delta_lat
        new_lon_bk = O_base_point[1] + delta_lon
        new_lat = int(round(new_lat_bk, 0))
        new_lon = int(round(new_lon_bk, 0))

        # 点をリストに追加
        O_points.append((new_lat, new_lon))

        # デバッグ用に各点を表示
        #angle_degrees = np.degrees(O_angle)
        print(f"Point {i+1}: x = {new_lat}, y = {new_lon}, angle = {O_angle}°")
    return O_points



#O文字を書く関数
def O_draw(master, cur_lat, cur_lon, target_altitude):
    #ポイントの起点の生成
    #斜辺を計算
    O_distance_bk = math.sqrt(2 * Tick + (2 * Tick)**2)
    #角度を計算
    O_bearing_bk = 90 + math.degrees(math.atan((2 * Tick) / 2 * Tick))
    O_distance = int(round(O_distance_bk, 0))
    O_bearing = int(round(O_bearing_bk, 0))
    print(f"角度={O_bearing}, 斜辺={O_distance}m")
    O_bearing = 135
    O_distance = 7
    #「O」の起点を生成
    O_base_point = calculate_new_position(cur_lat, cur_lon, O_distance, O_bearing)

    #「O」のポイントを生成
    O_points = O_make_points(O_base_point, Tick)
    for i, (new_lat, new_lon) in enumerate(O_points):
        #「O」のポイントの確認
        new_latitude = new_lat / 1e7  # 緯度（1e7でスケール変換）
        new_longitude = new_lon / 1e7  # 経度（1e7でスケール変換）
        print(f"ポイント{i+1}: 緯度={new_latitude}, 経度={new_longitude}, 高度={target_altitude}m に移動中...")
        #「O」のポイントの実行
        #time.sleep(4)
        time.sleep(2)
        move_next_location(master, new_lat, new_lon, target_altitude)
        #一定時間待機（到達時間を調整）
        print("next point")
    print("すべてのポイントを飛行完了しました")
    

#離陸
takeoff_operation()
print("takeoff_関数ぬけました")
#ホーム地点を記憶
home_location = get_current_position(master)
if home_location[0] is not None and home_location[1] is not None:  # 緯度と経度が取得できた場合のみ実行
   cur_lat, cur_lon, _ = home_location  # 高度は無視
   print(f"HOME位置: 緯度={cur_lat}, 経度={cur_lon}, 高度={target_altitude}m")
time.sleep(15)
#１文字目のスタート地点の設定
start_location = calculate_new_position(cur_lat, cur_lon, distance, bearing)
    #１文字目のスタート地点へ移動

print(f"Start位置={start_location}")
move_next_location(master, *start_location, target_altitude)
time.sleep(20)
#cur_lat, cur_lon, cur_alt = get_current_position(master)
#print(f"start位置2: 緯度={cur_lat}, 経度={cur_lon}, 高度={target_altitude}m")
#time.sleep(15)
    #１文字目を書く
#S_draw(master, cur_lat, cur_lon, target_altitude)

S_draw(master, *start_location, target_altitude)
time.sleep(5)

    #２文字目のスタート地点の設定
second_location = calculate_new_position(*start_location, 4*Tick, 90)
    #２文字目のスタート地点へ移動
move_next_location(master, *second_location, target_altitude)
time.sleep(5)
    #２文字目を書く
O_draw(master, *second_location, target_altitude)
time.sleep(5)

    #３文字目のスタート地点の設定
third_location = calculate_new_position(*second_location, 4*Tick, 90)
    #３文字目のスタート地点へ移動
move_next_location(master, *third_location, target_altitude)
time.sleep(5)
    #３文字目を書く
S_draw(master, *third_location, target_altitude)
time.sleep(10)

    #ホーム地点へ移動
move_next_location(master, *home_location)
time.sleep(10)
    #着陸
land()