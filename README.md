# Course2_final_assignments

# プロジェクト名
SKY　SCRIPT

## インストール方法
このプロジェクトをインストールする手順を説明します。
おそらく下記のURLから取得できると思います。

git clone https://github.com/yuto38mind/Course2_final_assignments.git

## 実行方法
ターミナルから下記のコマンドを実行する。
sim_vehicle.py -v ArduCopter --console --map

他のターミナルでリポジトリされているコードを実行する。

## 実装内容と説明
コース３での成果物と同じく、上空にSOSの文字を描くことを試みました。
このコードは、mavlinkを使用し、pythonで記述されています。
RTLやWPを設けることは容易のため、あえてこの機能は使用せず、実装しています。
なぜなら自らソースを作成することで、この高機能に感謝できると考えたためです。

今回は3文字を描くため、3マスの範囲を設けており、それぞれの始点を　start_location,secound_location,third_location　としています。

文字を描く時の動作は遅く、緯度経度を使用した移動のため、次の地点への移動は遅いため、
今後はxyz軸での操作で実装を試みます。

## 最後に
人生ではじめてこの量のコードを描きました。
また、ご教授くださった方々のおかげもあり、pythonで始めて実装することができました。
ありがとうございます。

お忙しい中とは存じますが、ご評価のほどよろしくお願いいたします。
