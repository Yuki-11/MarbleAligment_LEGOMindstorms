# TTI (2021/10/8)  Yuki Kondo
# -*- coding: utf-8 -*-
# MIT Licence

#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from random import shuffle 
from time import sleep, time


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.
# https://pybricks.github.io/ev3-micropython/index.html


# Write your program here.

class Alignment:
    """
    ビー玉整列クラス
    """

    def __init__(self, colSensPort, motorPort, debug=False):
        """
        初期設定メソッド
        """

        """オブジェクト定義"""
        self.ev3 = EV3Brick() # EV3オブジェクトの定義
        self.colSensor = ColorSensor(colSensPort) # カラーセンサーオブジェクトの定義
        self.motorL = Motor(motorPort) # Lモーターオブジェクトの定義

        """ボックスに関する変数の定義"""
        self.targetList = [0, 0, 0, 0, 0, 0, 0, 0, 0]  # 目標配列のリスト
        self.boxList = [0, 0, 0, 0, 0, 0, 0, 0, 0] # リアルタイムなシステムボックス配列のリスト
        #(旧設定)         self.convIdx = [[0, 5, 7], [1, 3, 8], [2, 4, 6]] # 3色それぞれの目標配列でのインデックス，目標配列設定時に使用
        # self.convIdx = [[0, 3, 8], [1, 5, 6], [2, 4, 7]] # 3色それぞれの目標配列でのインデックス，目標配列設定時に使用
        self.convIdx = [[0, 4, 8], [1, 3, 7], [2, 5, 6]] # 3色それぞれの目標配列でのインデックス，目標配列設定時に使用
        self.iniIdx = -1 # 入力部(センサーAssy)の初期位置
        self.idx2deg = {-2:-30, -1:0, 0:51, 1:111, 2:169, 3:232,
                       4:290, 5:350, 6:411, 7:472, 8:527, 9:615}
        self.trigger1Idx = 9.2 # 排出部ゲート解放用トリガー位置
        self.trigger2Idx = -1 # 排出部ゲート閉塞用トリガー位置
        self.storingIdx = 100 # ビー玉格納対象となる位置(初期値:100 現実的には大きすぎる値を初期値とする)
        self.currentIdx = self.iniIdx # 入力部の現在の位置
        self.old_dict=1

        """センサに関する変数の定義"""
        self.colNum = 0 # 検出した色番号．（None:0, Red:1, Green:2, Blue:3）
        self.colLabel = [None, Color.RED, Color.GREEN, Color.BLUE] # センサーオブジェクトが返す色オブジェクトを色番号に変換するためのリスト

        """Lモータに関する変数の定義"""
        self.c_rotate = 148*2*1*0.9/4*0.901     # 1ボックス移動分の回転角度*ギア比*損失係数 [deg]
        self.speed_rotate = 360*160*60 # モータの角速度(1回転*角速度*s/min) [deg/s]

        """ポート番号設定・デバッグモードの設定"""
        self.ports = [colSensPort, motorPort] # デバイスごとのポートの定義[カラーセンサ，Lモータ]
        self.debug = debug # デバッグフラグ
        if self.debug: # デバッグフラグが立つとき
            self.makeVarInput() # 仮想入力配列作成メソッドの実行

        """その他 表示およびブザー"""
        print("[debug] Initialized.") 
        self.ev3.speaker.beep(duration=300) # 処理開始時のビープ音
        self.start = time() # タイム測定開始
        self.motorL.reset_angle(0) # モータの角度をリセット
        print("[debug] degree {}".format(self.motorL.angle()))
        
    def makeVarInput(self):
        """
        仮想入力配列作成メソッド
        (self.degub==Trueのときのみ実行される)
        """
        self.varInput =[1 , 1, 1, 2, 2, 2, 3, 3, 3] # 3色のラベルを3個ずつ準備
        shuffle(self.varInput) # リスト内でランダムにシャッフルし，更新
        print("[debug] Vartual input is {}".format(self.varInput)) 

    def colorDetect(self):
        """
        色検出メソッド
        
        カラーセンサで色を検出し，所定のカラーラベル(番号)に変換する．
        この色をself.colNumに保存する．
        """
        
        # デバッグモードの場合
        if self.debug:
           print("[debug] Remaining Vartual input is {}".format(self.varInput))
           self.colNum = self.varInput.pop(0) # 色番号を仮想入力配列の先頭から削除して取得        

        # 通常モード(実機検証)の場合
        else:
            colorObj = 0 # 色オブジェクトの初期化
            while not colorObj in self.colLabel[1:4]: # R,G,Bの色が検出されるまで
                # colorObj = self.colSensor.color() # カラーセンサーの検出結果を取得
                colorObj = self.colorClf()
            self.colNum = self.colLabel.index(colorObj) # カラーオブジェクトをカラーラベルに変換
        
        self.ev3.speaker.beep() # 色検出時のビープ音
        print("[debug] Color senser detected {}".format(self.colNum))

    def colorClf(self):
        n=8
        cnt=0
        while True:
            result_sum = [0, 0, 0]
            result_avg = [0, 0, 0]
            for i in range(n):
                result = self.colSensor.rgb()
                result_sum = [x+y for x, y in zip(result, result_sum)]
            result_avg = [round(x/n,3) for x in result_sum]
            # print(result_avg)
            if result_avg[1]>11 and 10>result_avg[0] and 10>result_avg[2]:
                print("rgb", result_avg)
                return Color.GREEN
            if 50>result_avg[0]>10 and 10>result_avg[1]:
                print("rgb", result_avg)
                return Color.RED
            if result_avg[2] > 40 and 10>result_avg[0]:
                print("rgb", result_avg)
                return Color.BLUE
            if cnt > 300:
                print("None", result_avg)
                cnt=0
            cnt+=1


    def listSet(self):
        """
        目標配列設定メソッド

        初めて検出された色から順にself.targetListを設定する．
        """
        for i in range(3):  # 0~2のインデックスに1色ずつ配置
            if self.targetList[i]==0 and self.targetList.count(self.colNum)==0:  # 目標配列の該当位置に色が指定されておらず，その色自体も初検出の時
                for j in self.convIdx[i]: # 3色それぞれに指定される3箇所のインデックスを取得
                    self.targetList[j] = self.colNum # 目標配列の該当インデックスの場所に色ラベルを設定
                print("[debug] Target list change {}".format(self.targetList))
                print("[debug] Output 2 axis array's expression")

                """(旧設定)# 設定後の目標配列の2次元配列表示
                for j in range(0,7,3):
                    print(f"        {self.targetList[j:j+3]}")
                break"""

                # 設定後の目標配列の2次元配列表示
                target2dimArray =[[8, 6, 5], [7, 4, 2], [4, 1, 0]]
                print(self.targetList)
                for i in range(0,3):
                    for j in range(0,3):
                        target2dimArray[i][j] = self.targetList[target2dimArray[i][j]]

                for i in range(0,3):
                    print("        {}".format(target2dimArray[i]))

    def getStoringIdx(self):
        """
        格納インデックス取得メソッド

        self.targetListからself.colNum(検出色)のインデックスをすべて取得し，
        (インデックスが若い場所を優先した)未格納箇所のインデックスを取得する．
        """
        colIdxs = [i for i, x in enumerate(self.targetList) if x == self.colNum] # 検出色が格納されるインデックスをすべて取得
        for colIdx in colIdxs: # 取得したインデックスを番号の若い順に検証
            if self.boxList[colIdx]==0: # ボックスにビー玉が存在しない場合
                self.storingIdx = colIdx # 格納場所のインデックスとして取得する．
                print("[debug] Get target index {}".format(self.storingIdx))
                break # forループを終了
    
    def move_store(self):
        """
        格納用移動メソッド

        現在のインデックスと格納箇所のインデックスの差から，モータの回転角度を
        計算し，モータを作動．入力部を格納場所まで移動する．
        """        
        # targetAngle = self.c_rotate*(self.storingIdx-self.currentIdx)  # 目標角度を計算
        targetAngle = self.idx2deg[self.storingIdx]+7
        if self.storingIdx-self.currentIdx<0:
            targetAngle = self.idx2deg[self.storingIdx]-9
        # if self.old_dict*targetAngle<0: 
        #     if targetAngle<0:
        #         targetAngle*=1.1
        #     if targetAngle>0:
        #         targetAngle*=1.13
        #     print("debug reglation")
        
        self.motorL.run_target(self.speed_rotate, targetAngle, then=Stop.HOLD) # 目標角度まで回転(停止時はホールド)
        # sleep(abs(targetAngle/self.speed_rotate)) # debug用 実装時はコメントアウト
        print("[debug] Moved  {} ==> {}".format(self.currentIdx, self.storingIdx))
        print("[debug] degree {}".format(self.motorL.angle()))
        # if self.old_dict*targetAngle<0 and targetAngle<0:
        #     if targetAngle<0:
        #         targetAngle/=1.1
        #     if targetAngle>0:
        #         targetAngle/=1.13
        # self.old_dict=targetAngle
        self.currentIdx = self.storingIdx  # 格納箇所のインデックスを現在の場所を表すインデックスとして更新
        
    def check_storing(self):
        """
        全格納確認メソッド

        全ビー玉が格納されているかを確認する．未格納の場合はビー玉の安定化のために一時停止．
        全格納時は出力部ゲート解放用のトリガー1を押すメソッドを実行．
        """
        self.boxList[self.currentIdx] = self.colNum  # 現在のボックスリストに格納した色ラベルで更新
        print("[debug] Box list is {}.".format(self.storingIdx))
        
        # 全格納時
        if self.boxList.count(0) == 0:
            self.storingIdx=9 
            print("[debug] All marbles stored.")
            self.move_trigger1() # トリガー1用移動メソッドの実行

        # ビー玉残存時
        else:
            print("[debug] Waiting for Stabilization.\n")
            wait(300) # ビー玉のボックス挿入時間および検査用ビー玉の安定化時間
            # sleep(1) # debug用 実装時はコメントアウト
            

    def move_trigger1(self):
        """
        トリガー1用移動メソッド

        現在のインデックスとトリガー1のインデックスの差から，モータの回転角度を
        計算し，モータを作動．ボックスがトリガー1を押すまで移動する．
        """
        targetAngle = self.idx2deg[self.storingIdx]
        # targetAngle = self.c_rotate*(self.trigger1Idx-self.currentIdx)  # 目標角度を計算
        # self.motorL.reset_angle(0) # モータの角度をリセット
        self.motorL.run_target(self.speed_rotate, targetAngle, then=Stop.HOLD) # 目標角度まで回転(停止時はホールド)
        wait(300) # debug用 実装時はコメントアウト
        print("[debug] Switched Trigger1.")
        self.storingIdx=8 
        self.release()  # ビー玉解放メソッドの実行

    def release(self):
        """
        ビー玉解放メソッド

        格納されたビー玉を，トリガー1から近い順番(すなわち後ろ側から順番)に
        モータでボックスを移動させることで，出力部から排出する．
        """
        # targetAngle = self.c_rotate*(8-self.trigger1Idx)  # 目標角度を計算
        targetAngle = self.idx2deg[self.storingIdx]
        # self.motorL.reset_angle(0) # モータの角度をリセット
        self.motorL.run_target(self.speed_rotate, targetAngle, then=Stop.HOLD) # 目標角度まで回転(停止時はホールド)
        print("[debug] Released 8's marble.(colorClass:{})".format(self.boxList[0]))
        wait(500) # 次のビー玉排出までの間隔
        # sleep(0.5) # debug用 実装時はコメントアウト
        # targetAngle = -self.c_rotate # 1ボックスごとに逆方向に移動．

        for i in range(7,-1,-1):    # 8番目~1番目のボックスのビー玉排出
            # self.motorL.reset_angle(0) # モータの角度をリセット
            targetAngle = self.idx2deg[i]
            self.motorL.run_target(self.speed_rotate, targetAngle, then=Stop.HOLD) # 目標角度まで回転(停止時はホールド)
            print("[debug] Released {}'s marble.(colorClass:{})".format(i,self.boxList[i]))
            wait(500) # 次のビー玉排出までの間隔
            # sleep(0.5) # debug用 実装時はコメントアウト

        # 全ビー玉の排出完了
        elapsed_time = time() - self.start # タイム測定終了
        print ("elapsed_time:{:.5}".format(elapsed_time) + "[sec]") # 測定タイムの表示
        print("[debug] All marbles released.")
        self.initialize() # 初期化メソッド

    def initialize(self):
        """
        初期化メソッド

        先頭インデックスのボックスとトリガー2のインデックスの差から，モータの回転角度を
        計算し，モータを作動．ボックスがトリガー2を押すまで移動する．これにより，出力部の
        ゲートが閉塞．また，ボックスを初期位置に戻すとともに，初期設定メソッドを実行することで
        次のビー玉セットの整列を実行できるようにする．
        """
        # トリガー2の作動工程
        targetAngle = self.idx2deg[-2]
        # targetAngle = self.c_rotate*(self.trigger2Idx-0)  # 目標角度を計算
        # self.motorL.reset_angle(0) # モータの角度をリセット
        self.motorL.run_target(self.speed_rotate, targetAngle, then=Stop.HOLD) # 目標角度まで回転(停止時はホールド)
        # sleep(300)) # debug用 実装時はコメントアウト
        print("[debug] Switched Trigger2.")
        wait(200) # Trigger2の作動待機時間
        # sleep(0.2) # debug用 実装時はコメントアウト

        # 初期位置への移動・変数初期化
        targetAngle = self.idx2deg[-1]
        # targetAngle = self.c_rotate*(self.iniIdx-self.trigger2Idx)  # 目標角度を計算
        # self.motorL.reset_angle(0) # モータの角度をリセット
        self.motorL.run_target(self.speed_rotate, targetAngle, then=Stop.HOLD) # 目標角度まで回転(停止時はホールド)
        self.__init__(self.ports[0], self.ports[1], debug=self.debug) # 初期設定メソッドの実行


"""=====================================  main Program  ================================================"""
almt = Alignment(Port.S1, Port.A, debug=False) # Alignmentオブジェクトの生成
for j in range(3): # 3回試行
    print("===========The {}'s try==============".format(j+1))
    

    for i in range(9): # ビー玉の個数
        almt.colorDetect() # 色検出メソッドの実行
        almt.listSet() # 目標配列設定メソッド
        almt.getStoringIdx() # 格納インデックス取得メソッド
        almt.move_store() # 格納用移動メソッド
        almt.check_storing() # 全格納確認メソッド