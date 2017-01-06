#! /usr/bin/env python
# -*- coding:utf-8 -*-

import os
import pymongo
import rospy
import rosmsg_utils as ru


"""
rosのトピックを購読し，JSONに変換してmongodbに保存
各トピック名につき，最新のデータのみを保存したカラムが１行だけ保存されれる．
最新以外のデータは削除される．
<JSONのフォーマット>
{
    "topic_name":"トピック名",
    "type":"メッセージの型"
    ”rostime”:"mongodbに登録した時刻. rospy.get_time()"
    "msg":{"メッセージの内容"}
}
"""

CO = None

def getDBConnection():
    user = os.environ.get("ROS_MONGO_USER")
    if not user:
        rospy.logwarn("環境変数 'ROS_MONGO_USER' が見つかりません．手動入力モードに移行")
        user = raw_input("UserName: ")
    password = os.environ.get("ROS_MONGO_PASS")
    if not password:
        rospy.logwarn("環境変数 'ROS_MONGO_PASS' が見つかりません．手動入力モードに移行")
        password = raw_input("Password: ")

    db_address = rospy.get_param("~DB_ADDRESS", "ds061474.mlab.com:61474")
    db_name = rospy.get_param("~DB_NAME", "first_db")
    db_collection = rospy.get_param("~DB_COLLECTION", "bsen2wc")
    path = [db_address, db_name, db_collection]
    auth = [user, password]
    client = pymongo.MongoClient(
        'mongodb://{auth[0]}:{auth[1]}@{path[0]}/{path[1]}'.format(auth=auth, path=path))
    db = client[path[1]]
    co = db[path[2]]
    return co


def callback(topic_name, topic_type, arg):
    # print "get callback", topic_name, topic_type, msg
    msg = ru.msg2dict(arg)
    global CO
    db_data = {}
    db_data["msg"] = msg
    db_data["rostime"] = float(rospy.get_time())
    db_data["topic_name"] = topic_name
    db_data["type"] = topic_type
    # print db_data
    QUERY = {"topic_name": topic_name}
    col_num = len(list(CO.find(QUERY)))
    print "topic名={}の個数:{}".format(topic_name, col_num)
    if 0 == col_num:
        # データ挿入
        CO.insert_one(db_data)
    elif 1 == col_num:
        # データ書き換え
        CO.update(QUERY, db_data)
    elif 1 < col_num:
        # 何故か複数個ある．全部削除して挿入
        print "同じトピック名のカラムが何故か複数個あります！！！"
        CO.remove(QUERY)
        CO.insert_one(db_data)


if __name__ == '__main__':
    rospy.init_node('topic2db')
    rospy.loginfo("Hello World!")

    # DBへの接続
    CO = getDBConnection()
    # DBのデータを全削除
    global CO
    CO.remove()

    # DBに登録するトピック名を取得
    TOPIC_NAMES = rospy.get_param("~topic_names")
    # TOPIC_NAMES = ["/test", "/test2"]
    if not TOPIC_NAMES:
        # トピック名が空の時
        rospy.logwarn("rosparam 'topic_names' is enmpy!!")
        exit()
    # 各トピック名に対するサブスクライバーを生成
    for name in TOPIC_NAMES:
        # メッセージの型を取得
        topic_type = ru.getTopicType(name)
        msgtype = ru.str2msgtype(topic_type)
        # 購読時の関数を登録
        ## name=nameを入れないと現在値が保存されない
        callback_ = lambda x, name=name, topic_type=topic_type: callback(name, topic_type, x)
        rospy.Subscriber(name, msgtype, callback_)
    rospy.spin()