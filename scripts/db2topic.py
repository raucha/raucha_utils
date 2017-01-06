#! /usr/bin/env python
# -*- coding:utf-8 -*-

import os
import pymongo
import rospy
import rosmsg_utils as ru


"""
mongodbからJSONデータを読み取り，存在する全てのカラムをrosのトピックとして発行
<JSONのフォーマット>
{
    "topic_name":"トピック名",
    "type":"メッセージの型"
    ”rostime”:"mongodbに登録した時刻. rospy.get_time()"
    "msg":{"メッセージの内容"}
}
"""

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


if __name__ == '__main__':
    print "Hello World!"
    rospy.init_node('db2topic')

    # DBのへの接続
    CO = getDBConnection()

    # DBに登録されているトピック名を取得
    ## TOPICS: (トピック名, メッセージ型)
    TOPICS = [(n["topic_name"], n["type"]) for n in list(CO.find())]

    # トピック名ごとにパブリッシャーを生成
    pub_stats = {}
    for i in TOPICS:
        nm = i[0]
        # pkg, typ = i[1].split("/")
        # メッセージの型をインポート
        msgtype = ru.str2msgtype(i[1])
        pub = rospy.Publisher(nm, msgtype, queue_size=3)
        pub_stat = {"publisher": pub, "last_time": 0.0}
        pub_stats[nm] = pub_stat

    # DBを監視して，更新があればパブリッシュ
    r = rospy.Rate(5)  # 5hz
    while not rospy.is_shutdown():
        r.sleep()
        db_datas = list(CO.find())
        if not db_datas:  # 空の時
            continue
        for db_data in db_datas:
            nm = db_data["topic_name"]
            topic_time = db_data["rostime"]
            if not db_data["topic_name"] in pub_stats:
                # 新しいトピック名の挿入
                # @TODO: pub_statsに挿入の処理
                pass
            if topic_time > pub_stats[nm]["last_time"]:
                # 更新あり，データをパブリッシュ
                msg = ru.getPublishMsg(db_data["type"], db_data["msg"])
                pub_stats[nm]["publisher"].publish(msg)
                pub_stats[nm]["last_time"] = topic_time
