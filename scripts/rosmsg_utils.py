#! /usr/bin/env python
# -*- coding:utf-8 -*-

from subprocess import check_output
import traceback
import rospy
import genpy


def isPrimirive(cls):
    return not hasattr(cls, "__slots__") and not hasattr(cls, "__dict__")


def isTimeType(cls_instance):
    '''
    stampの型だけは特殊で，２種類のクラスの型があるため，
    個別に判定関数を利用する
    '''
    time_types = [rospy.rostime.Time, genpy.rostime.Time]
    return type(cls_instance) in time_types


def getTopicType(topic_name):
    try:
        ret = check_output("rostopic type {}".format(
            topic_name).split(" ")).rstrip()
    except Exception as e:
        print traceback.format_exc(),
        print "topicの型の検出に失敗", " topicname:", topic_name
        exit()
    return ret


def msg2dict(msg_instance):
    # プリミティブ型だったらそのままリターン
    if isPrimirive(msg_instance):
        return msg_instance
    # そうでなければ再起呼び出し
    elif hasattr(msg_instance, "__dict__"):
        # rosのmsgは__dict__を持たない
        raise Exception("error. class has __dict__. data:{}".format(
            msg_instance.__dict__))
    ret = {}
    for n in msg_instance.__slots__:
        if "stamp" == n and isTimeType(getattr(msg_instance, n)):
            ret["stamp"] = {"secs": getattr(msg_instance, n).secs,
                            "nsecs": getattr(msg_instance, n).nsecs}
        else:
            # print type(n), n
            # ret.update({n: None})
            ret[n] = msg2dict(getattr(msg_instance, n))
    return ret


def dict2msg(msg_instance, dic):
    # プリミティブ型だったらそのままリターン
    if isPrimirive(msg_instance):
        return dic
    # そうでなければ再起呼び出し
    elif hasattr(msg_instance, "__dict__"):
        # rosのmsgは__dict__を持たない
        # print "error. class has __dict__", msg_instance.__dict__
        raise Exception("error. class has __dict__. type:{} data:{}".format(
            type(msg_instance), msg_instance.__dict__))
    for n in msg_instance.__slots__:
        # print "msginstance: ", msg_instance
        # print "dic: ", dic
        # print
        setattr(msg_instance, n,
                dict2msg(getattr(msg_instance, n), dic[n]))
        # exec("msg_instance.{0} = dict2msg(msg_instance.{0}, dic[n])".format(n))
    return msg_instance


def str2msgtype(arg):
    pkg, typ = arg.split("/")
    # メッセージの型をインポート
    code = "from {}.msg import {}".format(pkg, typ)
    exec(code, globals())
    exec("solid_type = " + typ)
    return solid_type


def getPublishMsg(type_str, msg):
    msg_class = str2msgtype(type_str)
    return dict2msg(msg_class(), msg)


def isEqualRosMsg(msg1, msg2):
    # if type(msg1) != type(msg2):
    #     raise Exception("msg1 data:{} type:{},  msg2 data:{} type:{}".format(
    #         msg1, type(msg1), msg2, type(msg2)))
    #     # return False
    if isPrimirive(msg1):
        return msg1 == msg2
    # if type(list()) == type(msg1):
    #     return msg1==msg2
    ret = True
    for n in msg1.__slots__:
        if isTimeType(getattr(msg1, n)):
            ret = ret and (getattr(msg1, n).secs == getattr(msg2, n).secs)
            ret = ret and (getattr(msg1, n).nsecs == getattr(msg2, n).nsecs)
        else:
            ret = ret and isEqualRosMsg(getattr(msg1, n), getattr(msg2, n))
    return ret
