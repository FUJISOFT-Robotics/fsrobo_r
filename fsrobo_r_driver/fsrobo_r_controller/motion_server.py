#!/usr/bin/env python
# -*- coding: utf-8 -*-

# FSRobo-R Package BSDL
# ---------
# Copyright (C) 2019 FUJISOFT. All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation and/or
# other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ---------

import SocketServer
import socket
from struct import pack, unpack
import sys
from simple_message import SimpleMessageSocket, \
    SimpleMessageType, JointTrajPtReplyMessage, CommunicationType, ReplyCode, SimpleMessage, SpecialSequence, \
    SetIOReplyMessage, IOFunctionType, ExecuteProgramReplyMessage, \
    SetPostureReplyMessage, GetPostureReplyMessage, \
    SysStatReplyMessage, \
    SetToolOffsetReplyMessage
from robot_controller import RobotController
import math

HOST, PORT = "0.0.0.0", 11000

class MotionTCPHandler(SocketServer.BaseRequestHandler):
    def handle(self):
        #client = self.request
        sock = SimpleMessageSocket(self.request)
        address = self.client_address[0]
        ctl = RobotController()

        msg_handlers = {
            SimpleMessageType.JOINT_TRAJ_PT: self.on_joint_traj_pt,
            SimpleMessageType.SET_IO: self.on_set_io,
            SimpleMessageType.EXECUTE_PROGRAM: self.on_execute_program,
            SimpleMessageType.SET_POSTURE: self.on_set_posture,
            SimpleMessageType.GET_POSTURE: self.on_get_posture,
            SimpleMessageType.SYS_STAT: self.on_sys_stat,
            SimpleMessageType.SET_TOOL_OFFSET: self.on_set_tool_offset
        }

        try:
            while True:
                recv_msg = sock.recv()

                msg_handler = msg_handlers.get(
                    recv_msg.msg_type, self.on_unkown_message)

                reply_msg = msg_handler(ctl, recv_msg)
                sock.send(reply_msg)
        finally:
            ctl.close()

    def on_joint_traj_pt(self, ctl, recv_msg):
        joint_deg = map(math.degrees, recv_msg.joint_data[:6])
        print(recv_msg.sequence)
        print(recv_msg.joint_data)

        result = True
        if recv_msg.sequence == SpecialSequence.STOP_TRAJECTORY:
            result = ctl.abort()
            print("abort move command")
        else:
            ctl.set_speed(recv_msg.velocity)
            result = ctl.move(joint_deg)

        reply_msg = JointTrajPtReplyMessage()
        if not result:
            print("move error")
        reply_msg.reply_code = ReplyCode.SUCCESS if result else ReplyCode.FAILURE

        return reply_msg

    def on_set_io(self, ctl, recv_msg):
        print(recv_msg.fun)
        print(recv_msg.address)
        print(recv_msg.data_size)
        print(recv_msg.data)

        if recv_msg.fun == IOFunctionType.SET_DIGITAL_OUT:
            result = ctl.set_dio(
                recv_msg.address, recv_msg.data[0:recv_msg.data_size])
        elif recv_msg.fun == IOFunctionType.SET_ADC_MODE:
            result = ctl.set_adc_mode(recv_msg.address, recv_msg.data[0])

        reply_msg = SetIOReplyMessage()
        reply_msg.reply_code = ReplyCode.SUCCESS if result else ReplyCode.FAILURE
        reply_msg.result = reply_msg.Result.SUCCESS if result else reply_msg.Result.FAILURE
        print("received: I/O message")

        return reply_msg

    def on_execute_program(self, ctl, recv_msg):
        print(recv_msg.name)
        print(recv_msg.param)
        print("received: Execute program message")

        result = ctl.exec_program(recv_msg.name, recv_msg.param)

        reply_msg = ExecuteProgramReplyMessage()
        reply_msg.reply_code = ReplyCode.SUCCESS
        reply_msg.result = reply_msg.Result.SUCCESS if result else reply_msg.Result.FAILURE

        return reply_msg

    def on_set_posture(self, ctl, recv_msg):
        print(recv_msg.posture)
        print("received: Set posture message")

        result = ctl.set_posture(recv_msg.posture)

        reply_msg = SetPostureReplyMessage()
        reply_msg.reply_code = ReplyCode.SUCCESS if result else ReplyCode.FAILURE

        return reply_msg

    def on_get_posture(self, ctl, recv_msg):
        print("received: Get posture message")

        posture = ctl.get_posture()

        reply_msg = GetPostureReplyMessage()
        if posture == None:
            reply_msg.reply_code = ReplyCode.FAILURE
        else:
            reply_msg.reply_code = ReplyCode.SUCCESS
            reply_msg.posture = posture

        return reply_msg

    def on_sys_stat(self, ctl, recv_msg):
        print(recv_msg.stat_type)

        result = ctl.sys_stat(recv_msg.stat_type)

        reply_msg = SysStatReplyMessage()
        reply_msg.reply_code = ReplyCode.SUCCESS if result else ReplyCode.FAILURE
        print("result: {}".format(result))
        reply_msg.result = result
        print("received: sys_stat message")

        return reply_msg

    def on_set_tool_offset(self, ctl, recv_msg):
        print("received: Set tool offset message")
        result = ctl.set_tool_offset(recv_msg.x, recv_msg.y, recv_msg.z,
            recv_msg.rz, recv_msg.ry, recv_msg.rx)
        print("result: {}".format(result))
        
        reply_msg = SetToolOffsetReplyMessage()
        reply_msg.reply_code = SetToolOffsetReplyMessage.Result.SUCCESS if result else SetToolOffsetReplyMessage.Result.FAILURE

        return reply_msg

    def on_unkown_message(self, ctl, recv_msg):
        raise NotImplementedError(
            "Unknown msg_type: {}".format(recv_msg.msg_type))


class MotionServer(SocketServer.ForkingTCPServer, object):
    def server_bind(self):
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(self.server_address)


if __name__ == "__main__":
    server = MotionServer((HOST, PORT), MotionTCPHandler)
    server.serve_forever()
