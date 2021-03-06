/*********************************************************************
* FSRobo-R Package BSDL
* ---------
* Copyright (C) 2019 FUJISOFT. All rights reserved.
* 
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
* 1. Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation and/or
* other materials provided with the distribution.
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software without
* specific prior written permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "fsrobo_r_driver/simple_message/messages/get_posture_message.h"
#include "simple_message/byte_array.h"
#include "simple_message/log_wrapper.h"

using industrial::byte_array::ByteArray;
using industrial::simple_message::SimpleMessage;
using fsrobo_r_driver::simple_message::posture::Posture;

namespace fsrobo_r_driver
{
namespace simple_message
{
namespace get_posture_message
{

GetPostureMessage::GetPostureMessage(void)
{
  this->init();
}

GetPostureMessage::~GetPostureMessage(void)
{
}

bool GetPostureMessage::init(SimpleMessage & msg)
{
  ByteArray data = msg.getData();
  this->init();

  if (!data.unload(this->cmd_))
  {
    LOG_ERROR("Failed to unload Posture data");
    return false;
  }

  return true;
}

void GetPostureMessage::init(Posture & cmd)
{
  this->init();
  this->cmd_.copyFrom(cmd);
}

void GetPostureMessage::init()
{
  this->setMessageType(FSRoboRMsgTypes::GET_POSTURE);
  this->cmd_.init();
}

bool GetPostureMessage::load(ByteArray *buffer)
{
  LOG_COMM("Executing Posture message load");
  if (!buffer->load(this->cmd_))
  {
    LOG_ERROR("Failed to load Posture message");
    return false;
  }

  return true;
}

bool GetPostureMessage::unload(ByteArray *buffer)
{
  LOG_COMM("Executing Posture message unload");

  if (!buffer->unload(this->cmd_))
  {
    LOG_ERROR("Failed to unload Posture message");
    return false;
  }

  return true;
}

}
}
}