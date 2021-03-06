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

#include "fsrobo_r_driver/simple_message/messages/io_state_message.h"
#include "fsrobo_r_driver/simple_message/io_state.h"
#include "simple_message/byte_array.h"
#include "simple_message/log_wrapper.h"

using namespace industrial::shared_types;
using namespace industrial::byte_array;
using namespace industrial::simple_message;

namespace fsrobo_r_driver
{
namespace simple_message
{
namespace io_state_message
{

IOStateMessage::IOStateMessage(void)
{
  this->init();
}

IOStateMessage::~IOStateMessage(void)
{

}

bool IOStateMessage::init(industrial::simple_message::SimpleMessage & msg)
{
  bool rtn = false;
  ByteArray data = msg.getData();
  this->init();
  this->setCommType(msg.getCommType());

  if (data.unload(this->state_))
  {
    rtn = true;
  }
  else
  {
    LOG_ERROR("Failed to unload I/O state data");
  }
  return rtn;
}

void IOStateMessage::init(fsrobo_r_driver::simple_message::io_state::IOState &state)
{
  this->init();
  this->state_.copyFrom(state);
}

void IOStateMessage::init()
{
  this->setMessageType(FSRoboRMsgTypes::IO_STATE);
  this->state_.init();
}

bool IOStateMessage::load(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing I/O state message load");
  if (buffer->load(this->state_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to load I/O state data");
  }
  return rtn;
}

bool IOStateMessage::unload(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing I/O state message unload");

  if (buffer->unload(this->state_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to unload I/O state data");
  }
  return rtn;
}

}
}
}

