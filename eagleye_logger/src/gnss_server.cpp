// Copyright (c) 2020, Map IV, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the Map IV, Inc. nor the names of its contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "ros/ros.h"
#include "eagleye_logger/gnss_server.h"

/**
 * @brief Constructor
 */
GnssServer::GnssServer()
{
  port = 0;
}

GnssServer::~GnssServer()
{
  close(sock);
  close(fd_);
}

void GnssServer::open(int _port)
{
  port = _port;

  sock = socket(AF_INET, SOCK_STREAM, 0);

  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = INADDR_ANY;
  if(-1 == bind(sock, (struct sockaddr *)&addr, sizeof(addr)) )
  {
    perror("bind");
  }

  if(-1 == listen(sock, 5) )
  {
    perror("listen");
  }

  /* non-blocking socket */
  int val = 1;
  ioctl(sock, FIONBIO, &val);

}


void GnssServer::isConnect()
{

  if(0 >= fd_){
    len = 0;
    socklen_t sockerlen;
    fd_ = accept(sock, (struct sockaddr *)&client, &sockerlen);
    fcntl(fd_, F_SETFL, O_NONBLOCK);
  }

  if((0 < fd_)&&(len > 0)){
    /* connected */
    int write_len;
    write_len = write(fd_, sendbuff, len);
    len = 0;
    if(-1 == write_len){
      fd_ = 0;
    }
  }
#if 0 /* Send Packet Print */
  for(int h=0; h < len;h++){
    printf("%02X ", sendbuff[h]);
  }
  len = 0;
#endif
}

void GnssServer::setSendbuf(unsigned char* buf, int length)
{
  memcpy( &sendbuff[len], &buf[0], length );
  len += length;
}
