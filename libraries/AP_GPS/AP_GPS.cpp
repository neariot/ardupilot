// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "AP_GPS.h"
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_HAL/AP_HAL.h>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <sys/select.h>

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>
#include <arpa/inet.h>

#define BUFFER_LENGTH 100

class SocketVision {
public:
  SocketVision(bool _datagram);
  SocketVision(bool _datagram, int _fd);
  ~SocketVision();

  bool connect(const char *address, uint16_t port);
  bool bind(const char *address, uint16_t port);
  void reuseaddress();
  void set_blocking(bool blocking);
  void set_broadcast(void);

  ssize_t send(const void *pkt, size_t size);
  ssize_t sendto(const void *buf, size_t size, const char *address,
                 uint16_t port);
  ssize_t recv(void *pkt, size_t size, uint32_t timeout_ms);

  // return the IP address and port of the last received packet
  void last_recv_address(const char *&ip_addr, uint16_t &port);

  // return true if there is pending data for input
  bool pollin(uint32_t timeout_ms);

  // return true if there is room for output data
  bool pollout(uint32_t timeout_ms);

  // start listening for new tcp connections
  bool listen(uint16_t backlog);

  // accept a new connection. Only valid for TCP connections after
  // listen has been used. A new socket is returned
  SocketVision *accept(uint32_t timeout_ms);

private:
  bool datagram;
  struct sockaddr_in in_addr {};

  int fd = -1;

  void make_sockaddr(const char *address, uint16_t port,
                     struct sockaddr_in &sockaddr);
};

SocketVision::SocketVision(bool _datagram)
    : SocketVision(_datagram,
                   socket(AF_INET, _datagram ? SOCK_DGRAM : SOCK_STREAM, 0)) {}

SocketVision::SocketVision(bool _datagram, int _fd)
    : datagram(_datagram), fd(_fd) {
  fcntl(fd, F_SETFD, FD_CLOEXEC);
  if (!datagram) {
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
  }
}

SocketVision::~SocketVision() {
  if (fd != -1) {
    ::close(fd);
    fd = -1;
  }
}

void SocketVision::make_sockaddr(const char *address, uint16_t port,
                                 struct sockaddr_in &sockaddr) {
  memset(&sockaddr, 0, sizeof(sockaddr));

#ifdef HAVE_SOCK_SIN_LEN
  sockaddr.sin_len = sizeof(sockaddr);
#endif
  sockaddr.sin_port = htons(port);
  sockaddr.sin_family = AF_INET;
  sockaddr.sin_addr.s_addr = inet_addr(address);
}

/*
  connect the socket
 */
bool SocketVision::connect(const char *address, uint16_t port) {
  struct sockaddr_in sockaddr;
  make_sockaddr(address, port, sockaddr);

  if (::connect(fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0) {
    return false;
  }
  return true;
}

/*
  bind the socket
 */
bool SocketVision::bind(const char *address, uint16_t port) {
  struct sockaddr_in sockaddr;
  make_sockaddr(address, port, sockaddr);

  if (::bind(fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0) {
    return false;
  }
  return true;
}

/*
  set SO_REUSEADDR
 */
void SocketVision::reuseaddress(void) {
  int one = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
}

/*
  set blocking state
 */
void SocketVision::set_blocking(bool blocking) {
  if (blocking) {
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) & ~O_NONBLOCK);
  } else {
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
  }
}

/*
  send some data
 */
ssize_t SocketVision::send(const void *buf, size_t size) {
  return ::send(fd, buf, size, 0);
}

/*
  send some data
 */
ssize_t SocketVision::sendto(const void *buf, size_t size, const char *address,
                             uint16_t port) {
  struct sockaddr_in sockaddr;
  make_sockaddr(address, port, sockaddr);
  return ::sendto(fd, buf, size, 0, (struct sockaddr *)&sockaddr,
                  sizeof(sockaddr));
}

/*
  receive some data
 */
ssize_t SocketVision::recv(void *buf, size_t size, uint32_t timeout_ms) {
  if (!pollin(timeout_ms)) {
    return -1;
  }
  socklen_t len = sizeof(in_addr);
  return ::recvfrom(fd, buf, size, MSG_DONTWAIT, (sockaddr *)&in_addr, &len);
}

/*
  return the IP address and port of the last received packet
 */
void SocketVision::last_recv_address(const char *&ip_addr, uint16_t &port) {
  ip_addr = inet_ntoa(in_addr.sin_addr);
  port = ntohs(in_addr.sin_port);
}

void SocketVision::set_broadcast(void) {
  int one = 1;
  setsockopt(fd, SOL_SOCKET, SO_BROADCAST, (char *)&one, sizeof(one));
}

/*
  return true if there is pending data for input
 */
bool SocketVision::pollin(uint32_t timeout_ms) {
  fd_set fds;
  struct timeval tv;

  FD_ZERO(&fds);
  FD_SET(fd, &fds);

  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000UL;

  if (select(fd + 1, &fds, NULL, NULL, &tv) != 1) {
    return false;
  }
  return true;
}

/*
  return true if there is room for output data
 */
bool SocketVision::pollout(uint32_t timeout_ms) {
  fd_set fds;
  struct timeval tv;

  FD_ZERO(&fds);
  FD_SET(fd, &fds);

  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000UL;

  if (select(fd + 1, NULL, &fds, NULL, &tv) != 1) {
    return false;
  }
  return true;
}

/*
   start listening for new tcp connections
 */
bool SocketVision::listen(uint16_t backlog) {
  return ::listen(fd, (int)backlog) == 0;
}

/*
  accept a new connection. Only valid for TCP connections after
  listen has been used. A new socket is returned
*/
SocketVision *SocketVision::accept(uint32_t timeout_ms) {
  if (!pollin(timeout_ms)) {
    return NULL;
  }

  int newfd = ::accept(fd, NULL, NULL);
  if (newfd == -1) {
    return NULL;
  }
  // turn off nagle for lower latency
  int one = 1;
  setsockopt(newfd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
  return new SocketVision(false, newfd);
}

extern const AP_HAL::HAL &hal;

struct vision_pos {
  uint8_t status;
  int32_t alt;
  int32_t lat;
  int32_t lng;
  int32_t lon;
  uint8_t num_sats;
  float ground_speed;       ///< ground speed in m/sec
  int32_t ground_course_cd; ///< ground course in 100ths of a degree
  uint16_t hdop;            ///< horizontal dilution of precision in cm
  uint16_t vdop;            ///< vertical dilution of precision in cm
  Vector3f velocity;        ///< 3D velocitiy in m/s, in NED format
};

class AP_GPS_VISION : public AP_GPS_Backend {
public:
  AP_GPS_VISION(AP_GPS &_gps, AP_GPS::GPS_State &_state,
                AP_HAL::UARTDriver *_port);
  bool read();

private:
  int _gps_sub;
  struct vision_pos _gps_pos;
  uint8_t buf[BUFFER_LENGTH];
  ssize_t recsize;
  socklen_t fromlen;
  int bytes_sent;
  mavlink_local_position_ned_t pos;
  uint16_t len;
  int i = 0;
  unsigned int temp = 0;
  int sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  struct sockaddr_in gcAddr;
  struct sockaddr_in locAddr;
  char target_ip[100];
};

AP_GPS_VISION::AP_GPS_VISION(AP_GPS &_gps, AP_GPS::GPS_State &_state,
                             AP_HAL::UARTDriver *_port)
    : AP_GPS_Backend(_gps, _state, _port) {

  strcpy(target_ip, "127.0.0.1");
  memset(&locAddr, 0, sizeof(locAddr));
  locAddr.sin_family = AF_INET;
  locAddr.sin_addr.s_addr = INADDR_ANY;
  locAddr.sin_port = htons(12345);
  bind(sock, (struct sockaddr *)&locAddr, sizeof(struct sockaddr));
  fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC);
  memset(&gcAddr, 0, sizeof(gcAddr));
  gcAddr.sin_family = AF_INET;
  gcAddr.sin_addr.s_addr = inet_addr(target_ip);
  gcAddr.sin_port = htons(12345);
}

bool AP_GPS_VISION::read(void) {

  memset(buf, 0, BUFFER_LENGTH);
  recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0,
                     (struct sockaddr *)&gcAddr, &fromlen);
  if (recsize > 0) {
     mavlink_message_t msg;
     mavlink_status_t status;
     for (i = 0; i < recsize; ++i) {
      temp = buf[i];
      if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
        mavlink_local_position_ned_t pos;
        mavlink_msg_local_position_ned_decode(&msg, &pos);
        state.location.lat = pos.x; //_gps_pos.lat;
        state.location.lng = pos.y; //_gps_pos.lon;
        state.location.alt = pos.z; //_gps_pos.alt / 10; 90898
        state.last_gps_time_ms = AP_HAL::millis();
        state.status = (AP_GPS::GPS_Status)3; //_gps_pos.status;
        state.num_sats = 10;
        state.hdop = _gps_pos.hdop;
        state.ground_speed = _gps_pos.ground_speed;
        state.hdop = _gps_pos.hdop;
      }
    }
    memset(buf, 0, BUFFER_LENGTH);
    return true;
  }
  return false;
}

// table of user settable parameters
const AP_Param::GroupInfo AP_GPS::var_info[] = {
  // @Param: TYPE
  // @DisplayName: GPS type
  // @Description: GPS type
  // @Values:
  // 0:None,1:AUTO,2:uBlox,3:MTK,4:MTK19,5:NMEA,6:SiRF,7:HIL,8:SwiftNav,9:PX4-UAVCAN,10:SBF,11:GSOF
  // @RebootRequired: True
  AP_GROUPINFO("TYPE", 0, AP_GPS, _type[0], 1),

  // @Param: TYPE2
  // @DisplayName: 2nd GPS type
  // @Description: GPS type of 2nd GPS
  // @Values:
  // 0:None,1:AUTO,2:uBlox,3:MTK,4:MTK19,5:NMEA,6:SiRF,7:HIL,8:SwiftNav,9:PX4-UAVCAN,10:SBF,11:GSOF
  // @RebootRequired: True
  AP_GROUPINFO("TYPE2", 1, AP_GPS, _type[1], 0),

  // @Param: NAVFILTER
  // @DisplayName: Navigation filter setting
  // @Description: Navigation filter engine setting
  // @Values:
  // 0:Portable,2:Stationary,3:Pedestrian,4:Automotive,5:Sea,6:Airborne1G,7:Airborne2G,8:Airborne4G
  // @RebootRequired: True
  AP_GROUPINFO("NAVFILTER", 2, AP_GPS, _navfilter, GPS_ENGINE_AIRBORNE_4G),

  // @Param: AUTO_SWITCH
  // @DisplayName: Automatic Switchover Setting
  // @Description: Automatic switchover to GPS reporting best lock
  // @Values: 0:Disabled,1:Enabled
  // @User: Advanced
  AP_GROUPINFO("AUTO_SWITCH", 3, AP_GPS, _auto_switch, 1),

  // @Param: MIN_DGPS
  // @DisplayName: Minimum Lock Type Accepted for DGPS
  // @Description: Sets the minimum type of differential GPS corrections
  // required before allowing to switch into DGPS mode.
  // @Values: 0:Any,50:FloatRTK,100:IntegerRTK
  // @User: Advanced
  // @RebootRequired: True
  AP_GROUPINFO("MIN_DGPS", 4, AP_GPS, _min_dgps, 100),

  // @Param: SBAS_MODE
  // @DisplayName: SBAS Mode
  // @Description: This sets the SBAS (satellite based augmentation system)
  // mode if available on this GPS. If set to 2 then the SBAS mode is not
  // changed in the GPS. Otherwise the GPS will be reconfigured to
  // enable/disable SBAS. Disabling SBAS may be worthwhile in some parts of
  // the world where an SBAS signal is available but the baseline is too long
  // to be useful.
  // @Values: 0:Disabled,1:Enabled,2:NoChange
  // @User: Advanced
  // @RebootRequired: True
  AP_GROUPINFO("SBAS_MODE", 5, AP_GPS, _sbas_mode, 2),

  // @Param: MIN_ELEV
  // @DisplayName: Minimum elevation
  // @Description: This sets the minimum elevation of satellites above the
  // horizon for them to be used for navigation. Setting this to -100 leaves
  // the minimum elevation set to the GPS modules default.
  // @Range: -100 90
  // @Units: Degrees
  // @User: Advanced
  // @RebootRequired: True
  AP_GROUPINFO("MIN_ELEV", 6, AP_GPS, _min_elevation, -100),

  // @Param: INJECT_TO
  // @DisplayName: Destination for GPS_INJECT_DATA MAVLink packets
  // @Description: The GGS can send raw serial packets to inject data to
  // multiple GPSes.
  // @Values: 0:send to first GPS,1:send to 2nd GPS,127:send to all
  AP_GROUPINFO("INJECT_TO", 7, AP_GPS, _inject_to, GPS_RTK_INJECT_TO_ALL),

  // @Param: SBP_LOGMASK
  // @DisplayName: Swift Binary Protocol Logging Mask
  // @Description: Masked with the SBP msg_type field to determine whether
  // SBR1/SBR2 data is logged
  // @Values: 0x0000:None, 0xFFFF:All, 0xFF00:External only
  // @User: Advanced
  AP_GROUPINFO("SBP_LOGMASK", 8, AP_GPS, _sbp_logmask, 0xFF00),

  // @Param: RAW_DATA
  // @DisplayName: Raw data logging
  // @Description: Enable logging of RXM raw data from uBlox which includes
  // carrier phase and pseudo range information. This allows for post
  // processing of dataflash logs for more precise positioning. Note that this
  // requires a raw capable uBlox such as the 6P or 6T.
  // @Values: 0:Disabled,1:log every sample,5:log every 5 samples
  // @RebootRequired: True
  AP_GROUPINFO("RAW_DATA", 9, AP_GPS, _raw_data, 0),

  // @Param: GNSS_MODE
  // @DisplayName: GNSS system configuration
  // @Description: Bitmask for what GNSS system to use (all unchecked or zero
  // to leave GPS as configured)
  // @Values: 0:Leave as currently configured, 1:GPS-NoSBAS, 3:GPS+SBAS,
  // 4:Galileo-NoSBAS, 6:Galileo+SBAS, 8:Beidou, 51:GPS+IMES+QZSS+SBAS (Japan
  // Only), 64:GLONASS, 66:GLONASS+SBAS, 67:GPS+GLONASS+SBAS
  // @Bitmask: 0:GPS,1:SBAS,2:Galileo,3:Beidou,4:IMES,5:QZSS,6:GLOSNASS
  // @User: Advanced
  // @RebootRequired: True
  AP_GROUPINFO("GNSS_MODE", 10, AP_GPS, _gnss_mode, 0),

  // @Param: SAVE_CFG
  // @DisplayName: Save GPS configuration
  // @Description: Determines whether the configuration for this GPS should be
  // written to non-volatile memory on the GPS. Currently working for UBlox.
  // @Values: 0:Do not save config,1:Save config
  // @User: Advanced
  AP_GROUPINFO("SAVE_CFG", 11, AP_GPS, _save_config, 0),
  AP_GROUPEND
};

/// Startup initialisation.
void AP_GPS::init(DataFlash_Class *dataflash,
                  const AP_SerialManager &serial_manager) {
  _DataFlash = dataflash;
  primary_instance = 0;

  // search for serial ports with gps protocol
  _port[0] =
      serial_manager.find_serial(AP_SerialManager::SerialProtocol_GPS, 0);
  _port[1] =
      serial_manager.find_serial(AP_SerialManager::SerialProtocol_GPS, 1);
  _last_instance_swap_ms = 0;
}

// baudrates to try to detect GPSes with
const uint32_t AP_GPS::_baudrates[] = { 4800U,  38400U, 115200U,
                                        57600U, 9600U,  230400U };

// initialisation blobs to send to the GPS to try to get it into the
// right mode
const char AP_GPS::_initialisation_blob[] =
    UBLOX_SET_BINARY MTK_SET_BINARY SIRF_SET_BINARY;
const char AP_GPS::_initialisation_raw_blob[] =
    UBLOX_SET_BINARY_RAW_BAUD MTK_SET_BINARY SIRF_SET_BINARY;

/*
  send some more initialisation string bytes if there is room in the
  UART transmit buffer
 */
void AP_GPS::send_blob_start(uint8_t instance, const char *_blob,
                             uint16_t size) {
  initblob_state[instance].blob = _blob;
  initblob_state[instance].remaining = size;
}

/*
  send some more initialisation string bytes if there is room in the
  UART transmit buffer
 */
void AP_GPS::send_blob_update(uint8_t instance) {
  // exit immediately if no uart for this instance
  if (_port[instance] == NULL) {
    return;
  }

  // see if we can write some more of the initialisation blob
  if (initblob_state[instance].remaining > 0) {
    int16_t space = _port[instance]->txspace();
    if (space > (int16_t)initblob_state[instance].remaining) {
      space = initblob_state[instance].remaining;
    }
    while (space > 0) {
      _port[instance]->write(*initblob_state[instance].blob);
      initblob_state[instance].blob++;
      space--;
      initblob_state[instance].remaining--;
    }
  }
}

/*
  run detection step for one GPS instance. If this finds a GPS then it
  will fill in drivers[instance] and change state[instance].status
  from NO_GPS to NO_FIX.
 */
void AP_GPS::detect_instance(uint8_t instance) {
  AP_GPS_Backend *new_gps = NULL;
  struct detect_state *dstate = &detect_state[instance];
  uint32_t now = AP_HAL::millis();

  new_gps = new AP_GPS_VISION(*this, state[instance], _port[instance]);
  if (new_gps != NULL) {
    state[instance].status = NO_FIX;
    drivers[instance] = new_gps;
    timing[instance].last_message_time_ms = now;
  }
}

AP_GPS::GPS_Status AP_GPS::highest_supported_status(uint8_t instance) const {
  if (drivers[instance] != NULL)
    return drivers[instance]->highest_supported_status();
  return AP_GPS::GPS_OK_FIX_3D;
}

AP_GPS::GPS_Status AP_GPS::highest_supported_status(void) const {
  if (drivers[primary_instance] != NULL)
    return drivers[primary_instance]->highest_supported_status();
  return AP_GPS::GPS_OK_FIX_3D;
}

/*
  update one GPS instance. This should be called at 10Hz or greater
 */
void AP_GPS::update_instance(uint8_t instance) {
  if (_type[instance] == GPS_TYPE_HIL) {
    // in HIL, leave info alone
    return;
  }
  if (_type[instance] == GPS_TYPE_NONE) {
    // not enabled
    state[instance].status = NO_GPS;
    state[instance].hdop = 9999;
    return;
  }
  if (locked_ports & (1U << instance)) {
    // the port is locked by another driver
    return;
  }

  if (drivers[instance] == NULL || state[instance].status == NO_GPS) {
    // we don't yet know the GPS type of this one, or it has timed
    // out and needs to be re-initialised
    detect_instance(instance);
    return;
  }

  send_blob_update(instance);

  // we have an active driver for this instance
  bool result = drivers[instance]->read();
  uint32_t tnow = AP_HAL::millis();

  // if we did not get a message, and the idle timer of 1.2 seconds
  // has expired, re-initialise the GPS. This will cause GPS
  // detection to run again
  if (!result) {
    if (tnow - timing[instance].last_message_time_ms > 1200) {
      // free the driver before we run the next detection, so we
      // don't end up with two allocated at any time
      delete drivers[instance];
      drivers[instance] = NULL;
      memset(&state[instance], 0, sizeof(state[instance]));
      state[instance].instance = instance;
      state[instance].status = NO_GPS;
      state[instance].hdop = 9999;
      timing[instance].last_message_time_ms = tnow;
    }
  } else {
    timing[instance].last_message_time_ms = tnow;
    if (state[instance].status >= GPS_OK_FIX_2D) {
      timing[instance].last_fix_time_ms = tnow;
    }
  }
}

/*
  update all GPS instances
 */
void AP_GPS::update(void) {
  for (uint8_t i = 0; i < GPS_MAX_INSTANCES; i++) {
    update_instance(i);
  }

  // work out which GPS is the primary, and how many sensors we have
  for (uint8_t i = 0; i < GPS_MAX_INSTANCES; i++) {
    if (state[i].status != NO_GPS) {
      num_instances = i + 1;
    }
    if (_auto_switch) {
      if (i == primary_instance) {
        continue;
      }
      if (state[i].status > state[primary_instance].status) {
        // we have a higher status lock, change GPS
        primary_instance = i;
        continue;
      }

      bool another_gps_has_1_or_more_sats =
          (state[i].num_sats >= state[primary_instance].num_sats + 1);

      if (state[i].status == state[primary_instance].status &&
          another_gps_has_1_or_more_sats) {

        uint32_t now = AP_HAL::millis();
        bool another_gps_has_2_or_more_sats =
            (state[i].num_sats >= state[primary_instance].num_sats + 2);

        if ((another_gps_has_1_or_more_sats &&
             (now - _last_instance_swap_ms) >= 20000) ||
            (another_gps_has_2_or_more_sats &&
             (now - _last_instance_swap_ms) >= 5000)) {
          // this GPS has more satellites than the
          // current primary, switch primary. Once we switch we will
          // then tend to stick to the new GPS as primary. We don't
          // want to switch too often as it will look like a
          // position shift to the controllers.
          primary_instance = i;
          _last_instance_swap_ms = now;
        }
      }
    } else {
      primary_instance = 0;
    }
  }

  // update notify with gps status. We always base this on the
  // primary_instance
  AP_Notify::flags.gps_status = state[primary_instance].status;
  AP_Notify::flags.gps_num_sats = state[primary_instance].num_sats;
}

/*
  set HIL (hardware in the loop) status for a GPS instance
 */
void AP_GPS::setHIL(uint8_t instance, GPS_Status _status,
                    uint64_t time_epoch_ms, const Location &_location,
                    const Vector3f &_velocity, uint8_t _num_sats, uint16_t hdop,
                    bool _have_vertical_velocity) {
  if (instance >= GPS_MAX_INSTANCES) {
    return;
  }
  uint32_t tnow = AP_HAL::millis();
  GPS_State &istate = state[instance];
  istate.status = _status;
  istate.location = _location;
  istate.location.options = 0;
  istate.velocity = _velocity;
  istate.ground_speed = pythagorous2(istate.velocity.x, istate.velocity.y);
  istate.ground_course_cd = wrap_360_cd(
      degrees(atan2f(istate.velocity.y, istate.velocity.x)) * 100UL);
  istate.hdop = hdop;
  istate.num_sats = _num_sats;
  istate.have_vertical_velocity |= _have_vertical_velocity;
  istate.last_gps_time_ms = tnow;
  uint64_t gps_time_ms =
      time_epoch_ms -
      (17000ULL * 86400ULL + 52 * 10 * 7000ULL * 86400ULL - 15000ULL);
  istate.time_week = gps_time_ms / (86400 * 7 * (uint64_t)1000);
  istate.time_week_ms =
      gps_time_ms - istate.time_week * (86400 * 7 * (uint64_t)1000);
  timing[instance].last_message_time_ms = tnow;
  timing[instance].last_fix_time_ms = tnow;
  _type[instance].set(GPS_TYPE_HIL);
}

/**
   Lock a GPS port, prevening the GPS driver from using it. This can
   be used to allow a user to control a GPS port via the
   SERIAL_CONTROL protocol
 */
void AP_GPS::lock_port(uint8_t instance, bool lock) {

  if (instance >= GPS_MAX_INSTANCES) {
    return;
  }
  if (lock) {
    locked_ports |= (1U << instance);
  } else {
    locked_ports &= ~(1U << instance);
  }
}

// Inject a packet of raw binary to a GPS
void AP_GPS::inject_data(uint8_t *data, uint8_t len) {
  // Support broadcasting to all GPSes.
  if (_inject_to == GPS_RTK_INJECT_TO_ALL) {
    for (uint8_t i = 0; i < GPS_MAX_INSTANCES; i++) {
      inject_data(i, data, len);
    }
  } else {
    inject_data(_inject_to, data, len);
  }
}

void AP_GPS::inject_data(uint8_t instance, uint8_t *data, uint8_t len) {
  if (instance < GPS_MAX_INSTANCES && drivers[instance] != NULL)
    drivers[instance]->inject_data(data, len);
}

void AP_GPS::send_mavlink_gps_raw(mavlink_channel_t chan) {
  static uint32_t last_send_time_ms[MAVLINK_COMM_NUM_BUFFERS];
  if (status(0) > AP_GPS::NO_GPS) {
    // when we have a GPS then only send new data
    if (last_send_time_ms[chan] == last_message_time_ms(0)) {
      return;
    }
    last_send_time_ms[chan] = last_message_time_ms(0);
  } else {
    // when we don't have a GPS then send at 1Hz
    uint32_t now = AP_HAL::millis();
    if (now - last_send_time_ms[chan] < 1000) {
      return;
    }
    last_send_time_ms[chan] = now;
  }
  const Location &loc = location(0);
  mavlink_msg_gps_raw_int_send(chan, last_fix_time_ms(0) * (uint64_t)1000,
                               status(0), loc.lat, // in 1E7 degrees
                               loc.lng,            // in 1E7 degrees
                               loc.alt * 10UL,     // in mm
                               get_hdop(0), get_vdop(0),
                               ground_speed(0) * 100, // cm/s
                               ground_course_cd(0),   // 1/100 degrees,
                               num_sats(0));
}

void AP_GPS::send_mavlink_gps2_raw(mavlink_channel_t chan) {
  static uint32_t last_send_time_ms[MAVLINK_COMM_NUM_BUFFERS];
  if (num_sensors() < 2 || status(1) <= AP_GPS::NO_GPS) {
    return;
  }
  // when we have a GPS then only send new data
  if (last_send_time_ms[chan] == last_message_time_ms(1)) {
    return;
  }
  last_send_time_ms[chan] = last_message_time_ms(1);

  const Location &loc = location(1);
  mavlink_msg_gps2_raw_send(
      chan, last_fix_time_ms(1) * (uint64_t)1000, status(1), loc.lat, loc.lng,
      loc.alt * 10UL, get_hdop(1), get_vdop(1), ground_speed(1) * 100, // cm/s
      ground_course_cd(1), // 1/100 degrees,
      num_sats(1), 0, 0);
}

void AP_GPS::send_mavlink_gps_rtk(mavlink_channel_t chan) {
  if (drivers[0] != NULL &&
      drivers[0]->highest_supported_status() > AP_GPS::GPS_OK_FIX_3D) {
    drivers[0]->send_mavlink_gps_rtk(chan);
  }
}

void AP_GPS::send_mavlink_gps2_rtk(mavlink_channel_t chan) {
  if (drivers[1] != NULL &&
      drivers[1]->highest_supported_status() > AP_GPS::GPS_OK_FIX_3D) {
    drivers[1]->send_mavlink_gps_rtk(chan);
  }
}
