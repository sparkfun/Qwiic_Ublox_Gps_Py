# Python library for the SparkFun's line of u-Blox GPS units.
#
# SparkFun GPS-RTK2 ZED-F9P:
#   https://www.sparkfun.com/products/15136
# SparkFun GPS-RTK2 NEO-M8P:
#   https://www.sparkfun.com/products/15005
# SparkFun GPS-RTK2 ZOE-M8Q:
#   https://www.sparkfun.com/products/15193
# SparkFun GPS-RTK2 SAM-M8Q:
#   https://www.sparkfun.com/products/15210
#
#------------------------------------------------------------------------
#
# Written by SparkFun Electronics, October 2019
#
# This python library supports the SparkFun Electroncis qwiic
# qwiic sensor/board ecosystem
#
# More information on qwiic is at https:// www.sparkfun.com/qwiic
#
# Do you like this library? Help support SparkFun. Buy a board!
#==================================================================================
# Copyright (c) 2019 SparkFun Electronics
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#==================================================================================
#
# This is mostly a port of existing Arduino functionaly, so pylint is sad.
# The goal is to keep the public interface pthonic, but internal is internal
#
# pylint: disable=line-too-long, bad-whitespace, invalid-name, too-many-public-methods
#

# pylint: disable-all
import ublox_comms
import module_constants as ubc

class UbloxGps(object):

    def __init__(self, comm_interface):
        self.comm_inter = comm_interface


        active_packet_buffer = ubc.SFE_UBLOX_PACKET_PACKETBUF
    
    def begin(self):
        pass


    def calc_fletch_checksum(self, packet):

        checksum_B = 0
        checksum_A = packet.get('class')
        checksum_A = checksum_A + packet.get('id')
        checksum_B = checksum_A + checksum_B
        checksum_A = checksum_A + packet.get('length')
        checksum_B = checksum_A + checksum_B
        for item in packet.get('payload'):
            checksum_A = checksum_A + item
            checksum_B = checksum_B + checksum_A

        packet["checkA"] = checksum_A
        packet["checkB"] = checksum_B

        return packet

    def add_fletch_checksum(self, incoming):
        ubc.rolling_checksum_A = ubc.rolling_checksum_A + incoming
        ubc.rolling_checksum_B = ubc.rolling_checksum_B + ubc.rolling_checksum_A

    def build_packet(self, ubx_class, ubx_id, ubx_length, ubx_payload):

        ubx_message = {
            "class" : ubx_class,
            "id" : ubx_id,
            "length_lsb" : ubx_length & 0xFF,
            "length_msb" : ubx_length >> 8,
            "length" : ubx_length,
            "payload" : ubx_payload, #list of bytes
            "checkA" : None,
            "checkB" : None
        }

        packet = self.calc_fletch_checksum(ubx_message)

        return packet


    def confirm_send(self, ubx_response, packet):

        if ubx_response.get('class') == UBX_CLASS_ACK:
            if ubx_response.get('ubx_id') == UBX_ACK_ACK:
                if ubx_response.get('ubx_payload')[0] == \
                    packet.get('class') and \
                    ubx_response.get['ubx_payload'][1] == \
                    packet.get('ubx_id'):
                    return True

            elif ubx_response.get('ubx_id') == UBX_ACK_NAK:
                return False

    def get_altitude(self):
        return True
    
    def byte_mill(self):
        passed_char = self.comm_inter.read_ublox_stream()
        self.process(passed_char)
        return(passed_char)

            
    def process(self, ublox_data, incoming_ubx, requested_class, requested_id):
        
        # check what the byte is
        if ubc.current_sentence == None or ubc.current_sentence == NMEA:

            if ublox_data == ubc.UBX_SYNCH_1:
                ubc.UBX_FRAME_COUNTER = 0
                ubc.current_sentence = ubc.UBX
                ubc.ubx_message_buffer['counter'] = 0
                ubc.IGNORE_PAYLOAD = False
                self.active_packet_buffer = ubc.SFE_UBLOX_PACKET_PACKETBUF

            elif ublox_data == ubc.NMEA_BEGIN:
                ubc.current_sentence = ubc.NMEA
            elif ublox_data == ubc.RTCM_BEGIN_VAL:
                ubc.current_sentence = ubc.RTCM
            else: 
                pass # Missed something

        if ubc.current_sentence == ubc.UBX: 
            if ubc.UBX_FRAME_COUNTER == 0 and ublox_data != ubc.UBX_SYNCH_1: 
                ubc.current_sentence == None
            elif ubc.UBX_FRAME_COUNTER == 1 and ublox_data != ubc.UBX_SYNCH_2: 
                ubc.current_sentence == None
            elif ubc.UBX_FRAME_COUNTER == 2: 
                ubc.ubx_message_buffer['class'] = ublox_data  
                ubc.ubx_message_buffer['checksum_A'] = 0
                ubc.ubx_message_buffer['checksum_B'] = 0
                ubc.ubx_message['start_spot'] = incoming_ubx['start_spot']
            elif ubc.UBX_FRAME_COUNTER == 3:
                ubc.ubx_message_buffer['id'] = ublox_data
                if ubc.ubx_message_buffer['class'] != ubc.UBX_CLASS_ACK:
                    if ubc.ubx_message_buffer['class'] == requested_class and \
                       ubc.ubx_message_buffer['id'] == requested_id:

                        ubc.ubx_message['class'] = ubc.ubx_message_buffer['class']
                        ubc.ubx_message['id'] = ubc.ubx_message_buffer['id']
                        ubc.ubx_message['counter'] = ubc.ubx_message_buffer['counter']

                    else:
                        ubc.IGNORE_PAYLOAD = True
                else:
                    pass
                            
            elif ubc.UBX_FRAME_COUNTER == 4:
                ubc.ubx_message_buffer['length'] = ublox_data
                ubc.ubx_message_buffer['length_lsb'] = ublox_data

            elif ubc.UBX_FRAME_COUNTER == 5:
                ubc.ubx_message_buffer['length'] |= (ublox_data << 8)
                ubc.ubx_message_buffer['length_msb'] = ublox_data

            elif ubc.UBX_FRAME_COUNTER == 6: #In the payload now
                if ubc.ubx_message_buffer['length'] == 0: 
                    ubc.ubx_message_buffer['checksumA'] = ublox_data  
                else:
                    ubc.ubx_message_buffer['payload'][0] = ublox_data

            elif ubc.UBX_FRAME_COUNTER == 7: 
                if ubc.ubx_message_buffer['length'] == 0: 
                    ubc.ubx_message_buffer['checksumB'] = ublox_data  
                elif ubc.ubx_message_buffer['length'] == 1: 
                    ubc.ubx_message_buffer['checksumA'] = ublox_data  
                else: 
                    ubc.ubx_message_buffer['payload'][1] = ublox_data

                if self.active_packet_buffer == ubc.SFE_UBLOX_PACKET_PACKETBUF and\
                   ubc.ubx_message_buffer['class'] == ubc.UBX_CLASS_ACK and\
                   ubc.ubx_message_buffer['payload'][0] == requested_class and\
                   ubc.ubx_message_buffer['payload'][1] == requested_id: 
                    if ubc.ubx_message_buffer['length'] == 2:
                        self.active_packet_buffer = ubc.SFE_UBLOX_PACKET_PACKETACK
                        ubc.ubx_packet_ack['class'] = ubc.ubx_message_buffer['class']
                        ubc.ubx_packet_ack['id'] = ubc.ubx_message_buffer['id']
                        ubc.ubx_packet_ack['length'] = ubc.ubx_message_buffer['length']
                        ubc.ubx_packet_ack['counter'] = ubc.ubx_message_buffer['counter']
                        ubc.ubx_packet_ack['payload'][0] = ubc.ubx_message_buffer['payload'][0]
                        ubc.ubx_packet_ack['payload'][1] = ubc.ubx_message_buffer['payload'][1]
                    else:
                        #Add something here
                        pass
            
            # Clean this up
            if self.active_packet_buffer == ubc.SFE_UBLOX_PACKET_PACKETACK:
                self.process_UBX(ublox_data, ubc.ubx_packet_ack, requested_class, requested_id) # modified but ignoring
            elif self.active_packet_buffer == ubc.SFE_UBLOX_PACKET_PACKETCFG:
                self.process_UBX(ublox_data, ubc.ubx_packet_cfg, requested_class, requested_id)
            else: 
                self.process(ublox_data, ubc.ubx_message_buffer, requested_class, requested_id)

            ubc.UBX_FRAME_COUNTER = ubc.UBX_FRAME_COUNTER + 1            

        elif ubc.current_sentence == NMEA:
            self.process_NMEA(ublox_data)
        elif ubc.current_sentence == RTCM:
            self.process_RTCM(ublox_data)
                

    
    def process_UBX(self,  incoming_data, incoming_ubx, requested_class, requested_id): # modified but ignorning
        if incoming_ubx['counter'] < incoming_ubx['len'] + 4:  
            self.add_fletch_checksum(incoming_data)             

        if incoming_ubx['counter'] == 0:
            incoming_ubx['class'] = incoming_data
        elif incoming_ubx['counter'] == 1:
            incoming_ubx['id'] = incoming_data
        elif incoming_ubx['counter'] == 2: #LSB
            incoming_ubx['length'] = incoming_data
        elif incoming_ubx['counter'] == 3:
            incoming_ubx['length'] |= (incoming_data<<8) #MSB
        elif incoming_ubx['counter'] == incoming_ubx['len'] + 4:  
            incoming_ubx['checkA'] = incoming_data
        elif incoming_ubx['counter'] == incoming_ubx['len'] + 5:  
            incoming_ubx['checkB'] = incoming_data
            
            ubc.current_sentence == None

            if incoming_ubx['checkA'] == ubc.rolling_checksum_A and\
               incoming_ubx['checkB'] == ubc.rolling_checksum_B:

                incoming_ubx['valid'] = ubc.SFE_UBLOX_PACKET_VALIDITY_VALID

                if incoming_ubx['class'] == requested_class and\
                   incoming_ubx['id'] == requested_id:

                    incoming_ubx['class_id_match'] == ubc.SFE_UBLOX_PACKET_VALIDITY_VALID

                elif incoming_ubx['class'] == ubc.UBX_CLASS_ACK and\
                     incoming_ubx['id'] == ubc.UBX_ACK_ACK and\
                     incoming_ubx['payload'][0] == requested_class and\
                     incoming_ubx['payload'][1] == requested_id:

                    incoming_ubx['class_id_match'] == ubc.SFE_UBLOX_PACKET_VALIDITY_VALID

                elif incoming_ubx['class'] == ubc.UBX_CLASS_ACK and\
                     incoming_ubx['id'] == ubc.UBX_ACK_NACK and\
                     incoming_ubx['payload'][0] == requested_class and\
                     incoming_ubx['payload'][1] == requested_id:

                    incoming_ubx['class_id_match'] == ubc.SFE_UBLOX_PACKET_NOTACKNOWLEDGED
                
                if ubc.IGNORE_PAYLOAD == False:
                    self.process_UBX_Packet(incoming_ubx)
            else: #Checksum Failure
                incoming_ubx['valid'] = SFE_UBLOX_PACKET_VALIDITY_NOT_VALID

                if incoming_ubx['class'] == requested_class and\
                   incoming_ubx['id'] == requested_id:

                    incoming_ubx['class_id_match'] == ubc.SFE_UBLOX_PACKET_NOTACKNOWLEDGED

                elif incoming_ubx['class'] == ubc.UBX_CLASS_ACK and\
                     incoming_ubx['id'] == ubc.UBX_ACK_ACK and\
                     incoming_ubx['payload'][0] == requested_class and\
                     incoming_ubx['payload'][1] == requested_id:

                    incoming_ubx['class_id_match'] == ubc.SFE_UBLOX_PACKET_VALIDITY_VALID

        else: 
            start_spot = incoming_ubx['start_spot']
            if incoming_ubx['class'] == ubc.UBX_CLASS_NAV and incoming_ubx['id'] == ubc.UBX_NAV_PVT:
                start_spot = 0
            if incoming_ubx['counter'] - 4 >= start_spot:
                if ((incoming_ubx['counter'] - 4) - start_spot) < MAX_PAYLOAD_SIZE:
                    if ubc.IGNORE_PAYLOAD == False: 
                        incoming_ubx['payload'][incoming_ubx['counter']- 4 - start_spot] = incoming_data
    
        incoming_ubx['counter'] = incoming_ubx['counter'] + 1

        if incoming_ubx['counter'] == MAX_PAYLOAD_SIZE:
            ubc.current_sentence == None

    def process_UBX_Packet(self, inc_ubx):
        if inc_ubx['class'] == UBX_CLASS_NAV:
            if inc_ubx['id'] == UBX_NAV_PVT and inc_ubx['length'] == 92:

                start_spot = 0

                ubc.time_of_week = self.extract_long(ubc.ubx_packet_cfg, 0)
                ubc.gps_millisecond = self.extract_long(ubc.ubx_packet_cfg, 0) % 1000
                ubc.gps_year = self.extract_int(ubc.ubx_packet_cfg, 4)
                ubc.gps_month = self.extract_byte(ubc.ubx_packet_cfg, 6)
                ubc.gps_day = self.extract_byte(ubc.ubx_packet_cfg, 7)
                ubc.gps_hour = self.extract_byte(ubc.ubx_packet_cfg, 8)
                ubc.gps_minute = self.extract_byte(ubc.ubx_packet_cfg, 9)
                ubc.gps_second = self.extract_byte(ubc.ubx_packet_cfg, 10)
                ubc.gps_nanosecond = self.extract_long(ubc.ubx_packet_cfg, 16) # Includes Milliseconds
                
                ubc.fix_type = self.extract_byte(ubc.ubx_packet_cfg, 20 - start_spot) 
                ubc.carrier_solution = self.extract_byte(ubc.ubx_packet_cfg, 21 - start_spot) >> 6
                ubc.SIV = self.extract_byte(ubc.ubx_packet_cfg, 23 - start_spot)
                ubc.longitude = self.extract_long(ubc.ubx_packet_cfg, 24 - start_spot)
                ubc.latitude = self.extract_long(ubc.ubx_packet_cfg, 28 - start_spot)
                ubc.altitude = self.extract_long(ubc.ubx_packet_cfg, 32 - start_spot)
                ubc.altitude_MSL = self.extract_long(ubc.ubx_packet_cfg, 36 - start_spot)
                ubc.ground_speed = self.extract_long(ubc.ubx_packet_cfg, 60 - start_spot)
                ubc.heading_motion = self.extract_long(ubc.ubx_packet_cfg, 64 - start_spot)
                ubc.pDOP = self.extract_int(ubc.ubx_packet_cfg, 76 - start_spot)

                ubc.is_module_queried['gps_iTOW'] = True
                ubc.is_module_queried['gps_iTOW'] = True
                ubc.is_module_queried['gps_year'] = True       
                ubc.is_module_queried['gps_month'] = True       
                ubc.is_module_queried['gps_day'] = True         
                ubc.is_module_queried['gps_hour'] = True        
                ubc.is_module_queried['gps_minute'] = True      
                ubc.is_module_queried['gps_second'] = True      
                ubc.is_module_queried['gps_nanosecond'] = True  
                ubc.is_module_queried['all'] = True             
                ubc.is_module_queried['longitude'] = True       
                ubc.is_module_queried['latitude'] = True        
                ubc.is_module_queried['altitude'] = True        
                ubc.is_module_queried['altitude_MSL'] = True    
                ubc.is_module_queried['SIV'] = True             
                ubc.is_module_queried['fix_type'] = True        
                ubc.is_module_queried['carrier_solution'] = True
                ubc.is_module_queried['ground_speed'] = True    
                ubc.is_module_queried['heading_motion'] = True  
                ubc.is_module_queried['pDOP'] = True            
                ubc.is_module_queried['version_num'] = True     

            elif inc_ubx['id'] == UBX_NAV_HPPOSLLH and inc_ubx['length'] == 36:

                ubc.time_of_week = self.extract_long(ubc.ubx_packet_cfg, 4)
                ubc.high_res_latitude = self.extract_long(ubc.ubx_packet_cfg, 8)
                ubc.high_res_longitude = self.extract_long(ubc.ubx_packet_cfg, 12)
                ubc.elipsoid = self.extract_long(ubc.ubx_packet_cfg, 16)
                ubc.mean_sea_level = self.extract_long(ubc.ubx_packet_cfg, 20)
                ubc.high_res_longitude_Hp = self.extract_long(ubc.ubx_packet_cfg, 24)
                
                ubc.fix_type = self.extract_byte(ubc.ubx_packet_cfg, 20 - start_spot) 
                ubc.carrier_solution = self.extract_byte(ubc.ubx_packet_cfg, 21 - start_spot) >> 6
                ubc.SIV = self.extract_byte(ubc.ubx_packet_cfg, 23 - start_spot)
                ubc.longitude = self.extract_long(ubc.ubx_packet_cfg, 24 - start_spot)
                ubc.latitude = self.extract_long(ubc.ubx_packet_cfg, 28 - start_spot)
                ubc.altitude = self.extract_long(ubc.ubx_packet_cfg, 32 - start_spot)
                ubc.altitude_MSL = self.extract_long(ubc.ubx_packet_cfg, 36 - start_spot)
                ubc.ground_speed = self.extract_long(ubc.ubx_packet_cfg, 60 - start_spot)
                ubc.heading_motion = self.extract_long(ubc.ubx_packet_cfg, 64 - start_spot)
                ubc.pDOP = self.extract_int(ubc.ubx_packet_cfg, 76 - start_spot)

                ubc.is_module_queried['gps_iTOW'] = True
                ubc.is_module_queried['gps_iTOW'] = True
                ubc.is_module_queried['gps_year'] = True       
                ubc.is_module_queried['gps_month'] = True       
                ubc.is_module_queried['gps_day'] = True         
                ubc.is_module_queried['gps_hour'] = True        
                ubc.is_module_queried['gps_minute'] = True      
                ubc.is_module_queried['gps_second'] = True      
                ubc.is_module_queried['gps_nanosecond'] = True  
                ubc.is_module_queried['all'] = True             
                ubc.is_module_queried['longitude'] = True       
                ubc.is_module_queried['latitude'] = True        
                ubc.is_module_queried['altitude'] = True        
                ubc.is_module_queried['altitude_MSL'] = True    
                ubc.is_module_queried['SIV'] = True             
                ubc.is_module_queried['fix_type'] = True        
                ubc.is_module_queried['carrier_solution'] = True
                ubc.is_module_queried['ground_speed'] = True    
                ubc.is_module_queried['heading_motion'] = True  
                ubc.is_module_queried['pDOP'] = True            
                ubc.is_module_queried['version_num'] = True     

    def process_NMEA(self, incoming_data):
        self.comm_inter.pipe_out(incoming_data) 
         
    def process_RTCM(self, incoming_data):
        # This is not direcly implemented in the Arduino library but is still
        # there for the user to build their own output method. 
        pass 

    def clear_configuration(self):
        packet = self.build_packet(ubc.UBX_CLASS_CFG, 
                                                  ubc.UBX_CFG_CFG,
                                                  0, 0)
        ublox_reponse = self.comm_inter.receive_command(packet)
        return(ublox_reponse.get('payload'))

    def extract_byte(self, packet, location):
        payload = packet.get('payload')
        byte = payload[location]
        return byte 

    def extract_int(self, packet, location):
        payload = packet.get('payload')
        two_byte = payload[location] << 8
        two_byte = two_byte | payload[location + 1]
        return two_byte 

    def extract_long(self, packet, location):
        payload = packet.get('payload')
        four_byte = payload[location] << 24
        four_byte = four_byte | (payload[location + 1] << 16)
        four_byte = four_byte | (payload[location + 2] << 8)
        four_byte = four_byte | payload[location + 3] 
        return four_byte 

    def get_soft_version(self):
        packet = self.build_packet(ubc.UBX_CLASS_MON,
                                                  ubc.UBX_MON_VER, 
                                                  0, [])
        ublox_reponse = self.comm_inter.send_command(packet)
        payload = ublox_reponse.get('payload') 
        if payload is not None:
            soft_vers = '0'
            for item in range(30): 
                soft_vers.append(ord(item))

            return soft_vers 
        else: 
            return 0

    def get_hard_version(self):
        packet = self.build_packet(ubc.UBX_CLASS_MON,
                                                  ubc.UBX_MON_VER, 
                                                  0, [])
        ublox_reponse = self.comm_inter.send_command(packet)
        payload = ublox_reponse.get('payload') 
        hard_vers = 0
        for byte in range(30, 41): 
           hard_vers.append(ord(byte)) 
        return hard_vers 
