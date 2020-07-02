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
        checksum_A = packet.get('ubx_class')
        checksum_A = checksum_A + packet.get('ubx_id')
        checksum_B = checksum_A + checksum_B
        checksum_A = checksum_A + packet.get('ubx_length')
        checksum_B = checksum_A + checksum_B
        for item in packet.get('ubx_payload'):
            checksum_A = checksum_A + item
            checksum_B = checksum_B + checksum_A

        packet["ubx_checkA"] = checksum_A
        packet["ubx_checkB"] = checksum_B

        return packet

    def add_fletch_checksum(self, incoming):
        ubc.rolling_checksum_A = ubc.rolling_checksum_A + incoming
        ubc.rolling_checksum_B = ubc.rolling_checksum_B + ubc.rolling_checksum_A

    def build_packet(self, ubx_class, ubx_id, ubx_length, ubx_payload):

        ubx_message = {
            "ubx_class" : ubx_class,
            "ubx_id" : ubx_id,
            "ubx_length_lsb" : ubx_length & 0xFF,
            "ubx_length_msb" : ubx_length >> 8,
            "ubx_length" : ubx_length,
            "ubx_payload" : ubx_payload, #list of bytes
            "ubx_checkA" : None,
            "ubx_checkB" : None
        }

        packet = self.calc_fletch_checksum(ubx_message)

        return packet


    def confirm_send(self, ubx_response, packet):

        if ubx_response.get('ubx_class') == UBX_CLASS_ACK:
            if ubx_response.get('ubx_id') == UBX_ACK_ACK:
                if ubx_response.get('ubx_payload')[0] == \
                    packet.get('ubx_class') and \
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

            
    def process(self, ublox_data, requested_class, requested_id):
        
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
                ubc.ubx_message_buffer['ubx_class'] = ublox_data  
                ubc.ubx_message_buffer['checksum_A'] = 0
                ubc.ubx_message_buffer['checksum_B'] = 0
                # ubc.ubx_message['start_spot'] = 
            elif ubc.UBX_FRAME_COUNTER == 3:
                ubc.ubx_message_buffer['ubx_id'] = ublox_data
                if ubc.ubx_message_buffer['ubx_class'] != ubc.UBX_CLASS_ACK:
                    if ubc.ubx_message_buffer['ubx_class'] == requested_class and \
                       ubc.ubx_message_buffer['ubx_id'] == requested_id:

                        ubc.ubx_message['ubx_class'] = ubc.ubx_message_buffer['ubx_class']
                        ubc.ubx_message['ubx_id'] = ubc.ubx_message_buffer['ubx_id']
                        ubc.ubx_message['counter'] = ubc.ubx_message_buffer['counter']

                    else:
                        ubc.IGNORE_PAYLOAD = True
                else:
                    pass
                            
            elif ubc.UBX_FRAME_COUNTER == 4:
                ubc.ubx_message_buffer['length'] = ublox_data
                ubc.ubx_message_buffer['ubx_length_lsb'] = ublox_data

            elif ubc.UBX_FRAME_COUNTER == 5:
                ubc.ubx_message_buffer['length'] |= (ublox_data << 8)
                ubc.ubx_message_buffer['ubx_length_msb'] = ublox_data

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
                        ubc.ublox_packet_ack['class'] = ubc.ubx_message_buffer['class']
                        ubc.ublox_packet_ack['id'] = ubc.ubx_message_buffer['id']
                        ubc.ublox_packet_ack['length'] = ubc.ubx_message_buffer['length']
                        ubc.ublox_packet_ack['counter'] = ubc.ubx_message_buffer['counter']
                        ubc.ublox_packet_ack['payload'][0] = ubc.ubx_message_buffer['payload'][0]
                        ubc.ublox_packet_ack['payload'][1] = ubc.ubx_message_buffer['payload'][1]
                    else:
                        #Add something here
                        pass
            
            # Clean this up
            if self.active_packet_buffer == ubc.SFE_UBLOX_PACKET_PACKETACK:
                self.process_UBX_ACK(ublox_data, requested_class, requested_id)
            elif self.active_packet_buffer == ubc.SFE_UBLOX_PACKET_PACKETCFG:
                self.process_UBX_CFG(ublox_data, requested_class, requested_id)
            else: 
                self.process_UBX(ublox_data, requested_class, requested_id)

            ubc.UBX_FRAME_COUNTER = ubc.UBX_FRAME_COUNTER + 1            

        elif ubc.current_sentence == NMEA:
            self.process_NMEA(ublox_data)
        elif ubc.current_sentence == RTCM:
            self.process_RTCM(ublox_data)
                

    
    def process_UBX_ACK(self,  incoming_data, requested_class, requested_id):
        if ubc.ublox_packet_ack['counter'] < ubc.ublox_packet_ack['len'] + 4:  
            self.add_fletch_checksum(incoming_data)             

        if ubc.ublox_packet_ack['counter'] == 0:
            ubc.ublox_packet_ack['ubx_class'] = incoming_data
        elif ubc.ublox_packet_ack['counter'] == 1:
            ubc.ublox_packet_ack['ubx_id'] = incoming_data
        elif ubc.ublox_packet_ack['counter'] == 2: #LSB
            ubc.ublox_packet_ack['ubx_length'] = incoming_data
        elif ubc.ublox_packet_ack['counter'] == 3:
            ubc.ublox_packet_ack['ubx_length'] |= (incoming_data<<8) #MSB
        elif ubc.ublox_packet_ack['counter'] == ubc.ublox_packet_ack['len'] + 4:  
            ubc.ublox_packet_ack['ubx_checkA'] = incoming_data
        elif ubc.ublox_packet_ack['counter'] == ubc.ublox_packet_ack['len'] + 5:  
            ubc.ublox_packet_ack['ubx_checkB'] = incoming_data
            
            ubc.current_sentence == None

            if self.ubx_packet_ack['ubx_checkA'] == ubc.rolling_checksum_A and\
               self.ubx_packet_ack['ubx_checkB'] == ubc.rolling_checksum_B:

                self.ubx_packet_ack['valid'] = ubc.SFE_UBLOX_PACKET_VALIDITY_VALID

               if self.ubx_packet_ack['class'] == requested_class and\
                  self.ubx_packet_ack['id'] == requested_id and\
                  self.ubx_packet_ack['payload'][1] == requested_id: 


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
