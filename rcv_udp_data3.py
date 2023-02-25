# Script with minimum parsing to send binary data to UDP with period 
# Test in linux system: sudo tcpdump -vv -A udp dst port 10005


import datetime
from multiprocessing.dummy import Process
import pprint
import time
import serial
import serial.tools.list_ports
import struct
# sys.setdefaultencoding('utf-8')
from mmradar_ops import mmradar_conf
from serial_ops import open_serial_ports, set_serials_cfg , close_serial_ports , open_serial_ports
import socket
from file_ops import write_data_2_local_file


################################################################
######################## DEFINITIONS ###########################
################################################################

control                         = 506660481457717506
frame                           = bytes(1)

frame_header_struct = 'Q9I2H'
frame_header_length = struct.calcsize ( frame_header_struct )

#dst_udp_ip                      = '10.0.0.157' # Lipków raspberry pi 3b+
#dst_udp_ip                      = '10.0.0.159' # Lipków raspberry pi 02w
#dst_udp_ip                      = '10.0.0.5' # Lipków GO3
#dst_udp_ip                      = '192.168.1.14' # Meander raspberrypi 3b
#dst_udp_ip                      = '192.168.1.17' # Meander raspberrypi 02w
#dst_udp_ip                      = '192.168.1.30' # Meander MW50-SV0
#src_udp_ip                      = '10.0.0.5' # Lipków GO3
#src_udp_ip                      = '192.168.1.30' # Meander MW50-SV0
#src_udp_ip                      = '10.0.0.157' # Lipków raspberry pi 3b
#src_udp_ip                      = '10.0.0.159' # Lipków raspberry pi 02w
#src_udp_ip                      = '192.168.43.227' # maczem raspberry pi 3b+
#src_udp_ip                      = '192.168.43.215' # maczem GO3
src_udp_ip                      = '127.0.0.1' # maczem GO3
#dst_udp_ip                      = '192.168.43.227' # maczem raspberry pi 3b+
#dst_udp_ip                      = '192.168.43.215' # maczem GO3
ctrl_udp_port                   = 10004
data_udp_port                   = 10005

hello = "\n\n##########################################\n############# mmradar started ############\n##########################################\n"

################################################################
################ SOCKET Configuration ##########################
################################################################
#src_udp = socket.socket ( socket.AF_INET , socket.SOCK_DGRAM , socket.IPPROTO_UDP )
#dest_udp.sendto ( bytes ( 'Hello' , 'utf-8') , ( dest_udp_ip , dest_udp_port ) )
src_udp = socket.socket ( socket.AF_INET , socket.SOCK_DGRAM , socket.IPPROTO_UDP )
src_udp.bind ( ( src_udp_ip , data_udp_port ) )

##################### READ DATA #################################
while True :
    try :
        frame , address = src_udp.recvfrom ( 10000 )
        print ( "\n\n 2. Server received: ", frame.decode ( 'utf-8' ) , "\n\n" )
    except struct.error as e :
        print ( e )
################# CLOSE DATA COM PORT FILE ######################
src_udp.close ()