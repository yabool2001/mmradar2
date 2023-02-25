# Issue 1 - tlv list
# Zamienić wszystkie self.tlv_header_length na self.tl_length
# Napisać do ti, że tlv_length to jest tl_length. Jeśli to jest tlv, to mnie poprawcie. Moim zdaniem lepiej żeby była podawana całą wartość tlv

import sys
#sys.setdefaultencoding('utf-8')
sys.path.append ( "/Users/mzeml/python/mmradar/modules/" )
import logging
import pprint

import datetime
import file_ops2
import mmradar_ops2
import mmradar_pc3d2
import serial
import serial_ops2
import socket
import threading
import time

#from mmradar_ops2 import mmradar_conf
#from file_ops2 import write_data_2_local_file

################################################################
### DEFINITIONS 
################################################################

data_source                     = 2 # 0: device, 1: UDP, 2: file
data_dst                        = 1 # 0: Azure, 1: UDP, 2: file
chirp_cfg                       = 0 # 0: no cfg, 1: only start(), 2: full cfg
raw_byte                        = bytes(1)
log_file_name                   = 'log/mmradar.log'
data_file_name                  = 'mmradar.data'
saved_parsed_data_file_name     = 'saved_parsed_data\mmradar.data.json'
saved_bin_data_file_name        = 'saved_bin_data\mmradar_gen_1675746223207587500.bin_raw_data'
chirp_conf_file_name            = 'chirp_cfg/ISK_6m_staticRetention.cfg'
#src_udp_ip                      = '192.168.43.227' # maczem raspberry pi 3b+
#src_udp_ip                      = '192.168.43.215' # maczem GO3
#src_udp_ip                      = '127.0.0.1' # maczem GO3
src_udp_ip                      = socket.gethostbyname ( socket.gethostname () )
#dst_udp_ip                      = '192.168.43.227' # maczem raspberry pi 3b+
#dst_udp_ip                      = '192.168.43.215' # maczem GO3
dst_udp_ip                      = '127.0.0.1' # maczem GO3
ctrl_udp_port                   = 10004
data_udp_port                   = 10005

hello = "\n\n##########################################\n############# mmradar started ############\n##########################################"

def data_dst_2_thread () :
    file_ops2.write_2_local_file ( saved_parsed_data_file_name , str ( pc3d_object.frame_dict ) )
def data_dst_1_thread () :
    udp.sendto ( str.encode ( str ( pc3d_object.frame_dict ) , "utf-8" ) , ( dst_udp_ip , data_udp_port ) )

################################################################
###################### LOGGING CONFIG ##########################
################################################################

LOG_FORMAT = '%(asctime)s;%(message)s;%(funcName)s;line:%(lineno)d;%(levelname)s'
logging.basicConfig ( filename = log_file_name , level = logging.INFO , format = LOG_FORMAT )
logging.info (f"\n")
logging.info ( f"########## Hello mmradar app! ##############" )
logging.info ( f"########## 'main.py' has started! ##########" )

################################################################
################ SERIALS COMM CONFiguration ####################
################################################################

################################################################
####################### START PROGRAM ##########################
################################################################

print ( hello )

### READ DATA
match data_source :
    case 0:
        print ( "\n############# Direct device sourcing.\n" )
        logging.info ( f"############# Direct device sourcing.\n")
        conf_com = serial.Serial ()
        data_com = serial.Serial ()
        serial_ops2.set_serials_cfg ( conf_com , data_com )
        serial_ops2.open_serial_ports ( conf_com , data_com )
        conf_com.reset_input_buffer ()
        conf_com.reset_output_buffer ()
        mmradar_ops2.mmradar_conf ( chirp_conf_file_name , conf_com )
        print ( "\n############# Device configured.\n" )
        logging.info ( f"############# Device configured.\n")
        frames_limit = 1
    case 1:
        print ( "\n############# UDP sourcing.\n" )
        logging.info ( f"############# UDP sourcing.\n")
        frames_limit = 1
        exit
    case 2:
        print ( "\n############# Saved raw data sourcing.\n" )
        logging.info ( f"############# Saved raw data sourcing.\n")
        saved_bin_frames = open ( saved_bin_data_file_name , 'r' ) .readlines ()
        frames_limit = len ( saved_bin_frames )
    case _:
        logging.info (f"Error: data_source not known. App exit!\n")
        exit

match data_dst :
    case 0 :
        logging.info (f"Error: data_dst = 0. App exit!\n")
        exit ()
    case 1 :
        ################ SOCKET Configuration
        udp = socket.socket ( socket.AF_INET , socket.SOCK_DGRAM , socket.IPPROTO_UDP )
    case 2 :
        pass

udp_ctrl_rcv = socket.socket ( socket.AF_INET , socket.SOCK_DGRAM , socket.IPPROTO_UDP )
udp_ctrl_rcv.setblocking ( False ) # Jak nic nie będzie w buforze to skrypt będzie działał dalej - chyba
udp_ctrl_rcv.bind ( ( src_udp_ip , ctrl_udp_port ) )

i = 0
start_t = time.perf_counter ()
while i < frames_limit :
    match data_source :
        case 0:
            data_com.reset_input_buffer ()
            data_com.reset_output_buffer ()
            frame = data_com.read ( 4666 )
            pc3d_object = mmradar_pc3d2.PC3D ( frame )
            pc3d_object.get_json_data ()
        case 1 :
            logging.info (f"Error: data_source = 1. App exit!\n")
            exit
        case 2 :
            frame = eval ( saved_bin_frames[i] )
            pc3d_object = mmradar_pc3d2.PC3D ( frame )
            pc3d_object.get_json_data ()
            i += 1
    match data_dst :
        case 0 :
            pass
        case 1 :
            #udp.sendto ( str.encode ( str ( pc3d_object.frame_dict ) , "utf-8" ) , ( dst_udp_ip , data_udp_port ) ) # alternatywa do 2 poniższych wierszy
            thread_data_dst_1 = threading.Thread ( target = data_dst_1_thread )
            thread_data_dst_1.start ()
        case 2 :
            #file_ops2.write_2_local_file ( saved_parsed_data_file_name , str ( pc3d_object.frame_dict ) )
            thread_data_dst_2 = threading.Thread ( target = data_dst_2_thread )
            thread_data_dst_2.start ()
    del pc3d_object
    rcvd_ctrl_data, addr = udp_ctrl_rcv.recvfrom ( 1024 ) # buffer size is 1024 bytes
    print ( "received message: %s" %rcvd_ctrl_data)
finish_t = time.perf_counter ()
print ( f"Total time for while loop = {finish_t - start_t}")
