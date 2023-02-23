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
import mmradar3_ops
import mmradar3_pc3d
import socket
import serial
import serial_ops2
import time

#from mmradar_ops2 import mmradar_conf
#from file_ops2 import write_data_2_local_file

################################################################
### DEFINITIONS 
################################################################

data_source                     = 0 # 0: device, 1: UDP, 2: file
cfg_chirp                       = 0 # 0: no cfg, 1: sensor start, 2: full cfg
data_destination                = 0 # 0: Azure, 1: UDP, 2: file
raw_byte                        = bytes(1)
log_file_name                   = 'log/mmradar.log'
data_file_name                  = 'mmradar.data'
saved_parsed_data_file_name     = 'saved_parsed_data\mmradar.data.json'
saved_bin_data_file_name        = 'saved_bin_data\mmradar_gen_1675746223207587500.bin_raw_data'
cfg_chirp_full_file_name        = 'chirp_cfg/ISK_6m_staticRetention.cfg'
cfg_chirp_start_file_name       = 'chirp_cfg/sensor_start.cfg'
#src_udp_ip                      = '192.168.43.227' # maczem raspberry pi 3b+
#src_udp_ip                      = '192.168.43.215' # maczem GO3
dst_udp_ip                      = '192.168.43.227' # maczem raspberry pi 3b+
#dst_udp_ip                      = '192.168.43.215' # maczem GO3
data_udp_port                    = 10005

hello = "\n\n##########################################\n############# mmradar started ############\n##########################################"

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
        match cfg_chirp :
            case 0 : # do nothing
                logging.info ( f"############# Device no cfg.\n")
            case 1 :
                mmradar3_ops.mmradar_conf ( cfg_chirp_start_file_name , conf_com )
                logging.info ( f"############# Device started.\n")
            case 2 : # full cfg
                mmradar3_ops.mmradar_conf ( cfg_chirp_full_file_name , conf_com )
                print ( "\n############# Device full cfg.\n" )
                logging.info ( f"############# Device full cfg.\n")
        conf_com.close ()
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

match data_destination :
    case 0 :
        pass
    case 1 :
        ################ SOCKET Configuration
        udp = socket.socket ( socket.AF_INET , socket.SOCK_DGRAM , socket.IPPROTO_UDP )
    case 2 :
        pass

i = 0
while i < frames_limit :
    match data_source :
        case 0 :
            pc3d_object = mmradar3_pc3d.PC3D ( data_com )
            if pc3d_object.get_frame_header () :
                pc3d_object.get_tlvs ()
        case 1 :
            logging.info (f"Error: data_source = 1. App exit!\n")
            exit
        case 2 :
            pass
    match data_destination :
        case 0 :
            pass
        case 1 :
            udp.sendto ( str ( pc3d_object.frame_dict ) , ( dst_udp_ip , data_udp_port ) )
        case 2 :
            file_ops2.write_2_local_file ( saved_parsed_data_file_name , str ( pc3d_object.frame_dict ) )
    del pc3d_object
udp.close ()
data_com.close ()
