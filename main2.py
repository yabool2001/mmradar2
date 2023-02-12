# Issue 1 - tlv list
# Zamienić wszystkie self.tlv_header_length na self.tl_length
# Napisać do ti, że tlv_length to jest tl_length. Jeśli to jest tlv, to mnie poprawcie. Moim zdaniem lepiej żeby była podawana całą wartość tlv

import sys
#sys.setdefaultencoding('utf-8')
sys.path.append ( "/Users/mzeml/python/mmradar2/modules/" )
import logging
import pprint

import datetime
import file_ops2
import mmradar_ops2
import mmradar_pc3d2
import serial
import serial_ops2
import time

#from mmradar_ops2 import mmradar_conf
#from file_ops2 import write_data_2_local_file

################################################################
### DEFINITIONS 
################################################################

data_source                     = 0 # 0: device, 1: UDP, 2: file
chirp_cfg                       = 0 # 0: no cfg, 1: only start(), 2: full cfg
raws                            = bytes(1)
log_file_name                   = 'log/mmradar.log'
data_file_name                  = 'mmradar.data'
data_json_file_name             = 'save_parsed_data\mmradar.data.json'
#saved_bin_data_file_name        = 'saved_bin_data\mmradar_gen_1675550277997491400.bin_raw_data' # poprawić nazwę katalogu na saved
saved_bin_data_file_name        = 'saved_bin_data\mmradar_gen_1675746223207587500.bin_raw_data'
#saved_bin_data_file_name        = 'saved_bin_data\mmradar_gen_1675746902363904100.bin_raw_data'
#saved_bin_data_file_name        = 'save_bin_data\mmradar_gen_1675550277997491400_edited.bin_raw_data' # poprawić nazwę katalogu na saved
chirp_conf_file_name            = 'chirp_cfg/ISK_6m_staticRetention.cfg'

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
        conf_com = serial.Serial ()
        data_com = serial.Serial ()
        serial_ops2.set_serials_cfg ( conf_com , data_com )
        serial_ops2.open_serial_ports ( conf_com , data_com )
    case 1:
        logging.info (f"Error: data_source = 1. App exit!\n")
        exit
    case 2:
        print ( "\n############# Read saved raw data.\n" )
        saved_bin_frames = open ( saved_bin_data_file_name , 'r' ) .readlines ()
        saved_bin_frames_numbers = len ( saved_bin_frames )
    case _:
        logging.info (f"Error: data_source not known. App exit!\n")
        exit
i = 0
frame_json_2_azure = ''
frame_json_2_file = ''
while i < saved_bin_frames_numbers :
    match data_source :
        case 0:
            conf_com.reset_input_buffer ()
            conf_com.reset_output_buffer ()
            serial_ops2.mmradar_conf ( chirp_conf_file_name , conf_com )
            # frame = data_com.read ( 4666 )
            logging.info (f"Error: data_source = 0. App exit!\n")
            exit
        case 1 :
            logging.info (f"Error: data_source = 1. App exit!\n")
            exit
        case 2 :
            frame = eval ( saved_bin_frames[i] )
            i += 1
        case _:
            logging.info (f"Error: data_source not known. App exit!\n")
            exit
    # pprint.pprint ( frame )
    pc3d_object = mmradar_pc3d2.PC3D ( frame )
    pc3d_object.get_json_data ()
    file_ops2.write_2_local_file ( f"save_parsed_data/mmradar_parsed_data_2023.02.12.json" , str ( pc3d_object.frame_dict ) )
    del pc3d_object
