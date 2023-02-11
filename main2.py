# Issue 1 - tlv list

import sys
#sys.setdefaultencoding('utf-8')
sys.path.append ( "/Users/mzeml/python/mmradar/modules/" )
import logging
import pprint

import datetime
import file_ops2
import mmradar_ops2
import mmradar_pc3d2
import time

#from mmradar_ops2 import mmradar_conf
#from file_ops2 import write_data_2_local_file

################################################################
### DEFINITIONS 
################################################################

data_source                     = 2 # 0: device, 1: UDP, 2: file
chirp_cfg                       = 0 # 0: no cfg, 1: only start(), 2: full cfg
raws                            = bytes(1)
log_file_name                   = 'log/mmradar.log'
data_file_name                  = 'mmradar.data'
#saved_bin_data_file_name        = 'saved_bin_data\mmradar_gen_1675550277997491400.bin_raw_data' # poprawić nazwę katalogu na saved
saved_bin_data_file_name        = 'saved_bin_data\mmradar_gen_1675746223207587500.bin_raw_data'
#saved_bin_data_file_name        = 'saved_bin_data\mmradar_gen_1675746902363904100.bin_raw_data'
#saved_bin_data_file_name        = 'save_bin_data\mmradar_gen_1675550277997491400_edited.bin_raw_data' # poprawić nazwę katalogu na saved
pc3d_cfg_file_name              = 'chirp_cfg/ISK_6m_default-mmwvt-v14.11.0.cfg'
mmradar_stop_conf_file_name     = 'chirp_cfg/sensor_stop.cfg'
mmradar_start_conf_file_name    = 'chirp_cfg/sensor_start.cfg' 

hello = "\n\n##########################################\n############# mmradar started ############\n##########################################\n"

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
        logging.info (f"Error: data_source = 0. App exit!\n")
        exit
    case 1:
        logging.info (f"Error: data_source = 1. App exit!\n")
        exit
    case 2:
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
    #pc3d_object.get_frame_header ()
    #pprint.pprint ( pc3d_object.frame_header_json )
    #pc3d_object.get_frame_header ()
    #frame_json_2_file += pc3d_object.frame_json_2_file
    #frame_json_2_azure += pc3d_object.frame_json_2_azure
    del pc3d_object
