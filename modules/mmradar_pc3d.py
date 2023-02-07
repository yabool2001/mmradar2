import logging
import pprint
import time
import struct


class PC3D :
    def __init__ ( self , raw_data ) :
        self.raw_data = raw_data
        self.control = 506660481457717506
        self.tlv_type_point_cloud = 1020
        self.tlv_type_target_list = 1010
        self.tlv_type_target_index = 1011
        self.tlv_type_target_height = 1012
        self.tlv_type_presence_indication = 1021
        # old firmwware self.frame_header_struct = 'Q9I2H'
        self.frame_header_struct = 'Q8I'
        self.frame_header_length = struct.calcsize ( self.frame_header_struct )
        self.frame_header_dict = dict ()
        self.frame_header_json = None
        self.tlv_header_struct = '2I'
        self.tlv_header_length = struct.calcsize ( self.tlv_header_struct )
        self.tlv_header_dict = dict ()
        self.tlv_header_json = None
        self.tlv_list = []
        self.tlvs_json = None
        self.point_cloud_unit_struct = '5f'
        self.point_cloud_unit_length = struct.calcsize ( self.point_cloud_unit_struct )
        self.point_struct = '2B3h'
        self.point_length = struct.calcsize ( self.point_struct )
        self.points_list = []
        self.points_json = None
        self.pointcloud_unit_dict = dict ()
        self.pointcloud_unit_json = None
        self.target_struct = 'I27f'
        self.target_part1_struct = 'I9f'
        self.target_part2_struct = '16f'
        self.target_part3_struct = '2f'
        self.target_length = struct.calcsize ( self.target_struct )
        self.target_part1_length = struct.calcsize ( self.target_part1_struct )
        self.target_part2_length = struct.calcsize ( self.target_part2_struct )
        self.target_part3_length = struct.calcsize ( self.target_part3_struct )
        self.targets_list = []
        self.targets_json = None
        self.presence_indication_struct = '1I'
        self.presence_indication_length = struct.calcsize ( self.presence_indication_struct )
        self.presence_indication_value = None
        self.presence_indication_json = None
        self.frame_json_2_azure = None
        self.frame_json_2_file = None

    def send_data_2_azure ( self ) :
        pass
        # Azure part
        #try :
        #    azure_client.send_message ( f"\n\n{{frame:{self.frame_header_json},{self.tlvs_json}}}" )
        #except :
        #    print ( "Azure error connecting or sending message")

    # Zdekodowanie wszystkich punktów z ramki zaczynającej się od Punktów
    # Zapisanie punktów do dict, zapisanie słownika do pliku i skasowanie słownika
    # Usunięcie Punktu z ramki w każdej iteracji

    def get_targets_list ( self , tlv_length ) :
        #  trzeba zastanowić sie i przerobić na listę dictionary
        targets_number = int ( ( tlv_length - self.tlv_header_length ) / self.target_length )
        for i in range ( targets_number ) :
            try :
                target_id , target_pos_x , target_pos_y , target_pos_z , target_vel_x , target_vel_y , target_vel_z , target_acc_x , target_acc_y , target_acc_z = struct.unpack ( self.target_part1_struct , self.raw_data[(self.tlv_header_length) + ( i * self.target_length ):][:self.target_part1_length] )
                # Zostawiam err_covariance[16] na później
                err_covariance = struct.unpack ( self.target_part2_struct , self.raw_data[(self.tlv_header_length) + ( i * self.target_length ) + self.target_part1_length:][:self.target_part2_length] )
                gain , confidence_level = struct.unpack ( self.target_part3_struct , self.raw_data[(self.tlv_header_length) + ( i * self.target_length ) + self.target_part1_length + self.target_part2_length:][:self.target_part3_length] )
                # Zapisz punkt
                if target_id :
                    self.targets_list.append ( f"{{'target_id':{target_id},'target_pos_x':{target_pos_x}, 'target_pos_y':{target_pos_y},'target_pos_z':{target_pos_z},'target_vel_x':{target_vel_x}, 'target_vel_y':{target_vel_y},'target_vel_z':{target_vel_z},'target_acc_x':{target_acc_x},'target_acc_y':{target_acc_y},'target_acc_z':{target_acc_z},'err_covariance':{err_covariance},'gain':{gain},'confidence_level':{confidence_level}}}" )
            except struct.error as e :
                self.targets_list.append ( f"{{'error':'{e}'}}" )
                return False
        l = len ( self.targets_list )
        self.targets_json = f"'num_targets':{targets_number},'targets':["
        for i in range ( l ) :
            self.targets_json += str ( self.targets_list[i] ) #self.targets_json = self.targets_json + str ( self.targets_list[i] )
            if i < ( l - 1 ) :
                self.targets_json = self.targets_json + ","
        self.targets_json = self.targets_json + "]"
        pprint.pprint ( self.targets_json )
        self.targets_list.clear ()
        return True

    def get_points_list ( self , tlv_length ) :
        # przeanalizować przerobienie na listę dict
        points_number = int ( ( tlv_length - self.tlv_header_length - self.point_cloud_unit_length ) / self.point_length )
        for i in range ( points_number ) :
            try :
                # muszę zrobi c range_m, bo inaczej nie będzie działać składnia "for range", bo range zostanie uznane za lokalną zmienną
                point_elevation , point_azimuth , point_doppler , point_range , point_snr = struct.unpack ( self.point_struct , self.raw_data[(self.tlv_header_length + self.point_cloud_unit_length ) + ( i * self.point_length ):][:self.point_length] )
                # Zapisz punkt
                if point_doppler :
                    self.points_list.append ( f"{{'elevation':{point_elevation} , 'azimuth':{point_azimuth} , 'doppler':{point_doppler} , 'range':{point_range} , 'snr':{point_snr}}}" )
            except struct.error as e :
                self.points_list.append ( f"{{'error':'{e}'}}" )
        l = len ( self.points_list )
        self.points_json = f"'num_points':{points_number},'points':["
        for i in range ( len ( self.points_list ) ) :
            self.points_json += str ( self.points_list[i] ) #self.points_json = self.points_json + str ( self.points_list[i] )
            if i < ( l - 1 ) :
                self.points_json = self.points_json + ","
        self.points_json = self.points_json + "]"
        self.points_list.clear ()

    def get_point_cloud_unit ( self ) :
        try :
            elevation_unit , azimuth_unit , doppler_unit , range_unit , snr_unit = struct.unpack ( self.point_cloud_unit_struct , self.raw_data[self.tlv_header_length:][:self.point_cloud_unit_length] )
            self.pointcloud_unit_dict = { 'elevation_unit' : elevation_unit , 'azimuth_unit' : azimuth_unit , 'doppler_unit' : doppler_unit , 'range_unit' : range_unit , 'snr_unit' : snr_unit }
        except struct.error as e :
            self.pointcloud_unit_dict = { 'error' : {e} }
        self.pointcloud_unit_json = f"'point_cloud_unit':{self.pointcloud_unit_dict}"
        # pprint.pprint ( self.pointcloud_unit_json )
        if self.pointcloud_unit_dict.get ( 'error' ) :
            return False
        else:
            return True

    def get_presence_indication ( self ) :
        try :
            presence_indication = struct.unpack ( self.presence_indication_struct , self.raw_data[self.tlv_header_length:][:self.presence_indication_length] )
            self.presence_indication_value = presence_indication[0] # Dlacze otrzymuję to jako tuple, a nie int 32bit
            self.presence_indication_json = f"'presence_indication':{self.presence_indication_value}"
            #print ( type(presence_indication) )
            #print ( type(self.presence_indication_value) )
            #print ( f"{self.presence_indication_value}" )
            pprint.pprint (self.presence_indication_json)
            return True
        except struct.error as e :
            self.presence_indication_json = f"{{'presence_indication':{{'error':'{e}'}}}}"
            return False

    def get_tlv_header ( self ) :
        try:
            tlv_type, tlv_length = struct.unpack ( self.tlv_header_struct , self.raw_data[:self.tlv_header_length] )
            self.tlv_header_dict = { 'tlv_type' : tlv_type , 'tlv_length' : tlv_length }
        except struct.error as e :
            self.tlv_header_dict = { 'error' : {e} }
            logging.info ( f"TLV Header unpack error during frame unpack number: {self.frame_header_dict['frame_number']}" )
        self.tlv_header_json = f"'tlv_header':{self.tlv_header_dict}"
        if tlv_type != self.tlv_type_point_cloud : 
            pprint.pprint ( self.tlv_header_json )
        if self.tlv_header_dict.get ( 'error' ) :
            return False
        else:
            return True

    def get_tlv ( self ) :
        if self.get_tlv_header () :
            match self.tlv_header_dict.get ( 'tlv_type' ) :
                case self.tlv_type_point_cloud :
                    if self.get_point_cloud_unit () : 
                        self.get_points_list ( self.tlv_header_dict['tlv_length'] )
                        self.tlv_list.append ( f"{{tlv:{{{self.tlv_header_json},{self.pointcloud_unit_json},{self.points_json}}}}}" )
                    else : # tutaj coś jest nie tak, bo jesli jest False to czemu wstawiam to unit? jeśli chodzi o point to trzeba to poprawić
                        self.tlv_list.append ( f"{{tlv:{{{self.tlv_header_json},{self.pointcloud_unit_json}}}}}" )
                case self.tlv_type_target_list :
                    if self.get_targets_list ( self.tlv_header_dict['tlv_length'] ) :
                        self.tlv_list.append ( f"{{tlv:{{{self.tlv_header_json},{self.targets_json}}}}}" )
                    else :
                        self.tlv_list.append ( f"{{tlv:{{{self.tlv_header_json}}}}}" )
                case self.tlv_type_target_index :
                    self.tlv_list.append ( f"{{tlv:{{{self.tlv_header_json}}}}}" )
                case self.tlv_type_target_height :
                    pass
                case self.tlv_type_presence_indication :
                    if self.get_presence_indication () :
                        self.tlv_list.append ( f"{{tlv:{{{self.tlv_header_json},{self.presence_indication_json}}}}}" )
                case _ :
                    self.raw_data = self.raw_data[self.tlv_header_dict['tlv_length']:]
            return True
        else :
            self.tlv_list.append ( f"{{tlv:{{{self.tlv_header_json}}}}}" )
            return False

    # Rozpakuj po kolei każdy z TLV
    def get_tlvs ( self ) :
        for i in range ( self.frame_header_dict['number_of_tlvs'] ) : # self.frame_header_dict['num_tlvs'] exists for sure and I don't need get function.
            if not self.get_tlv () :
                logging.info ( f"Break 1 in function: with frame number: {self.frame_header_dict['frame_number']}" )
                break
        l = len ( self.tlv_list )
        self.tlvs_json = "'tlvs':["
        for i in range ( l ) :
            self.tlvs_json = self.tlvs_json + str ( self.tlv_list[i] )
            if i < ( l - 1 ) :
                self.tlvs_json = self.tlvs_json + ","
        self.tlvs_json = self.tlvs_json + "]"
        self.tlv_list.clear ()
        # pprint.pprint(self.tlvs_json)
            
    # Rozpakuj i zapisz dane z Frame header
    def get_frame_header ( self ) :
        try:
            magic_word , version , total_packet_length , platform , frame_number , time , number_of_points , number_of_tlvs , subframe_number = struct.unpack ( self.frame_header_struct , self.raw_data[:self.frame_header_length] )
            if magic_word == self.control :
                self.frame_header_dict = { 'frame_number' : frame_number , 'number_of_tlvs' : number_of_tlvs , 'number_of_points' : number_of_points , 'subframe_number' : subframe_number , 'version' : version , 'total_packet_length' : total_packet_length , 'platform' : platform , 'time' : time }
            else :
                self.frame_header_dict = { 'error' : {magic_word} }
                logging.info ( f"Frame header magic word is not corrected: {magic_word}" )
        except struct.error as e :
            self.frame_header_dict = { 'error' : {e} }
            logging.info ( f"Frame header unpack error during frame unpack number: {self.frame_header_dict}" )
        self.frame_header_json = f"{{'frame_header':{self.frame_header_dict}}}"
        # pprint.pprint ( self.frame_header_json ) # do usunięcia

    def get_json_data ( self ) :
        self.get_frame_header ()
        if ( self.frame_header_dict.get ( 'number_of_tlvs' ) ) : # To jest super sprawdzone. Nie zmieniaj! get jest bezpieczne, bo w tym miejscu nie jest jeszcze pewny czy self.frame_header_dict['num_tlvs'] istnieje i jego brak mógłby spowodować błąd w programie
            self.raw_data = self.raw_data[self.frame_header_length:]
            self.get_tlvs ()
        self.frame_json_2_file = f"\n\n{{frame:{self.frame_header_json},timestamp_ns:{time.time_ns ()},{self.tlvs_json}}}"
        self.frame_json_2_azure = f"{{'id' : {time.time_ns ()},'frame_number':{self.frame_header_dict.get('frame_number')},'presence':{self.presence_indication_value}}}"
        #self.frame_json_2_azure = f"{{'id' : {datetime.datetime.utcnow().strftime('%Y%m%d%H%M%S%f')[:17]},'frame_number':{self.frame_header_dict.get('frame_number')},'presence':{self.presence_indication_value}}}"
        #self.frame_json_2_azure = f"{{'frame':{{'frame_number':{self.frame_header_dict.get('frame_number')},'presence':{random.randint ( 0, 12 )}}}"