#!/usr/bin/env python

################################################################################
# Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

import sys, os
import px_generate_uorb_topic_files

if len(sys.argv) > 1:
    msg_files = sys.argv[1:]
else:
    print("At least one .msg file must be specified")
    exit(-1)

root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
msg_folder = root_path + "/msg"
px_generate_uorb_topic_files.append_to_include_path({msg_folder}, px_generate_uorb_topic_files.INCL_DEFAULT)

out_dir = root_path + "/msgenerated"
print("Files created in: " + out_dir)

uorb_templates_dir = root_path + "/msg/templates/uorb"
urtps_templates_dir = root_path + "/msg/templates/urtps"



uRTPS_TRANS_APP_GEN_TEMPL_FILE = 'general_uRTPS_UART_transmitter.cpp.template'
uRTPS_TRANS_CML_GEN_TEMPL_FILE = 'general_transmitter_CMakeLists.txt.template'
uRTPS_RECEV_H_GEN_TEMPL_FILE = 'general_uRTPS_UART_receiver.h.template'
uRTPS_RECEV_SRC_GEN_TEMPL_FILE = 'general_uRTPS_UART_receiver.cxx.template'
uRTPS_PUBSUBMAIN_GEN_TEMPL_FILE = 'general_uRTPS_UART_PubSubMain.cxx.template'
uRTPS_PUBLISHER_SRC_TEMPL_FILE = 'Publisher.cxx.template'
uRTPS_PUBLISHER_H_TEMPL_FILE = 'Publisher.h.template'
uRTPS_SUBSCRIBER_SRC_TEMPL_FILE = 'Subscriber.cxx.template'
uRTPS_SUBSCRIBER_H_TEMPL_FILE = 'Subscriber.h.template'


for msg_file in msg_files:
	px_generate_uorb_topic_files.generate_idl_file(msg_file, out_dir, urtps_templates_dir, 
		px_generate_uorb_topic_files.INCL_DEFAULT)
	px_generate_uorb_topic_files.generate_topic_file(msg_file, out_dir, urtps_templates_dir, 
		px_generate_uorb_topic_files.INCL_DEFAULT, uRTPS_PUBLISHER_SRC_TEMPL_FILE)
	px_generate_uorb_topic_files.generate_topic_file(msg_file, out_dir, urtps_templates_dir, 
		px_generate_uorb_topic_files.INCL_DEFAULT, uRTPS_PUBLISHER_H_TEMPL_FILE)
	px_generate_uorb_topic_files.generate_topic_file(msg_file, out_dir, urtps_templates_dir, 
		px_generate_uorb_topic_files.INCL_DEFAULT, uRTPS_SUBSCRIBER_SRC_TEMPL_FILE)
	px_generate_uorb_topic_files.generate_topic_file(msg_file, out_dir, urtps_templates_dir, 
		px_generate_uorb_topic_files.INCL_DEFAULT, uRTPS_SUBSCRIBER_H_TEMPL_FILE)

px_generate_uorb_topic_files.generate_uRTPS_general(msg_files, out_dir, uorb_templates_dir, 
				px_generate_uorb_topic_files.INCL_DEFAULT, uRTPS_TRANS_APP_GEN_TEMPL_FILE)

px_generate_uorb_topic_files.generate_uRTPS_general(msg_files, out_dir, uorb_templates_dir, 
				px_generate_uorb_topic_files.INCL_DEFAULT, uRTPS_TRANS_CML_GEN_TEMPL_FILE)

px_generate_uorb_topic_files.generate_uRTPS_general(msg_files, out_dir, urtps_templates_dir, 
				px_generate_uorb_topic_files.INCL_DEFAULT, uRTPS_RECEV_H_GEN_TEMPL_FILE)

px_generate_uorb_topic_files.generate_uRTPS_general(msg_files, out_dir, urtps_templates_dir, 
				px_generate_uorb_topic_files.INCL_DEFAULT, uRTPS_RECEV_SRC_GEN_TEMPL_FILE)

px_generate_uorb_topic_files.generate_uRTPS_general(msg_files, out_dir, urtps_templates_dir, 
				px_generate_uorb_topic_files.INCL_DEFAULT, uRTPS_PUBSUBMAIN_GEN_TEMPL_FILE)


