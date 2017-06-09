// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*! 
 * @file sensor_combined_Publisher.cpp
 * This file contains the implementation of the publisher functions.
 *
 * This file was generated by the tool fastcdrgen.
 */


#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/publisher/Publisher.h>
#include <fastrtps/attributes/PublisherAttributes.h>

#include <fastrtps/Domain.h>

#include <fastrtps/utils/eClock.h>

#include "sensor_combined_Publisher.h"




sensor_combined_Publisher::sensor_combined_Publisher() : mp_participant(nullptr), mp_publisher(nullptr) {}

sensor_combined_Publisher::~sensor_combined_Publisher() {	Domain::removeParticipant(mp_participant);}

bool sensor_combined_Publisher::init()
{
	// Create RTPSParticipant
	
	ParticipantAttributes PParam;
	PParam.rtps.builtin.domainId = 0;
	PParam.rtps.builtin.leaseDuration = c_TimeInfinite;
	PParam.rtps.setName("Participant_publisher");  //You can put here the name you want
	mp_participant = Domain::createParticipant(PParam);
	if(mp_participant == nullptr)
		return false;
	
	//Register the type
	
	Domain::registerType(mp_participant,(TopicDataType*) &myType);
	
	// Create Publisher
	
	PublisherAttributes Wparam;
	Wparam.topic.topicKind = NO_KEY;
	Wparam.topic.topicDataType = myType.getName();  //This type MUST be registered
	Wparam.topic.topicName = "sensor_combined_PubSubTopic";
	mp_publisher = Domain::createPublisher(mp_participant,Wparam,(PublisherListener*)&m_listener);
	if(mp_publisher == nullptr)
		return false;
	std::cout << "Publisher created, waiting for Subscribers." << std::endl;
	return true;
}

void sensor_combined_Publisher::PubListener::onPublicationMatched(Publisher* pub,MatchingInfo& info)
{
	if (info.status == MATCHED_MATCHING)
	{
		n_matched++;
		std::cout << "Publisher matched" << std::endl;
	}
	else
	{
		n_matched--;
		std::cout << "Publisher unmatched" << std::endl;
	}
}

void sensor_combined_Publisher::run()
{
	while(m_listener.n_matched == 0)
	{
		eClock::my_sleep(250); // Sleep 250 ms
	}
	
	// Publication code
	
	sensor_combined_ st;
	
	/* Initialize your structure here */
	
	int msgsent = 0;
	char ch = 'y';
	do
	{
		if(ch == 'y')
		{
			mp_publisher->write(&st);  ++msgsent;
			std::cout << "Sending sample, count=" << msgsent << ", send another sample?(y-yes,n-stop): ";
		}
		else if(ch == 'n')
		{
			std::cout << "Stopping execution " << std::endl;
			break;
		}
		else
		{
			std::cout << "Command " << ch << " not recognized, please enter \"y/n\":";
		}
	}while(std::cin >> ch);
}
