#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <ctype.h>
#include <string.h>
#include <iostream>

#include <queue>

#include "../include/simulator.h"

/* ******************************************************************
 ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.1  J.F.Kurose

   This code should be used for PA2, unidirectional data transfer 
   protocols (from A to B). Network properties:
   - one way network delay averages five time units (longer if there
     are other messages in the channel for GBN), but can be larger
   - packets can be corrupted (either the header or the data portion)
     or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
     (although some can be lost).
**********************************************************************/

/********* STUDENTS WRITE THE NEXT SEVEN ROUTINES *********/

static struct pkt datapacketA;
static struct pkt datapacketB;
static struct pkt ackpacketA;
static struct pkt ackpacketB;

static struct msg messageA;
static struct msg messageB;
static int stateA = 0;
static int ackExpectedA = 0;
static int seqExpectedB = 0;

std::queue<msg> bufferqueue;

/*calculate checksum and return it*/
int chcksm(int seq, int ack, char *data)
{
	int checksum = 0;
	checksum += (seq+ack);
	for(int i = 0; i < 20; i++)
	{
		checksum += data[i];
	}
	return checksum;
}
/* called from layer 5, passed the data to be sent to other side */
void A_output(struct msg message) //rdt_send(data)
{
	if(stateA==0 && bufferqueue.empty()==1)//bufferqueue.empty() - does this work?
	{
		//sndpkt = make_pkt(seqnum(0/1), data, checksum)
		if(ackExpectedA== 1)
		{
			datapacketA.seqnum = 0;
			ackExpectedA = 0;
		}
		else if (ackExpectedA == 0)
		{
			datapacketA.seqnum = 1;
			ackExpectedA = 1;
		}
		strcpy(datapacketA.payload, message.data);
		datapacketA.checksum = chcksm(datapacketA.seqnum, datapacketA.acknum, datapacketA.payload);

		//udt_send(pkt)
		tolayer3(0, datapacketA);

		//start_timer
		starttimer(0,30.0);

		//move to state ->  waiting for ACK (0/1)
		stateA = 1;
    }
	else if (stateA==1 || bufferqueue.empty()==0)
	{
		bufferqueue.push(message);
	}
}

/* called from layer 3, when a packet arrives for layer 4 */
void A_input(struct pkt ackpacketA)//rdt_rcv(rcvpkt).. 
{
	//if in 'Wait for ACK' state //.. && notcorrupt(rcvpkt) && isACK(rcvpkt, <expectedAckNo>)
	if(stateA == 1 && ackpacketA.checksum == chcksm(ackpacketA.seqnum, ackpacketA.acknum, ackpacketA.payload) && ackExpectedA == ackpacketA.acknum)
	{
		//stop_timer
		stoptimer(0);

		//move to state --> Wait for call (0/1) from above
		stateA = 0;

		//send next message in buffer queue --> rdt_send(data)
		if(bufferqueue.empty()==0)
		{
			strcpy(messageA.data, bufferqueue.front().data); //copy the first message in the queue
			bufferqueue.pop(); //remove the first message as it is read

			//sndpkt = make_pkt(seqnum(0/1), data, checksum)
			if(ackExpectedA== 1)
			{
				datapacketA.seqnum = 0;
				ackExpectedA = 0;
			}
			else if (ackExpectedA== 0)
			{
				datapacketA.seqnum = 1;
				ackExpectedA = 1;
			}
			strcpy(datapacketA.payload, messageA.data);
			datapacketA.checksum = chcksm(datapacketA.seqnum, datapacketA.acknum, datapacketA.payload);

			//udt_send(pkt)
			tolayer3(0, datapacketA);

			//start_timer
			starttimer(0,30.0);

			//move to state ->  waiting for ACK (0/1)
			stateA = 1;
		}
		else 
		{
			//buffer queue is empty. wait for message from above --> do nothing
        }
	}
	//.. && (corrupt(rcvpkt) || isACK(rcvpkt, !<expectedAckNo>) and if in 'Wait for call from above' state
	else 
	{
		//ack from B is corrupt --> do nothing
	}

}

/* called when A's timer goes off */
void A_timerinterrupt() //timeout
{
	//stateA should be 1 i.e the current state should be 'Wait for ACK' state. It most likely always is when here.
	//udt_send(pkt)
	tolayer3(0, datapacketA); //retransmit datapacketA
    
	//start_timer
	starttimer(0,30.0);
}

/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init()
{
	stateA = 0; //state = 0 --> wait for call 0 or 1 from above. //state = 1 --> wait for ACK 1 or 0 (from below)
	ackExpectedA = 1;
	strcpy(messageA.data,"arndmstringof20chars");
}

/* Note that with simplex transfer from a-to-B, there is no B_output() */

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt datapacketB)//rdt_rcv(rcvpkt).. 
{
	//..&&notcorrupt(rcvpkt) && hasseq<expectedSeqNo>(rcvpkt)
	if(datapacketB.checksum == chcksm(datapacketB.seqnum, datapacketB.acknum, datapacketB.payload) && seqExpectedB == datapacketB.seqnum)
	{
		//extract(rcvpkt,data)
		memcpy(messageB.data, datapacketB.payload, 20);

		//deliver_data(data)
		tolayer5(1, messageB.data);
        
        //sndpkt = make_pkt(ACK,<expectedSeqNo>,checksum)
		ackpacketB.acknum = datapacketB.seqnum; //ackpacketB's seqnum, payload are irrelevant
		ackpacketB.checksum = chcksm(ackpacketB.seqnum, ackpacketB.acknum, ackpacketB.payload);
		
		//move to 'Wait for call !<expectedSeqNo>' state --> update next expected sequence number
		if(seqExpectedB== 0)
		{
			seqExpectedB = 1;
		}
		else //if(datapacketB.seqnum == 1)
		{
			seqExpectedB = 0;
		}

		//udt_send(sndpkt)
		tolayer3(1, ackpacketB);
        
    }
	//..&&(corrupt(rcvpkt) || hasseq!<expectedSeqNo>(rcvpkt))
	else
	{
		//sndpkt = make_pkt(ACK, !<expectedSeqNo>, checksum)
		if(seqExpectedB== 0)
		{
			ackpacketB.acknum = 1;
		}
		else //if(datapacketB.seqnum == 1)
		{
			ackpacketB.acknum = 0;
		}
		//ackpacketB.acknum = datapacketB.seqnum; //ackpacketB's seqnum, payload are irrelevant
		ackpacketB.checksum = chcksm(ackpacketB.seqnum, ackpacketB.acknum, ackpacketB.payload);
		
		//udt_send(pkt)
		tolayer3(1, ackpacketB); 

	}	

}

/* the following rouytine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init()
{
	datapacketB.seqnum = 0;
	datapacketB.acknum = 1; //irrelevant
	strcpy(datapacketB.payload,"arndmstringof20chars"); //for checksum calculation to not return error
	datapacketB.checksum = 0;

	ackpacketB.acknum = 1;
	ackpacketB.seqnum = 0; //irrelevant
	strcpy(ackpacketB.payload,"arndmstringof20chars"); //for checksum calculation to not return error
	ackpacketB.checksum = 0;

	seqExpectedB = 0;

	strcpy(messageB.data,"arndmstringof20chars");
}
