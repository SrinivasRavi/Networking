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

int N;
static int baseA = 1;
static int last_max_ack = 0;
static int num_of_acked_packets = 0;
static int num_of_timedout_packets = 0;
static int nextseqnumA = 1;

static int seqExpectedB = 1;


std::queue<msg> bufferqueue;
std::queue<pkt> windowAqueue;
std::queue<pkt> windowAqueueRetransmit;

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
void A_output(struct msg message)//rdt_send(data)
{

    N = getwinsize();
    
    //Check if the sent 'message' can be sent with sequence 'nextseqnum' --> if nextseqnum fits in the window, send the data
    //if(nextseqnum<base+N){
    if(nextseqnumA<baseA+N)
    {
        //sndpkt[nextseqnum] = make_pkt(nextseqnum,data,checksum)
        datapacketA.seqnum = nextseqnumA;
        strcpy(datapacketA.payload, message.data);
        datapacketA.checksum = chcksm(datapacketA.seqnum, datapacketA.acknum, datapacketA.payload);
        
        //udt_send(sndpkt[nextseqnum])
        tolayer3(0, datapacketA);
        
        windowAqueue.push(datapacketA);//windowAqueue --> helpful for retransmission
        
        //if(base==nextseqnum){
        if(baseA==nextseqnumA)
        {
            //start_timer}
            starttimer(0,30.0);
        }
        
        //nextseqnum++ }
        nextseqnumA++;
    
    }
    //else refuse_data --> But we will buffer the data
    else
    {
        bufferqueue.push(message);
    }
}

/* called from layer 3, when a packet arrives for layer 4 */
void A_input(struct pkt ackpacketA)//rdt_rcv(rcvpkt)..
{
    //Check if the recieved packet is not corrupt and in the window range
    //..&&notcorrupt(rcvpkt)
    if(ackpacketA.checksum == chcksm(ackpacketA.seqnum, ackpacketA.acknum, ackpacketA.payload) && (nextseqnumA>ackpacketA.acknum) && (ackpacketA.acknum>=baseA) && (ackpacketA.acknum>last_max_ack))
    //>=baseA not required as last_max_ack takes care of the minimality condition
    {
        usleep(500);
        last_max_ack = ackpacketA.acknum;//last_max_ack ensures the expected lower seq nos out of order should not disturb the regular flow
        num_of_acked_packets = ackpacketA.acknum - baseA + 1;

        //base=getacknum(rcvpkt)+1
        baseA = ackpacketA.acknum + 1;
        
        for(int i = 0; i<num_of_acked_packets; i++)
        {
            //pop out the packets from the queue - windowAqueue --> windowAqueue.pop();
            //populate(push) into this queue from the bufferqueue --> windowAqueue()
            //also send those packets to B.
            
             windowAqueue.pop();

            //send next message in buffer queue --> rdt_send(data)
             if(bufferqueue.empty()==0)//if non-empty
             {
                 memcpy(messageA.data, bufferqueue.front().data,20); //copy the first message in the queue
                 bufferqueue.pop(); //remove the first message as it is read
             
                 //sndpkt[nextseqnum] = make_pkt(nextseqnum,data,checksum)
                 datapacketA.seqnum = nextseqnumA;
                 memcpy(datapacketA.payload, messageA.data,20);
                 datapacketA.checksum = chcksm(datapacketA.seqnum, datapacketA.acknum, datapacketA.payload);
             
                 //udt_send(sndpkt[nextseqnum])
                 tolayer3(0, datapacketA);
             
                 windowAqueue.push(datapacketA);
             
                 //if(base==nextseqnum){
                 if(baseA==nextseqnumA)
                 {
                     //start_timer}
                     //r//starttimer(0,30.0);
                 }
             
                 //nextseqnum++ }
                 nextseqnumA++;

             }
             else
             {
                 //buffer queue is empty. wait for message from above --> do nothing
             }
        }
        
        //Case where there is no more data arriving from the A's application
        //if(base==nextseqnum)
        if(baseA == nextseqnumA)
        {
            //stop_timer
            stoptimer(0);
        }
        //Other cases - like where more acks are expected. so if base object is received, a new timer is started for the new base. as long as acks keep coming, timer keeps getting pushed. The earlier timer is no longer used.
        //else
        else
        {
            //stoptimer(0);//to prevent the warning - starting an already running timer
            
            //start_timer
            starttimer(0,30.0);
        }
    }
    //packet is corrupted and out of the window range.
    //..&&corrupt(rcvpkt)
    else
    {
        //^
        //Do nothing.
    }
}

/* called when A's timer goes off */
void A_timerinterrupt()//timeout
{
    //All packets from base to the nextseqnum-1 are resent. timer is started again for the base. (earlier also the timer for the base timed-out)
    windowAqueueRetransmit = windowAqueue;
    num_of_timedout_packets = windowAqueue.size();
    
    for(int i = 0; i<windowAqueue.size(); i++)
    {
        tolayer3(0, windowAqueueRetransmit.front());
        windowAqueueRetransmit.pop();
    }
    
    //start_timer
    starttimer(0,30.0);
}

/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init()//^
{
    //base=1
    baseA = 1;
    
    //nextseqnum=1
    nextseqnumA = 1;
}

/* Note that with simplex transfer from a-to-B, there is no B_output() */

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt datapacketB)//rdt_rcv(rcvpkt)..
{
    //Send ack for the expectedseqnum-1 packet
    
    //..&&notcorrupt(rcvpkt)&&hasseqnum(rcvpkt,expectedseqnum)
    if(datapacketB.checksum == chcksm(datapacketB.seqnum, datapacketB.acknum, datapacketB.payload) && seqExpectedB == datapacketB.seqnum)
    {
    
        //extract(rcvpkt,data)
        memcpy(messageB.data, datapacketB.payload, 20);

        //deliver_data(data)
        tolayer5(1, messageB.data);
    
        //sndpkt = make_pkt(expectedseqnum,ACK,checksum)
        ackpacketB.acknum = seqExpectedB;//or try ackpacketB.acknum = datapacketB.seqnum;
        ackpacketB.checksum = chcksm(ackpacketB.seqnum, ackpacketB.acknum, ackpacketB.payload);
    
        //udt_send(sndpkt)
        tolayer3(1, ackpacketB);
    
        //expectedseqnum++
        seqExpectedB++;
    }
    //..default
    else
    {
        ackpacketB.acknum = seqExpectedB - 1;//acknum of last received and accepted packet
     
        if(ackpacketB.acknum >= 1) //special case: when first packet is lost and next packets arrive at B --> don't ack anything!(implemented outside of this 'if')
        {
            ackpacketB.checksum = chcksm(ackpacketB.seqnum, ackpacketB.acknum, ackpacketB.payload);
            
            //udt_send(sndpkt)
            tolayer3(1, ackpacketB);
        }
    }
}

/* the following rouytine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init()
{
    seqExpectedB = 1; //first sequence number expected by the receiver
    strcpy(messageB.data,"arndmstringof20chars");
}
