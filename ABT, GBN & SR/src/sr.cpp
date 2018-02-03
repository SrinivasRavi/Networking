#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <ctype.h>
#include <string.h>
#include <iostream>

#include <queue>
#include <list>
#include <algorithm>

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
struct pktofpkt
{
    struct pkt packet;
    float starttime; //get_sim_time() is done just before the message is transmitted from A_output() or A_timerinterrupt()
    float relative_time;//used to store relative timeouts. Set according to relative arrival time.
    int ackflag; //0 = packet not yet acknowledged. 1 = Packet acknowledged
};

struct pktofmsg
{
    char datatoB[20];
    struct msg messagetoB;
    int received_flag;
};

static struct pktofpkt pktofpkt_aoutput_initial;
static struct pktofpkt pktofpkt_aoutput_subsequent;
static struct pktofpkt pktofpkt_ainterrupt_first_node;
static struct pktofpkt pktofpkt_ainterrupt_last_node;

static struct pkt datapacketA;
static struct pkt datapacketB;
static struct pkt ackpacketA;
static struct pkt ackpacketB;

static struct msg messageA;
static struct msg messageB;

static struct pktofmsg receivedmessages[1500];
static struct pktofpkt sentpackets[1500];


static int N;
static int Nb;

static float timeout_value = 20;
static int baseA = 1;
static int nextseqnumA = 1;

static int baseB = 1;
static int maxseqnumB = 1;

static float arrival_time_packet_a_output = 0;
static float current_time = 0;
static float last_packet_starttime = 0;
static float next_timeout = 0;
static float new_next_timeout = 0;
static float early_ack_base_compensation = 0;
static float timer_starttime = 0;
static float timer_stoptime = 0;


std::queue<msg> bufferqueue; //when sender window is full this buffer is used
std::list<pktofpkt> packetwindowLL; //List of unacked packets and their other details
std::list<int> packetwindow_sequenceLL; //List of unacked packets and their other details


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
void A_output(struct msg message)
{

    N = getwinsize();
    
    //Check if the sent 'message' can be sent with sequence 'nextseqnum' --> if nextseqnum fits in the window, send the data
    //if(nextseqnum<base+N){
    if(nextseqnumA<baseA+N)
    {
        if(packetwindowLL.empty()==1)//the window has no packets on the fly. It was waiting for new packets
            //Imagine: How baseA always is lesser than nextseqnumA when messages are sent. It is equal only when all the packets sent are acked.
        {   /*
             This code block ensures that the LL - packetwindowLL is properly set up each time. The LL represents the sender window of the packets that are sent by and contains their other information too like when they are sent to the network and what's their relative time wrt to last sent packet. Note - last sent packet could be either the original message made into packet and sent into the network or could be could be the retransmitted packets that were not acked within the timeout
             */
            
            //Make_pkt
             datapacketA.seqnum = nextseqnumA;
             //strcpy(datapacketA.payload, message.data);
            memcpy(datapacketA.payload, message.data,20);
             datapacketA.checksum = chcksm(datapacketA.seqnum, datapacketA.acknum, datapacketA.payload);
            
            //Send_pkt
             tolayer3(0, datapacketA);
            
             //Create first node of the LL with timer 0.0
            //Save pkt in LL node developed in this method
             pktofpkt_aoutput_initial.packet = datapacketA; //Save the packet for retransmission later
             
              pktofpkt_aoutput_initial.relative_time = 0; // First unacked message hence the timer that would be set would be for this packet and hence this one is relatively 0.0

            sentpackets[nextseqnumA].ackflag = 0; //mark not acked
            pktofpkt_aoutput_initial.ackflag = 0; //mark not acked in the LL

              //save the arrival_time_packet_a_output in LL node developed in this method
             arrival_time_packet_a_output = get_sim_time();
              pktofpkt_aoutput_initial.starttime = get_sim_time();
             
             //Push the LL node to the linked list at the end or beginning. Same result as the LL is empty
            packetwindowLL.push_back(pktofpkt_aoutput_initial);// Insert a new element at the end - this is the first element in the packetwindowLL
            packetwindow_sequenceLL.push_back(nextseqnumA);
            
            //Starttimer(0,timeout_value)
             starttimer(0,timeout_value);
            next_timeout = timeout_value;
            
            timer_starttime = get_sim_time(); //used in A_input() to calculate elapsed time
            
        }
        else//some packets are on the fly when this message from application A arrived. Don't stop the timer!
        {
            //Make_pkt
            datapacketA.seqnum = nextseqnumA;
            //strcpy(datapacketA.payload, message.data);
            memcpy(datapacketA.payload, message.data, 20);
            datapacketA.checksum = chcksm(datapacketA.seqnum, datapacketA.acknum, datapacketA.payload);
            
            //Send_pkt
            tolayer3(0, datapacketA);
            
            //create a new LL node at the end of LL
            //Save pkt in LL node developed in this method
            pktofpkt_aoutput_subsequent.packet = datapacketA; //Save the packet for retransmission later
            
            pktofpkt_aoutput_initial = packetwindowLL.back();
            last_packet_starttime = packetwindowLL.back().starttime;
            current_time  = get_sim_time();
            pktofpkt_aoutput_subsequent.starttime = current_time;
            pktofpkt_aoutput_subsequent.relative_time = current_time - last_packet_starttime;
            
            sentpackets[nextseqnumA].ackflag = 0; //mark not acked in the array
            pktofpkt_aoutput_subsequent.ackflag = 0; //mark not acked in the LL
            
            //Push the LL node to the linked list at the end or beginning. Same result as the LL is empty
            if(packetwindowLL.empty()==0)//not necessary: check if the LL is indeed NON-empty
            {
                packetwindowLL.push_back(pktofpkt_aoutput_subsequent);// Insert a new node at the end
                packetwindow_sequenceLL.push_back(nextseqnumA);
            }
            else
            {
                    //do nothing
            }
            
        }
        //nextseqnum++ }
        nextseqnumA++;

    }
    //else --> messages don't fall under the window so buffer them
    else
    {
        bufferqueue.push(message);
    }
}

/* called from layer 3, when a packet arrives for layer 4 */
void A_input(struct pkt ackpacketA)
{

    /*
     v1
     Core idea is to delete the LL node for which ack is received and update the subsequent node's relative timer to account for the deletion of some time as well!
     Stop the timer when this processing is going on and resume the timer. Account for the elapsed time between when the timer was earlier started and when the timer was stopped here.
     
     v2
     Core idea changed to now only mark whether acked or not. Just like in SR's window. The LL now represents the window of all packets just sent, whether they are acked or not. So in A_input if we get an ack, and it's not corrupted, we simply mark the corresponding packet as being acked. And we DON'T DELETE nodes here now. Timer interrupt does that now.
     */
    
    stoptimer(0);
    
    timer_stoptime = get_sim_time();//required to account for elapsed time of the timer.
    
    //check whether the ack is for packet that is expected to be ack else ignore it --> if (ackpacketA.acknum) is present in in (packetwindow_sequenceLL), then process if block
    //find the ack_no in the packetwindow_sequenceLL --> seq_iterator points to it
    std::list<int>::iterator seq_iterator = std::find(packetwindow_sequenceLL.begin(), packetwindow_sequenceLL.end(), ackpacketA.acknum);
    int pos = std::distance(packetwindow_sequenceLL.begin(), seq_iterator);
    
    std::list<pktofpkt>::iterator pkt_iterator = packetwindowLL.begin();
    std::advance(pkt_iterator, pos);//both would now point to corresponding nodes in their respective LLs

    bool is_expected_non_acked_ack = (seq_iterator != packetwindow_sequenceLL.end());

    //bool is_expected_non_acked_ack = (std::find(packetwindow_sequenceLL.begin(), packetwindow_sequenceLL.end(), ack_no) != packetwindow_sequenceLL.end());
    
    bool is_not_corrupt = (ackpacketA.checksum == chcksm(ackpacketA.seqnum, ackpacketA.acknum, ackpacketA.payload));
    
    if(is_expected_non_acked_ack && is_not_corrupt)
    {
        (*pkt_iterator).ackflag = 1; //mark as acked in LL
        sentpackets[ackpacketA.acknum].ackflag = 1; //mark as acked in array of pktofpkt

        
        if(ackpacketA.acknum == baseA) //if packet is baseA seq no. sender window can be moved forward. That also means being able to send new messages from the buffer(if messages exist in it)
        {
            int more_packets = 0;

            //whatever has the next ack num 0, make that base
            for(int i=baseA; i<=(baseA+N); i++)//baseA+N worst case is just outside window and would have no ack
            {
                
                if(sentpackets[i].ackflag==1)//has received ack from B
                {
                    more_packets++;
                }
                else//hasn't received ack from B
                {
                    //i has seq num for which ack is yet to be received
                    baseA = i;
                    break;
                }
            }
            
            //try to send 'more_packets' number of packets --> try why? --> if there is less than that number of messages in buffer break out of the loop!
            for(int i = 0; i<more_packets; i++)
            {
                if(bufferqueue.empty()==0)//buffer queue is not empty
                {
                    
                    ///////////////////
                    if(nextseqnumA<baseA+N)//yes mostly because the vacancy was created by shifting of baseA. still added it as an additional check to avoid errors.
                    {
                        
                        ////
                        
                        memcpy(messageA.data, bufferqueue.front().data,20); //copy the first message in the queue
                        bufferqueue.pop(); //remove the first message as it is read
                        
                        ////
                        
                        
                        if(packetwindowLL.empty()==1)//the window has no packets on the fly. It was waiting for new packets
                            //Imagine: How baseA always is lesser than nextseqnumA when messages are sent. It is equal only when all the packets sent are acked.
                        {   /*
                             This code block ensures that the LL - packetwindowLL is properly set up each time. The LL represents the sender window of the packets that are sent by and contains their other information too like when they are sent to the network and what's their relative time wrt to last sent packet. Note - last sent packet could be either the original message made into packet and sent into the network or could be could be the retransmitted packets that were not acked within the timeout
                             */
                            
                            //Make_pkt
                            datapacketA.seqnum = nextseqnumA;
                            //strcpy(datapacketA.payload, message.data);
                            memcpy(datapacketA.payload, messageA.data,20);
                            datapacketA.checksum = chcksm(datapacketA.seqnum, datapacketA.acknum, datapacketA.payload);
                            
                            //Send_pkt
                            tolayer3(0, datapacketA);
                            
                            //Create first node of the LL with timer 0.0
                            //Save pkt in LL node developed in this method
                            pktofpkt_aoutput_initial.packet = datapacketA; //Save the packet for retransmission later
                            
                            pktofpkt_aoutput_initial.relative_time = 0; // First unacked message hence the timer that would be set would be for this packet and hence this one is relatively 0.0
                            
                            sentpackets[nextseqnumA].ackflag = 0; //mark not acked
                            pktofpkt_aoutput_initial.ackflag = 0; //mark not acked in the LL
                            
                            //save the arrival_time_packet_a_output in LL node developed in this method
                            arrival_time_packet_a_output = get_sim_time();
                            pktofpkt_aoutput_initial.starttime = get_sim_time();
                            
                            //Push the LL node to the linked list at the end or beginning. Same result as the LL is empty
                            packetwindowLL.push_back(pktofpkt_aoutput_initial);// Insert a new element at the end - this is the first element in the packetwindowLL
                            packetwindow_sequenceLL.push_back(nextseqnumA);
                            
                            //Starttimer(0,timeout_value)
                            starttimer(0,timeout_value);
                            next_timeout = timeout_value;
                            
                            timer_starttime = get_sim_time(); //used in A_input() to calculate elapsed time
                            
                        }
                        else//some packets are on the fly when this message from application A arrived. Don't stop the timer!
                        {
                            //Make_pkt
                            datapacketA.seqnum = nextseqnumA;
                            //strcpy(datapacketA.payload, message.data);
                            memcpy(datapacketA.payload, messageA.data, 20);
                            datapacketA.checksum = chcksm(datapacketA.seqnum, datapacketA.acknum, datapacketA.payload);
                            
                            //Send_pkt
                            tolayer3(0, datapacketA);
                            
                            //create a new LL node at the end of LL
                            //Save pkt in LL node developed in this method
                            pktofpkt_aoutput_subsequent.packet = datapacketA; //Save the packet for retransmission later
                            
                            pktofpkt_aoutput_initial = packetwindowLL.back();
                            last_packet_starttime = packetwindowLL.back().starttime;
                            current_time  = get_sim_time();
                            pktofpkt_aoutput_subsequent.starttime = current_time;
                            pktofpkt_aoutput_subsequent.relative_time = current_time - last_packet_starttime;
                            
                            sentpackets[nextseqnumA].ackflag = 0; //mark not acked in the array
                            pktofpkt_aoutput_subsequent.ackflag = 0; //mark not acked in the LL
                            
                            //Push the LL node to the linked list at the end or beginning. Same result as the LL is empty
                            packetwindowLL.push_back(pktofpkt_aoutput_subsequent);// Insert a new node at the end
                            packetwindow_sequenceLL.push_back(nextseqnumA);
                            
                        }
                        //nextseqnum++ }
                        nextseqnumA++;
                        
                    }
                    ///////////////////
                    
                }
                else//buffer queue is empty --> no more messages to send --> break out
                {
                       break;
                }
            }//end of for loop checking for more messages and shifting baseA
        }//end of if checking if the message can indeed be fit in the new window.
        else
        {
            //nothing
        }
    }
    else
    {
        //do nothing
    }
    
    new_next_timeout = next_timeout - (timer_stoptime - timer_starttime);//if all is right, timer_stoptime - timer_startime will be lesser than next_timeout
    if(new_next_timeout<1.5)
    {
        new_next_timeout = 8;
    }

    starttimer(0,new_next_timeout);
    next_timeout = new_next_timeout;
    timer_starttime = get_sim_time();
    
}

/* called when A's timer goes off */
void A_timerinterrupt()
{
    
    /*  v2 - Timerinterrupt() now would delete the first node. and would start timer with the next node's relative_timer value.
     if nodes present in the LL
         if next node is also present
             if the first node was not acked
                 Create new packet of the first node
                 Send it to layer 3
                 Make new node with that packet, relativetime wrt to last node in LL(use it's arrivaltime to determine the new node's relativetime),capture new node's currenttime for the future next node
                 Add the new node at the end of the LL
             else//first node was acked
                 //nothing
             Delete the first node //if not acked new node is created. if acked we don't need it anymore.
     
             if node is present in the LL
                 copy the relative timer to be started from the new first node
                 Start timer with the relative timer copied
             else//no node in LL
                 //nothing
         else //only one node is present
             if the node was not acked
                 send the packet of the node to layer 3
                 create new node with relativetime = 0 and currenttime captured using get_simulation_time()
                 add the node at the end of the LL
                 starttimer with timeout value
             else//node was acked
                 //do nothing. don't set the timer.ofcourse!
             delete the first node
     else //no node in LL
         do nothing
     */
    if(packetwindowLL.empty()==0)//if there are one or more nodes
    {
        if(packetwindowLL.size()>1)//if next node is also present
        {
            if(packetwindowLL.front().ackflag!=1)//if the first node was not acked
            {
                //Create new packet of the first node
                //Access the first node of the LL
                pktofpkt_ainterrupt_first_node = packetwindowLL.front();
                //Fetch the packet
                datapacketA = pktofpkt_ainterrupt_first_node.packet;
                
                //Send it to layer 3
                //Resend it
                tolayer3(0, datapacketA);
                
                //Make new node with that packet, relativetime wrt to last node in LL(use it's arrivaltime to determine the new node's relativetime),capture new node's currenttime for the future next node
                //Create new node in LL for the resent packet
                pktofpkt_ainterrupt_last_node.packet = datapacketA;
                //pktofpkt_aoutput_initial = packetwindowLL.back(); //reusing variable 'pktofpkt_aoutput_initial' not used anyways in the next step
                last_packet_starttime = packetwindowLL.back().starttime;
                current_time  = get_sim_time();
                pktofpkt_ainterrupt_last_node.starttime = current_time;
                pktofpkt_ainterrupt_last_node.relative_time = current_time - last_packet_starttime;
                
                pktofpkt_ainterrupt_last_node.ackflag = 0;
                sentpackets[nextseqnumA].ackflag = 0; //mark not acked in the array

                //Add the node to the end of the LL
                packetwindowLL.push_back(pktofpkt_ainterrupt_last_node);// Insert the new node at the end
                packetwindow_sequenceLL.push_back(datapacketA.seqnum);
            }
            else//first node was acked
            {
                    //do nothing here. needs to be simply deleted. Being done outside this immediate conditional block
            }
            //Delete the first node //if not acked new node is created. if acked we don't need it anymore.
            packetwindowLL.pop_front();
            packetwindow_sequenceLL.pop_front();

            
            if(packetwindowLL.empty()==0)//if any node is still present in the LL
            {
                //copy the relative timer to be started from the new first node
                //Start timer with the relative timer copied
                
                //Check the relative timeout value for the next packet to timeout
                next_timeout = packetwindowLL.front().relative_time;
                
                //Start the new timer with the relative time
                starttimer(0,next_timeout);
                timer_starttime = get_sim_time();//required in A_input to account for elapsed time since when the timer was started.
            }
            else //no node in LL
            {
                //do nothing
            }
        }
        else//only one node is present
        {
            if(packetwindowLL.front().ackflag!=1)//if the node was not acked
            {
                //send the packet of the node to layer 3
                //edit the same node with relativetime = 0 and currenttime captured using get_simulation_time()
                //starttimer with timeout value
                //reset the next_timeout variable to timeout value
                
                //Access the first node of the LL
                pktofpkt_ainterrupt_first_node = packetwindowLL.front();
                //Fetch the packet
                datapacketA = pktofpkt_ainterrupt_first_node.packet;
                
                //Send it to layer 3
                //Resend it
                tolayer3(0, datapacketA);
                
                //Create new node in LL for the resent packet
                pktofpkt_ainterrupt_last_node.packet = datapacketA;
                current_time  = get_sim_time();
                pktofpkt_ainterrupt_last_node.starttime = current_time;
                pktofpkt_ainterrupt_last_node.relative_time = 0;
                
                //Add the node to the end of the LL
                packetwindowLL.push_back(pktofpkt_ainterrupt_last_node);// Insert the new node at the end
                packetwindow_sequenceLL.push_back(datapacketA.seqnum);

                //Starttimer(0,timeout_value)
                starttimer(0,timeout_value);
                next_timeout = timeout_value;
                
            }
            else
            {
                //do nothing
                //don't set the timer.ofcourse!
            }
            
            //Delete the first node //if not acked new node is created. if acked we don't need it anymore.
            packetwindowLL.pop_front();
            packetwindow_sequenceLL.pop_front();

        }
    }
    else //no node in LL
    {
            //do nothing
    }
    
}

/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init()
{
    //base=1
    baseA = 1;
    
    //nextseqnum=1
    nextseqnumA = 1;
    
    
    arrival_time_packet_a_output = 0;
    current_time = 0;
    last_packet_starttime = 0;
    next_timeout = 0;
    new_next_timeout = 0;
    //early_ack_base_compensation = 0;
    timer_starttime = 0;
    timer_stoptime = 0;
}

/* Note that with simplex transfer from a-to-B, there is no B_output() */

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt datapacketB)
{
    /*
     
     if packets sent for seqnum < base+N (or seqnum < maxexpseqB)
         if packet is within window
             if packet is base
                 mark accepted
                 mark base as the first dataless entry either in window or outside it //ofcourse if all inside window are with data dataless would be the one outside the window. Typically this would be Base+N seq num. But hey! no need to check that.
                     <imp> whatever is the next seq num without any data in it make that base!! </imp>
                 send all the message from the oldbase to newbase-1 to the application
             else if packet is non-base
                  store the packet in buffer
                 mark as received
                 send ack to the sender for the packet
         else//packet is lesser than the window. Receiver had accepted the packet earlier. So no need to send message to application
             ack it. poor sender is stuck. save him with the ack for that packet.
     else
         ye konse packets hai bhai!
     */
    
     bool is_not_corrupt = (datapacketB.checksum == chcksm(datapacketB.seqnum, datapacketB.acknum, datapacketB.payload));
    
     if(is_not_corrupt && maxseqnumB >= datapacketB.seqnum)
     {

         if(datapacketB.seqnum >= baseB)
         {
             //extract(rcvpkt,data)
             memcpy(messageB.data, datapacketB.payload, 20);
             
             //store the packet in buffer
             receivedmessages[datapacketB.seqnum].messagetoB = messageB;
             //or
             memcpy(receivedmessages[datapacketB.seqnum].datatoB,messageB.data,20);
             
             //Mark as received
             receivedmessages[datapacketB.seqnum].received_flag = 1;
             
             //send ack to the sender for the packet
             ackpacketB.acknum = datapacketB.seqnum;
             ackpacketB.checksum = chcksm(ackpacketB.seqnum, ackpacketB.acknum, ackpacketB.payload);
             //udt_send(sndpkt)
             tolayer3(1, ackpacketB);
             
             if(datapacketB.seqnum == baseB)//packet is base
             {
                 //whatever is the next seq num without any data in it make that base
                 for(int i=baseB; i<=(baseB+N); i++)//base+N worst case is just outside window and would have no data
                 {
                     
                     if(receivedmessages[i].received_flag==1)//has received packet from A
                     {
                         //deliver_data(data)
                         tolayer5(1, receivedmessages[i].datatoB);
                     }
                     else//hasn't received packet from A
                     {
                         //i has seq num for which data is yet to be received
                         baseB = i;
                         maxseqnumB = baseB + Nb - 1;
                         break;
                     }
                 }
                 
             }
             else//packet is non-base
             {
                 //Do nothing
             }
         }
         else//packet is lesser than the window. Receiver had accepted the packet earlier. So no need to send message to application. ack it. poor sender is stuck. save him with the ack for that packet.
         {
             //sndpkt = make_pkt(expectedseqnum,ACK,checksum)
             ackpacketB.acknum = datapacketB.seqnum;
             ackpacketB.checksum = chcksm(ackpacketB.seqnum, ackpacketB.acknum, ackpacketB.payload);
             
             //udt_send(sndpkt)
             tolayer3(1, ackpacketB);
         }
     }
     else
     {
         // Corrupted packet. Do nothing
     }
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init()
{
    Nb = getwinsize();
    baseB = 1;
    maxseqnumB = baseB + Nb - 1;

    strcpy(messageB.data,"arndmstringof20chars");
}
