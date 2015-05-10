#ifndef SERIALHANDLER_H
#define SERIALHANDLER_H
#include <Arduino.h>

#define INTERCOMMS_BAUD 9600

//Handler message of type:
// |SYNCB1|SYNCB2|LEN|SENDER|TYPE|PAYLOAD|
class SerialHandler
{
 public:
   enum {SYNC_LEN=5, LEN_LOC=2, SENDER_LOC, TYPE_LOC};
   enum {SYNCB1=0xFF, SYNCB2 = 0xFF};
   enum {MAX_PAYLOAD=12};
   enum {NUM_MSG=2};
    
   //Main constructor
   SerialHandler(void(*callback)(byte*,byte,byte,byte));
   
   //Byte processor
   template <class SerialType>
   void readSerial(SerialType* serial)
   {
     //While we have something in the buffer
     while (serial->available())
     {
        //Read one byte
    	byte dat = serial->read();

     	//Test state
    	if (comms_state == SYNC)
	{  
	   syncHandler(dat);
        }
	else if (comms_state == DATA)
        {
	   dataHandler(dat);
        }
	else
	{
	  //Unknown state
          comms_state = SYNC; 
	  ring_idx = 0;
	}
     }
   }

   template <class SerialType>
   static void writeSerial(SerialType* serial, byte* buf, byte len, byte sender, byte msg_type)
   {
     byte header[SYNC_LEN]={SYNCB1,SYNCB2,len,sender,msg_type};
     serial->write(header, SYNC_LEN);
     serial->write(buf,len);
   }    

 private:
   //Synchornization handler
   void syncHandler(byte data);
   //Data handler
   void dataHandler(byte data);
   //Test synchronization
   bool isSynced(){return (ring[0] == SYNCB1) && (ring[1] == SYNCB2);};
   //Communication states
   enum {SYNC=0, DATA=1};
   //The communication state
   byte comms_state;
   //The ring buffer for sync
   byte ring[SYNC_LEN];
   //The current ring index
   byte ring_idx;
   //The current data index
   byte data_idx;
   byte data_len;
   //The message buffer
   byte payload[MAX_PAYLOAD];
   //Payload callback callback(byte* buf, byte len, byte sender, byte msg_type)
   void (*callback)(byte*, byte, byte, byte);
};

#endif
