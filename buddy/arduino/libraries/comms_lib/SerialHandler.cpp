#include <SerialHandler.h>


SerialHandler::SerialHandler(void(*callback)(byte*,byte,byte,byte)):
  ring_idx(0), 
  data_idx(0),
  data_len(0),
  callback(callback)
{
  for(int i=0; i<SYNC_LEN; ++i) ring[i]=0;
}

void SerialHandler::syncHandler(byte data)
{
   ring[ring_idx++] = data;
	     
   //If ring buffer is full
   if (ring_idx == SYNC_LEN)
   {
      if (isSynced())
      {
	//Switch to data mode
        comms_state = DATA;
	ring_idx = 0;
        data_idx = 0;
        data_len = ring[LEN_LOC];
	if (data_len > MAX_PAYLOAD)
	{
	  comms_state = SYNC;
	}
      }
      else
      { 
        //Move buffer one to the left
        for(int i=0;i<SYNC_LEN-1; ++i) 
	   ring[i] = ring[i+1];
        //Reset index for a new single byte read
        ring_idx = SYNC_LEN-1;
      }
   }
}

void SerialHandler::dataHandler(byte data)
{
   payload[data_idx] = data;
   if (++data_idx == data_len)
   {
     //Execute the external handler callback
     (*callback)(payload, data_len, ring[SENDER_LOC], ring[TYPE_LOC]); 
     //Wait for next message
     comms_state = SYNC;
     ring_idx = 0;
   }
}
