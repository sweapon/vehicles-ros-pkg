#ifndef BUDDYMSGS_H
#define BUDDYMSGS_H

#define MASTER_ID 0
#define VISION_ID 1
#define BATTERY_ID 2

struct LData
{
  enum {MSG_TYPE=0, ELEM=1, LEN=4};
  enum {LEAK=0};
};

struct LTData
{
  enum {MSG_TYPE=1, ELEM=2, LEN=8};
  enum {LEAK=0, TEMP};
};

struct LPData
{
  enum {MSG_TYPE=2, ELEM=2, LEN=8};
  enum {LEAK=0, PRESSURE};
};

struct LPTData
{
  enum {MSG_TYPE=3, ELEM=3, LEN=12};
  enum {LEAK=0, PRESSURE, TEMP};
};


#endif
