//typedef struct x {
//    int member_a;
//    int member_b;
//} x;

```
#define MAX_RC_CHANNELS 10

typedef struct x {

  // Signature (1)[0]
  uint8_t   setup;          // Byte to identify if already setup

  // Menu adjustable items
  // RC settings (8)[1]
  int8_t    channelOrder[MAX_RC_CHANNELS];  // Assign channel numbers to hard-coded channel order
                                            // OpenAeroVtol uses Thr, Ail, Ele, Rud, Gear, Aux1, Aux2, Aux3
                                            // THROTTLE will always return the correct data for the assigned throttle channel
                                            // AILERON will always return the correct data for the assigned aileron channel
                                            // ELEVATOR will always return the correct data for the assigned elevator channel
                                            // RUDDER will always return the correct data for the assigned rudder channel
} x_t;


void setup() {

  x *s;
  s = (x *)malloc(sizeof(x));
  char *base;
  size_t offset;
  int *b;

  Serial.begin(115200);

  // initialize both members to known values
  s->setup = 1;
  s->channelOrder[0] = 0;
  s->channelOrder[1] = 1;
  s->channelOrder[2] = 2;

  // get base address
  base = (char *)s;

  // and the offset to member_b
  offset = offsetof(x, channelOrder[2]);

  // Compute address of member_b
  b = (int *)(base+offset);

  // write to member_b via our pointer
  *b = 10;

  // print out via name, to show it was changed to new value.
  Serial.printf("%d\n", s->channelOrder[2]);

}

void loop() {
  // put your main code here, to run repeatedly:

}
```