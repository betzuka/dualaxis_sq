#define __SFR_OFFSET 0
 
#include "avr/io.h"

.global mirror


#define inputs_last r19
#define mode r20
#define outputs r21
#define dirFlipMask r22
#define debounce r23
#define debounce_count r24

#define DIR_POS_BIT     0
#define CTX_HOME_BIT 4
#define CTX_VALID_BIT 5

mirror: 
    cli;
    clr     inputs_last;
    clr     mode;
    ldi     XH, hi8(lut); //1
    ldi     dirFlipMask, 0b11110000;

mirror_poll:

    IN      debounce, PINB;
    ldi     debounce_count, 3;
    //read inputs, no mask required as we are using PINB[5:0] and since the arduino has a xtal across PINB[7:6] these will always read 0
    //note there is an inherent 0.5-1.5 cycle delay as the AVR latches the inputs, this is unavoidable
debounce:
    IN      XL, PINB; //1

    cp      XL, debounce;
    brne    mirror_poll;
    dec     debounce_count;
    brne    debounce;
    
    
    //de-dupe, this is faster than polling the pin change interrupt as that takes 3 cycles to mask the inputs, an actual interrupt handler would be even slower due to the jump table
    cp      XL, inputs_last;
    breq    mirror_poll;

    or      XL, mode;        //merge input bits with mode

    //get the output of the lookup table. This contains the new mode (or old mode if the fsm has not transitioned), a bit to indicate home should be reflected back to the controller, a bit
    //that tells us whether the input was valid and 4 step bits, two for each motor
    ld      outputs, X; //2
    
    //check valid. It is highly unlikely this ever occurs as DDCS switches differential outputs atomically using FPGA, on the scope they switch simultaneously.
    sbrs    outputs, CTX_VALID_BIT;
    rjmp    mirror_poll;    //we are in an invalid state because the step/dir inputs are not differentially settled so do nothing and return to start

    //save new mode and status bits before we butcher outputs
    mov     mode, outputs;
   
    andi    outputs, 0b00001111; //retain step bits
    ori     outputs, 0b01010000; //copy in direction bits (note we guess direction and flip later if wrong)
    
    sbrs    XL, DIR_POS_BIT;
    eor     outputs, dirFlipMask;   //guess was wrong, flip direction bits to match input direction
    
    out     PORTD, outputs;

    //AND BREATHE! Motor outputs have been written, we now have a bit of time before the inputs next change.

    //set or clear home output, messy stuff! seemingly no better way to write single bit to a port
    sbrs    mode, CTX_HOME_BIT;
    sbi     PORTC, 0;
    sbrc    mode, CTX_HOME_BIT;
    cbi     PORTC, 0;

    andi    mode, 0b11000000;   //clear status bits retaining mode

    mov     inputs_last, XL;            //inputs have changed, save them for next de-dupe
    andi    inputs_last, 0b00111111;    //clear off old mode
    
    rjmp    mirror_poll;

