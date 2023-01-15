
/**
 * 
 * INPUTS PORT B
 * 0    PB0     D8      DIR_POS
 * 1    PB1     D9      DIR_NEG
 * 2    PB2     D10     STEP_POS
 * 3    PB3     D11     STEP_NEG
 * 4    PB4     D12     HOME_A
 * 5    PB5     D13     HOME_B
 * 
 * OUTPUTS PORT D
 * 0    PD0     RXD/D0  DIR_A_POS
 * 1    PD1     TXD/D1  DIR_A_NEG
 * 2    PD2     D2      DIR_B_POS
 * 3    PD3     D3      DIR_B_NEG
 * 4    PD4     D4      STEP_A_POS
 * 5    PD5     D5      STEP_A_NEG
 * 6    PD6     D6      STEP_B_POS
 * 7    PD7     D7      STEP_B_NEG
 * 
 * OUTPUTS PORT C
 * 0    PC0     A0      HOME_OUT
 * 1    PC1     A1      ESTOP_OUT
 * 4    PC4     A4      SDA (display)
 * 5    PC5     A5      SCL (display)
 * 
 */

#define HOME_OUT        0   //PORT C BIT 0 -> PIN A0
#define ESTOP_OUT       1   //PORT C BIT 1 -> PIN A1

#define HOME_SHIFT      4

#define DIR_SHIFT      0

#define STEP_SHIFT      2

#define DIR_POS_BIT     0
#define DIR_NEG         1
#define STEP_POS        2
#define STEP_NEG        3
#define HOME_A          4   
#define HOME_B          5

#define LL              0
#define LH              1
#define HL              2
#define HH              3

#define DIFF_LOW        HL
#define DIFF_HIGH       LH

#define HOME_DIR        DIFF_LOW
#define STEP_HIGH       DIFF_HIGH
#define STEP_LOW        DIFF_LOW

#define HOME_A_MASK     1
#define HOME_B_MASK     2
#define HOME_BOTH_HIT   LL
#define HOME_BOTH_REL   HH
#define HOME_OUT_HIT    1
#define HOME_OUT_REL    0

//steps on rising edge reverse if required
#define STEP_EDGE_1    STEP_LOW
#define STEP_EDGE_2    STEP_HIGH

#define MAX_OVERDRIVE   100

#define SET_PIN(PORT, BIT) (PORT |= (1<<BIT))
#define CLEAR_PIN(PORT, BIT) (PORT &= ~(1<<BIT))


uint8_t lut [256] __attribute__((aligned(0x100)));

#define MODE_NORMAL 0
#define MODE_HOME 1
#define MODE_LIMIT 2

#define CTX_HOME_BIT 4
#define CTX_VALID_BIT 5

void setup() {

    //populate lut
    /**
     * ctx format:
     *  [7:6]   mode (valid modes 00,01,10)
     *  [5]     Home switch B (active low)
     *  [4]     Home switch A (active low)
     *  [3]     Step neg
     *  [2]     Step pos
     *  [1]     Dir neg
     *  [0]     Dir pos
     *  
     * lookup entry format: 
     *  [7:6]   new mode (state machine advance)
     *  [5]     Valid - 1=the ctx was valid so proceed, 0=the ctx was invalid due to non-differential input pairs so ignore
     *  [4]     Home output - 1=Signal home to controller, 0=clear home to controller
     *  [3]     B step neg
     *  [2]     B step pos
     *  [1]     A step neg
     *  [0]     A step pos
     *  
     *  
     *  
     */ 
        
    
    for (int ctx=0;ctx<256;ctx++) {

        //parse ctx
        uint8_t mode = (ctx & B11000000) >> 6;

        bool bHit    = (ctx & B00100000)==0;
        bool aHit    = (ctx & B00010000)==0;

        uint8_t step = (ctx & B00001100) >> 2;
        uint8_t dir  = (ctx & B00000011);
        
        uint8_t entry = 0;

        //validate
        if (mode<=2 && (step==LH || step==HL) && (dir==LH || dir==HL)) {
            //we will form a valud entry

            //true if we are travelling towards home
            bool dirHome = dir==HOME_DIR;

            bool suppressA = false;
            bool suppressB = false;
            bool setHome = false;

            //advance state machine
            if (!(aHit || bHit)) {
                //both switches open
                mode = MODE_NORMAL;
            } else if (mode==MODE_NORMAL) {
                //either or both switches closed, determine mode depending on direction of travel
                if (dirHome) {
                    //travelling in negative direction towards home
                    mode = MODE_HOME;
                } else {
                    //travelling in positive direction away from home
                    mode = MODE_LIMIT;
                }
            }

            //determine outputs
            if (mode==MODE_LIMIT) {
                //at limit end of machine and either or both switches closed
                //signal home to controller
                suppressA = true;
                suppressB = true;
                setHome = true;
            } else if (mode==MODE_HOME) {
                //at home end of machine and either or both switches closed
                if (dirHome) {
                    //travelling in negative direction towards home
                    if (aHit) {
                        //home A closed, suppress step to A
                        suppressA = true;
                    }
                    if (bHit) {
                        //home B closed, suppress step to B
                        suppressB = true;
                    }
                    if (aHit && bHit) {
                        //don't signal home to controller until both switches are closed
                        setHome = true;
                    } 
                } else {
                    //travelling in positive direction away from home
                    if (!aHit) {
                        //home A not closed, suppress step
                        suppressA = true;
                    }
                    if (!bHit) {
                        //home B not closed, suppress step
                        suppressB = true;
                    }
                    //signal home to controller whilst either or both switches are hit 
                    setHome = true;
                }
            }

            //build entry

            
            //encode B step
            if (suppressB) {
                entry |= (STEP_LOW << 2);
            } else {
                entry |= (step << 2);
            }
            //encode A step
            if (suppressA) {
                entry |= (STEP_LOW << 0);
            } else {
                entry |= (step << 0);
            }
            //encode home
            if (setHome) {
                entry |= (1<<CTX_HOME_BIT);
            }
            //set valid bit
            entry |= (1<<CTX_VALID_BIT);
            
        } 

        //pass mode back, even if invalid (pass original mode)
        entry |= (mode<<6);
            
        lut[ctx] = entry;
        
    }

    
    //disable uart so we can use PORT D 0:1
    UCSR0B = 0;

    //switch port D to high impedence (input) will be set to output on first iteration
    DDRD = B00000000;
    //temp all outs
    DDRD = B11111111;
    
    //set PORT C 0:1 as outputs
    
    DDRC |= B00000011;
    CLEAR_PIN(PORTC, HOME_OUT);
    CLEAR_PIN(PORTC, ESTOP_OUT);
        
    //set PORT B 0:5 as inputs
    DDRB &= B11000000;
    //enable PORT B pullups 
   // DDRB |= B00111111;   

}

extern "C" {
    void mirror();
}

void loop() {
    mirror();
}
