
//#include <elapsedMillis.h>
//#include <qpOASES.hpp>
//#include <qp_sim.h>


static uint32_t tTime;
static uint32_t trigger = 0;

/*
USING_NAMESPACE_QPOASES

//elapsedMillis timerMPC; 

real_t H[2*2] = { 1.0, 0.0, 0.0, 0.5 };
real_t A[1*2] = { 1.0, 1.0 };
real_t g[2] = { 1.5, 1.0 };
real_t lb[2] = { 0.5, -2.0 };
real_t ub[2] = { 5.0, 2.0 };
real_t lbA[1] = { -1.0 };
real_t ubA[1] = { 2.0 };


real_t g_new[2] = { 1.0, 1.5 };
real_t lb_new[2] = { 0.0, -1.0 };
real_t ub_new[2] = { 5.0, -0.5 };
real_t lbA_new[1] = { -2.0 };
real_t ubA_new[1] = { 1.0 };

int_t nWSR = 10;

real_t xOpt[2];
real_t yOpt[2+1];

int32_t init_case = 1;
*/

//QProblem example( 2,1 );


void setup() {
    /* serial to display data */
    Serial.begin(115200);
    while(!Serial) {}
    pinMode(LED_BUILTIN, OUTPUT);    

    //QProblem example( 2,1 );
    //Options options;
    //example.setOptions( options );
}

void loop() {
    uint32_t t = millis();
    if ((t-tTime) >= 1000 ) {
        if (trigger == 1) { 
            digitalWrite(LED_BUILTIN, HIGH); 
            trigger = 0;
        }
        else {
            digitalWrite(LED_BUILTIN, LOW);
            trigger = 1;
        }       
        tTime = t;
        Serial.println(trigger);
    }

        
        /*
        if (init_case == 1) {
            init_case = 0;
            
            example.init( H,g,A,lb,ub,lbA,ubA, nWSR );            
            example.getPrimalSolution( xOpt );
            example.getDualSolution( yOpt );
            printf( "\nxOpt = [ %e, %e ];  yOpt = [ %e, %e, %e ];  objVal = %e\n\n", 
                    xOpt[0],xOpt[1],yOpt[0],yOpt[1],yOpt[2],example.getObjVal() );
        }
        */
       
        /*
        nWSR = 10;
        example.hotstart( g_new,lb_new,ub_new,lbA_new,ubA_new, nWSR );

        
        example.getPrimalSolution( xOpt );
        example.getDualSolution( yOpt );

        Serial.print(xOpt[0]);                         Serial.print("   ");  
        Serial.println(xOpt[1]);                                 
        */

        //digitalWrite(13, LOW);




        /*
        printf( "\nxOpt = [ %e, %e ];  yOpt = [ %e, %e, %e ];  objVal = %e\n\n", 
                xOpt[0],xOpt[1],yOpt[0],yOpt[1],yOpt[2],example.getObjVal() );

        example.printOptions();
        */
        /*example.printProperties();*/

        /*getGlobalMessageHandler()->listAllMessages();*/

        //timerMPC = 0;   
        //Serial.print(timerMPC);
      
}

