#include <stdio.h>
#include <math.h>

#define NUM_SAMPLES (1 << 9) 

#define QUARTER_SINE 128 //(NUM_SAMPLES/4)

#define PEAK_AMPLITUDE (((1 << 16)-1 ) /2)

static signed short SINE_ARRAY[QUARTER_SINE+1];

signed short getSine(float degrees)
{
    unsigned int quadrant;

    while (degrees > 360.0) degrees -= 360.0;
    degrees += .5;

    quadrant = ((degrees) / 90.0);
    float ninety = degrees - (float) (quadrant * 90);

    int index;

    switch (quadrant)
    {
        case 0:
        case 2:
            index = (float) (QUARTER_SINE-1) * ninety/90.0;        
        break;

        case 1:
        case 3:
            index = QUARTER_SINE - (float) (QUARTER_SINE-1) * ninety/90.0;        
        break;
    }

    //printf("                   deg = %5.3f quadrant= %d index = %d\n", degrees, quadrant, index);

    switch (quadrant)
    {
        case 0:
        case 1:
            return SINE_ARRAY[index];
        case 2:
        case 3:
            return -SINE_ARRAY[index];

    }
    return 0;
}



void setup_sine (void)
{
    float amp;
	printf("QUARTER_SINE = %d\n", QUARTER_SINE);
	
    for (int i = 0 ; i < QUARTER_SINE; i++)
    {
		//if (i == QUARTER_SINE) return;
        double degrees = ( (float) i / (float) QUARTER_SINE) * 90.0;
        amp = (double) PEAK_AMPLITUDE * sin(M_PI * degrees/180. );
        SINE_ARRAY[i] = (signed short) amp;
        printf (" i = %3d  degrees = %4.3f  %5d 0x%4X \n", i, degrees, (signed short)SINE_ARRAY[i],  SINE_ARRAY[i]);
    }

	printf("\n");
    for (int i = 0; i < 360; i+=5)
    {
        printf ("%3d %5d 0x%4X\n", i,(int) getSine(i), (int) getSine(i));
    }


}
