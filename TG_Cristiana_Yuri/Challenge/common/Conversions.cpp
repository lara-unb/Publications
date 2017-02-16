#include <Conversions.hpp>


namespace Conversions
{
    int decimal_octal(int n) /* Function to convert decimal to octal */
    {
        int rem, i=1, octal=0;
        while (n!=0)
        {
            rem=n%8;
            n/=8;
            octal+=rem*i;
            i*=10;
        }
        return octal;
    }

    int octal_decimal(int n) /* Function to convert octal to decimal */
    {
        int decimal=0, i=0, rem;
        while (n!=0)
        {
            rem = n%10;
            n/=10;
            decimal += rem*pow(8,i);
            ++i;
        }
        return decimal;
    }

    void MoveCenterToCorner(int *x, int *y)
    {
        *x += FIELD_HEIGHT/2;
        *y += FIELD_WIDHT/2;
    }
    void MoveCornerToCenter(int *x, int *y)
    {
        *x -= FIELD_HEIGHT/2;
        *y -= FIELD_WIDHT/2;
    }

    //###################FIELD#######################
    //      ________________________________  x = 9000
    //     |           Transmiter           |
    //     |  Quadrant 7   |  Quadrant 6    |
    //     |               |                |
    //     |_______________|________________| x = 7750
    //     |               |                |
    //     |  Quadrant 5   |  Quadrant 4    |
    //     |               |                |
    //     |_______________|________________| x = 4500
    //     |               |                |
    //     |  Quadrant 3   |  Quadrant 2    |
    //     |               |                |
    //     |_______________|________________| x = 2250
    //     |               |                | ^
    //     |  Quadrant 1   |  Quadrant 0    | | X
    //     |               |                | |
    //     |___________Reciever_____________| |
    // y = 6000         y = 3000    <--------- (0,0)
    //                                 Y
    //################################################

    void ConvertToQuadrant(int *x, int *y, int *quadrant)
    {
        if(*y < FIELD_WIDHT/2)
        {
            if(*x < FIELD_HEIGHT/4)
            {
                *quadrant = 0;
            }
            else if(*x < FIELD_HEIGHT/2)
            {
                *quadrant = 2;
                *x -= FIELD_HEIGHT/4;
            }
            else if(*x < FIELD_HEIGHT*3/4)
            {
                *quadrant = 4;
                *x -= FIELD_HEIGHT/2;
            }
            else
            {
                *quadrant = 6;
                *x -= FIELD_HEIGHT*3/4;
            }
        }
        else
        {
            *y -= FIELD_WIDHT/2;
            if(*x < FIELD_HEIGHT/4)
            {
                *quadrant = 1;
            }
            else if(*x < FIELD_HEIGHT/2)
            {
                *quadrant = 3;
                *x -= FIELD_HEIGHT/4;
            }
            else if(*x < FIELD_HEIGHT*3/4)
            {
                *quadrant = 5;
                *x -= FIELD_HEIGHT/2;
            }
            else
            {
                *quadrant = 7;
                *x -= FIELD_HEIGHT*3/4;
            }
        }
    }

    void ConvertFromQuadrant(int *x, int *y, int quadrant)
    {
        switch (quadrant)
        {
        case 0:
            break;
        case 1:
            *y += FIELD_WIDHT/2;
            break;
        case 2:
            *x += FIELD_HEIGHT/4;
            break;
        case 3:
            *x += FIELD_HEIGHT/4;
            *y += FIELD_WIDHT/2;
            break;
        case 4:
            *x += FIELD_HEIGHT/2;
            break;
        case 5:
            *x += FIELD_HEIGHT/2;
            *y += FIELD_WIDHT/2;
            break;
        case 6:
            *x += FIELD_HEIGHT*3/4;
            break;
        case 7:
            *x += FIELD_HEIGHT*3/4;
            *y += FIELD_WIDHT/2;
            break;
        default:
            break;
        }
    }

    std::vector<int> num2vec(int x, int y, int quadrant)
    {
        std::vector<int> vec(9);

        vec[0]= quadrant;

        // x digits;
        vec[1] = x/1000;
        x = x-vec[1]*1000;
        vec[2] = x/100;
        x = x-vec[2]*100;
        vec[3] = x/10;
        x = x-vec[3]*10;
        vec[4] = x;

        //y digits
        vec[5] = y/1000;
        y = y-vec[5]*1000;
        vec[6] = y/100;
        y = y-vec[6]*100;
        vec[7] = y/10;
        y = y-vec[7]*10;
        vec[8] = y;

        return vec;
    }

    void vec2num(int *x, int *y, int *quadrant, std::vector<int> vec)
    {
        *quadrant = vec[0];

        *x = 1000*vec[1]+100*vec[2]+10*vec[3]+vec[4];

        *y = 1000*vec[5]+100*vec[6]+10*vec[7]+vec[8];
    }

    void transmitLED(int digit)
    {
        
        Command command;
        
        
        switch(digit)
        {
            case 0:
                command.leds = LEDAux::changeColor(YELLOW, YELLOW, YELLOW);
                break;
            case 1:
                command.leds = LEDAux::changeColor(YELLOW, YELLOW, CYAN);
                break;
            case 2:
                command.leds = LEDAux::changeColor(YELLOW, CYAN, YELLOW);
                break;
            case 3:
                command.leds = LEDAux::changeColor(YELLOW, CYAN, CYAN);
                break;
            case 4:
                command.leds = LEDAux::changeColor(CYAN, YELLOW, YELLOW);
                break;
            case 5:
                command.leds = LEDAux::changeColor(CYAN, YELLOW, CYAN);
                break;
            case 6:
                command.leds = LEDAux::changeColor(CYAN, CYAN, YELLOW);
                break;
            case 7:
                command.leds = LEDAux::changeColor(CYAN, CYAN, CYAN);
                break;

        }
        //command.body = MotionAux::Stand();
        MotionAux::SendCommand(command);

    }

    void changeChestLed(int color)
    {
        Command command;
        command.leds = LEDAux::changeColor(color, color, color);
        //command.body = MotionAux::Stand();
        MotionAux::SendCommand(command);
    }

    void turnOFF()
    {   
        Command command;
        command.leds = LEDAux::changeColor(RED, RED, RED);
        //command.body = MotionAux::Stand();
        MotionAux::SendCommand(command);
    }
}


