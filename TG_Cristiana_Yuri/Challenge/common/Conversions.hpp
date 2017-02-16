#pragma once

#include <cmath>
#include <ledAuxiliarFunctions.hpp>


namespace Conversions
{
    #define FIELD_HEIGHT 9000
    #define FIELD_WIDHT 6000

    int decimal_octal(int n); /* Function to convert decimal to octal */

    int octal_decimal(int n); /* Function to convert octal to decimal */
    
    void MoveCenterToCorner(int *x, int *y);
    
    void MoveCornerToCenter(int *x, int *y);
    

    //###################FIELD#######################
    //      ________________________________  x = 9000
    //     |           Transmiter           |
    //     |  Quadrant 8   |  Quadrant 7    |
    //     |               |                |
    //     |_______________|________________| x = 7750
    //     |               |                |
    //     |  Quadrant 6   |  Quadrant 5    |
    //     |               |                |
    //     |_______________|________________| x = 4500
    //     |               |                |
    //     |  Quadrant 4   |  Quadrant 3    |
    //     |               |                |
    //     |_______________|________________| x = 2250
    //     |               |                | ^
    //     |  Quadrant 2   |  Quadrant 1    | | X
    //     |               |                | |
    //     |___________Reciever_____________| |
    // y = 6000         y = 3000    <--------- (0,0)
    //                                 Y
    //################################################

    void ConvertToQuadrant(int *x, int *y, int *quadrant);
    

    void ConvertFromQuadrant(int *x, int *y, int quadrant);

    std::vector<int> num2vec(int x, int y, int quadrant);

    void vec2num(int *x, int *y, int *quadrant, std::vector<int> vec);

    enum Color { OFF = 0, RED , WHITE, BLUE, YELLOW, CYAN, MAGENTA, GREEN };

    void transmitLED(int digit);

    void changeChestLed(int color);
       
    void turnOFF();
}


