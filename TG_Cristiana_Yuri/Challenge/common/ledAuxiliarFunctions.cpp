//#include <includeMapBehavior.hpp>
#include <ledAuxiliarFunctions.hpp>

namespace LEDAux
{
	ActionCommand::LED changeColor (int Chest, int REye , int LEye)
	{
	    ActionCommand::LED led;

	    switch(Chest)
	    {
	    	case 0:
                led.chestButton = ActionCommand::rgb(false, false, false); //off
		    	break;
		    case 1:
                led.chestButton = ActionCommand::rgb(true, false, false); //red
		    	break;
		    case 2:
		    	led.chestButton = ActionCommand::rgb(true, true, true); //White
		    	break;
		    case 3:
                led.chestButton = ActionCommand::rgb(false, false, true); //blue
		    	break;
		    case 4:
                led.chestButton = ActionCommand::rgb(true, true, false); //Yellow
		    	break;
		    case 5:
			    led.chestButton = ActionCommand::rgb(false, true, true); //Cyan
		    	break;
		    case 6:
                led.chestButton = ActionCommand::rgb(true, false, true); //Magenta                
		    	break;
		    case 7:
                led.chestButton = ActionCommand::rgb(false, true, false); //green
		    	break;
	    }

	    switch(REye)
	    {
	    	case 0:
                led.rightEye = ActionCommand::rgb(false, false, false); //off
		    	break;
		    case 1:
                led.rightEye = ActionCommand::rgb(true, false, false); //red
		    	break;
		    case 2:
		    	led.rightEye = ActionCommand::rgb(true, true, true); //White
		    	break;
		    case 3:
                led.rightEye = ActionCommand::rgb(false, false, true); //blue
		    	break;
		    case 4:
                led.rightEye = ActionCommand::rgb(true, true, false); //Yellow
		    	break;
		    case 5:
			    led.rightEye = ActionCommand::rgb(false, true, true); //Cyan
		    	break;
		    case 6:
                led.rightEye = ActionCommand::rgb(true, false, true); //Magenta                
		    	break;
		    case 7:
                led.rightEye = ActionCommand::rgb(false, true, false); //green
		    	break;
	    }

	    switch(LEye)
	    {
	    	case 0:
                led.leftEye = ActionCommand::rgb(false, false, false); //off
		    	break;
		    case 1:
                led.leftEye = ActionCommand::rgb(true, false, false); //red
		    	break;
		    case 2:
		    	led.leftEye = ActionCommand::rgb(true, true, true); //White
		    	break;
		    case 3:
                led.leftEye = ActionCommand::rgb(false, false, true); //blue
		    	break;
		    case 4:
                led.leftEye = ActionCommand::rgb(true, true, false); //Yellow
		    	break;
		    case 5:
			    led.leftEye = ActionCommand::rgb(false, true, true); //Cyan
		    	break;
		    case 6:
                led.leftEye = ActionCommand::rgb(true, false, true); //Magenta                
		    	break;
		    case 7:
                led.leftEye = ActionCommand::rgb(false, true, false); //green
		    	break;

	    }
	    
	    return led;
	}
}
