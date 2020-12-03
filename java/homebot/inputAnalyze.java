//=======================================================================================
public class inputAnalyze
{
	boolean forwardStateOn=false; 
    boolean reverseStateOn=false;  
    boolean leftStateOn=false;
    boolean rightStateOn=false;
    float joyStickTurnAnalog=0;
    float joyStickSpeed=0;
 public inputAnalyze(){} 
 //=======================================================================================  
 public float getJoyStickYAnalog(){
	 return joyStickSpeed;
 }
 //=======================================================================================
 public float getJoyStickXAnalog(){
	 return joyStickTurnAnalog;
 }
 //=======================================================================================
 public String inputAnalyze(String inputString,String inputJoyStick){
    String outputString=inputString;
    if (inputJoyStick!=null){
      int len=inputJoyStick.length();
      int locComma=inputJoyStick.indexOf(",",0)+2;
      if (len>locComma){
		String shortString=inputJoyStick.substring(locComma,len);
        
        switch(shortString)
        {		
		  case "Button 3 changed to On":		    
		    if (!forwardStateOn){				
				forwardStateOn=true;
			    outputString="w$";
			}
		    break;
		  case "Button 3 changed to Off":		   
		    if (forwardStateOn){
				forwardStateOn=false;
			    outputString="s$";
			}
		    break;
		  case "Button 0 changed to On":		    
		    if (!reverseStateOn){				
				reverseStateOn=true;
			    outputString="x$";
			}
		    break;
		  case "Button 0 changed to Off":		   
		    if (reverseStateOn){
				reverseStateOn=false;
			    outputString="s$";
			} 
		  case "Button 2 changed to On":		    
		    if (!leftStateOn){				
				leftStateOn=true;
			    //outputString="w$";
			}
		    break;
		  case "Button 2 changed to Off":		   
		    if (leftStateOn){
				leftStateOn=false;
			    outputString="a$";
			}
		    break;
		  case "Button 1 changed to On":		    
		    if (!rightStateOn){				
				rightStateOn=true;
			    //outputString="x$";
			}
		    break;
		  case "Button 1 changed to Off":		   
		    if (rightStateOn){
				rightStateOn=false;
			    outputString="d$";
			} 		  	 	   
		  default:
		    if (inputJoyStick.contains("Y Rotation changed to")){
			  String magnitude=inputJoyStick.substring(locComma+22,len);
			  System.out.println(magnitude);
			  joyStickSpeed=Float.parseFloat(magnitude);
			  //System.out.println(joyStickSpeed);
			  outputString="JY"+ magnitude + "$";
			} 
			if (inputJoyStick.contains("X Rotation changed to")){
			  String turnString=inputJoyStick.substring(locComma+22,len);
			  System.out.println(turnString);
			  joyStickTurnAnalog=Float.parseFloat(turnString);
			  //System.out.println(joyStickTurnAnalog);
			  outputString="JX"+ turnString + "$";
			} 
			if (inputJoyStick.contains("Z Axis changed to")){
			  String turnString=inputJoyStick.substring(locComma+18,len);
			  System.out.println(turnString);
			  joyStickTurnAnalog=Float.parseFloat(turnString);
			  //System.out.println(joyStickTurnAnalog);
			  outputString="JZ"+ turnString + "$";
			}
	    }
	  }
	}      	
	return outputString;
  }  
}  
